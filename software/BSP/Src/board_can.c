#include "board_can.h"
#include "board.h"
#include "can_config.h"
#include "stm32f1xx.h"
#include <stddef.h>

/* RM0008 bxCAN: PCLK1 / [BRP * (1 + BS1 + BS2)].
 * At 36 MHz, 18 time quanta and BRP=2/4/8/16 give 1M/500k/250k/125k.
 * BS1=13, BS2=4, SJW=1; the sample point is 14/18 = 77.78%.
 */
#define CAN_PCLK_HZ             36000000u
#define CAN_TIME_QUANTA         18u
#define CAN_PRESCALER           (CAN_PCLK_HZ / (CAN_BITRATE * CAN_TIME_QUANTA))
#define CAN_RX_SIZE             32u
#define CAN_INIT_TIMEOUT_CYCLES (BOARD_CORE_HZ / 100u)
#define CAN_TX_TIMEOUT_MS       20u
#define CAN_RX_PIN              (1u << 11)
#define CAN_TX_PIN              (1u << 12)

#if CAN_NODE_ID < 1 || CAN_NODE_ID > 63
#error CAN_NODE_ID must be in 1..63
#endif
#if CAN_BITRATE != 125000u && CAN_BITRATE != 250000u && CAN_BITRATE != 500000u && CAN_BITRATE != 1000000u
#error Unsupported bxCAN bitrate
#endif
#if CAN_COMMAND_ID > 0x7ffu || CAN_ESTOP_ID > 0x7ffu || CAN_COMMAND_ID == CAN_ESTOP_ID
#error CAN receive identifiers must be distinct standard identifiers
#endif
#if CAN_PCLK_HZ % (CAN_BITRATE * CAN_TIME_QUANTA) != 0
#error CAN timing is not an exact division of PCLK1
#endif

static volatile BoardCanStatus status;
static BoardCanFrame rx_queue[CAN_RX_SIZE];
static volatile uint8_t rx_head, rx_tail;
static volatile uint8_t tx_pending, tx_aborting;
static uint32_t tx_started_ms[3];
static uint8_t tx_lifetime_ms[3];

static bool wait_inak(bool set)
{
    uint32_t started = Board_CycleCount();
    uint32_t attempts = 4000000u;
    while (((CAN1->MSR & CAN_MSR_INAK) != 0u) != set) {
        if ((uint32_t)(Board_CycleCount() - started) >= CAN_INIT_TIMEOUT_CYCLES ||
            --attempts == 0u) return false;
    }
    return true;
}

static bool init_failed(void)
{
    CAN1->IER = 0u;
    CAN1->MCR |= CAN_MCR_INRQ;
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_DisableIRQ(CAN1_SCE_IRQn);
    status.initialized = false;
    return false;
}

/* Caller holds the short BSP critical section or is a CAN ISR. The two CAN
 * interrupt vectors use equal preemption priority and cannot nest each other.
 */
static void refresh_status(void)
{
    uint32_t esr = CAN1->ESR;
    uint32_t tsr = CAN1->TSR;
    uint32_t now = Board_Millis();
    bool bus_off = (esr & CAN_ESR_BOFF) != 0u;
    if (bus_off && !status.bus_off) ++status.bus_off_count;
    status.bus_off = bus_off;
    status.error_passive = (esr & CAN_ESR_EPVF) != 0u;
    status.tx_error_count = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    status.rx_error_count = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
    uint8_t lec = (uint8_t)((esr & CAN_ESR_LEC) >> CAN_ESR_LEC_Pos);
    if (lec != 0u && lec != 7u) status.last_error_code = lec;

    for (unsigned mailbox = 0u; mailbox < 3u; ++mailbox) {
        uint8_t bit = (uint8_t)(1u << mailbox);
        uint32_t shift = mailbox * 8u;
        uint32_t completed = CAN_TSR_RQCP0 << shift;
        if ((tsr & completed) != 0u) {
            if ((tx_pending & bit) != 0u) {
                if ((tsr & (CAN_TSR_TXOK0 << shift)) != 0u) ++status.tx_count;
                else ++status.tx_dropped;
            }
            tx_pending = (uint8_t)(tx_pending & (uint8_t)~bit);
            tx_aborting = (uint8_t)(tx_aborting & (uint8_t)~bit);
            CAN1->TSR = completed; /* W1C clears RQCP and its completion flags. */
        }
        if ((tx_pending & bit) != 0u && (tx_aborting & bit) == 0u) {
            bool expired = (uint32_t)(now - tx_started_ms[mailbox]) >= tx_lifetime_ms[mailbox];
            if (bus_off || expired) {
                if (expired) ++status.tx_timeout_count;
                tx_aborting = (uint8_t)(tx_aborting | bit);
                /* Never let an old ACK/telemetry frame survive bus recovery.
                 * Completion/abort is accounted once through RQCP above. */
                CAN1->TSR = CAN_TSR_ABRQ0 << shift;
            }
        }
    }
}

static void record_fifo_flags(void)
{
    uint32_t flags = CAN1->RF0R;
    if ((flags & CAN_RF0R_FULL0) != 0u) {
        ++status.rx_fifo_full_count;
        CAN1->RF0R = CAN_RF0R_FULL0;
    }
    if ((flags & CAN_RF0R_FOVR0) != 0u) {
        ++status.rx_fifo_overrun_count;
        ++status.rx_dropped; /* At least one frame was lost per overrun event. */
        CAN1->RF0R = CAN_RF0R_FOVR0;
    }
}

bool BoardCan_Init(void)
{
    if (status.initialized) return true;
    BoardCan_LoadNodeId();
    /* Board_Init owns clocking and AFIO remapping. Refuse an unexpected clock
     * or remap rather than silently using the wrong bitrate/pins or touching
     * AFIO SWJ_CFG (whose readback is not reliable on this STM32 family). */
    if (SystemCoreClock != BOARD_CORE_HZ ||
        (RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV2) return false;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
    if ((AFIO->MAPR & AFIO_MAPR_CAN_REMAP) != 0u) return false;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;
    /* TJA1050 RXD -> PA11 input floating; PA12 -> TXD AF push-pull 50 MHz.
     * All other GPIOA modes, including TIM2, EN, USART1 and SWD, are preserved.
     */
    GPIOA->BSRR = CAN_TX_PIN;
    GPIOA->CRH = (GPIOA->CRH & ~((0xfu << 12) | (0xfu << 16))) |
                  (0x4u << 12) | (0xbu << 16);
    status = (BoardCanStatus){0};
    rx_head = rx_tail = tx_pending = tx_aborting = 0u;

    /* Locked RX FIFO drops the newest frame instead of overwriting old frames.
     * NART=0 permits arbitration retry: several nodes publish at the same
     * instant, so losing arbitration is normal, not a reason to discard data.
     * Retries remain bounded by the per-mailbox software expiry and bus-off
     * abort above (FAST 5 ms; all other frames at most 20 ms). */
    CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM |
                 CAN_MCR_RFLM | CAN_MCR_TXFP;
    if (!wait_inak(true)) return init_failed();
    CAN1->BTR = (CAN_PRESCALER - 1u) |
                  (12u << CAN_BTR_TS1_Pos) | (3u << CAN_BTR_TS2_Pos);

    /* One 32-bit LIST bank has TWO complete identifiers. Standard IDs occupy
     * bits 31:21, with IDE=0 and RTR=0 included in each exact comparison.
     * This rejects extended frames sharing the standard prefix and all RTRs.
     */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R = 0u;
    CAN1->FM1R = 1u;
    CAN1->FS1R = 1u;
    CAN1->FFA1R = 0u; /* Both matching identifiers go to FIFO0. */
    CAN1->sFilterRegister[0].FR1 = (0x200u + BoardCan_GetNodeId()) << 21;
    CAN1->sFilterRegister[0].FR2 = CAN_ESTOP_ID << 21;
    CAN1->FA1R = 1u;
    CAN1->FMR &= ~CAN_FMR_FINIT;
    CAN1->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;
    CAN1->RF0R = CAN_RF0R_FULL0 | CAN_RF0R_FOVR0;
    CAN1->MSR = CAN_MSR_ERRI;
    CAN1->MCR &= ~CAN_MCR_INRQ;
    if (!wait_inak(false)) return init_failed();

    status.initialized = true;
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 3u);
    NVIC_SetPriority(CAN1_SCE_IRQn, 3u);
    NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_ClearPendingIRQ(CAN1_SCE_IRQn);
    CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_FFIE0 | CAN_IER_FOVIE0 |
                  CAN_IER_EWGIE | CAN_IER_EPVIE | CAN_IER_BOFIE | CAN_IER_ERRIE;
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN1_SCE_IRQn);
    return true;
}

bool BoardCan_Read(BoardCanFrame *frame)
{
    if (frame == NULL) return false;
    uint32_t key = Board_EnterCritical();
    if (rx_tail == rx_head) {
        Board_ExitCritical(key);
        return false;
    }
    *frame = rx_queue[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1u) & (CAN_RX_SIZE - 1u));
    Board_ExitCritical(key);
    return true;
}

bool BoardCan_Send(const BoardCanFrame *frame)
{
    uint32_t key = Board_EnterCritical();
    if (!status.initialized || frame == NULL || frame->dlc > 8u ||
        frame->id > (frame->extended ? 0x1fffffffu : 0x7ffu)) {
        ++status.tx_dropped;
        Board_ExitCritical(key);
        return false;
    }
    refresh_status();
    uint32_t free_mailboxes = CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2);
    unsigned mailbox = 3u;
    for (unsigned candidate = 0u; candidate < 3u; ++candidate) {
        uint8_t bit = (uint8_t)(1u << candidate);
        /* CAN hardware keeps running inside the CPU critical section. A TX
         * may complete AFTER refresh_status took its TSR snapshot: do not
         * reuse that mailbox until the old completion has been accounted. */
        if ((free_mailboxes & (CAN_TSR_TME0 << candidate)) != 0u &&
            ((tx_pending | tx_aborting) & bit) == 0u) {
            mailbox = candidate;
            break;
        }
    }
    if (status.bus_off || mailbox == 3u) {
        ++status.tx_dropped;
        Board_ExitCritical(key);
        return false;
    }
    CAN_TxMailBox_TypeDef *target = &CAN1->sTxMailBox[mailbox];
    uint32_t identifier = frame->extended ?
        ((frame->id << 3) | CAN_TI0R_IDE) : (frame->id << 21);
    if (frame->remote) identifier |= CAN_TI0R_RTR;
    target->TIR = identifier;
    target->TDTR = frame->dlc;
    uint32_t low = 0u, high = 0u;
    for (unsigned i = 0u; i < 4u; ++i) {
        if (i < frame->dlc) low |= (uint32_t)frame->data[i] << (i * 8u);
        if (i + 4u < frame->dlc) high |= (uint32_t)frame->data[i + 4u] << (i * 8u);
    }
    target->TDLR = low;
    target->TDHR = high;
    tx_started_ms[mailbox] = Board_Millis();
    tx_lifetime_ms[mailbox] = frame->id == 0x700u + BoardCan_GetNodeId() ?
        (uint8_t)(CAN_FAST_FEEDBACK_PERIOD_US / 1000u) : CAN_TX_TIMEOUT_MS;
    tx_pending = (uint8_t)(tx_pending | (uint8_t)(1u << mailbox));
    tx_aborting = (uint8_t)(tx_aborting & (uint8_t)~(1u << mailbox));
    __DMB();
    target->TIR = identifier | CAN_TI0R_TXRQ;
    Board_ExitCritical(key);
    return true;
}

bool BoardCan_SendConfigurationReply(const BoardCanFrame *frame, uint32_t timeout_cycles)
{
    uint8_t previous = tx_pending;
    uint8_t added;
    unsigned mailbox;
    uint32_t start = Board_CycleCount();
    /* No interrupts or producers run in this terminal maintenance path. */
    refresh_status();
    previous = tx_pending;
    if (!BoardCan_Send(frame)) return false;
    added = (uint8_t)(tx_pending & (uint8_t)~previous);
    for (mailbox = 0u; mailbox < 3u; ++mailbox)
        if ((added & (1u << mailbox)) != 0u) break;
    if (mailbox == 3u) return false;
    do {
        uint32_t tsr = CAN1->TSR;
        if ((tsr & (CAN_TSR_RQCP0 << (mailbox * 8u))) != 0u) {
            bool ok = (tsr & (CAN_TSR_TXOK0 << (mailbox * 8u))) != 0u;
            refresh_status();
            return ok;
        }
    } while ((uint32_t)(Board_CycleCount() - start) < timeout_cycles);
    CAN1->TSR = CAN_TSR_ABRQ0 << (mailbox * 8u);
    return false;
}

void BoardCan_DiscardRx(void)
{
    uint32_t key = Board_EnterCritical();
    rx_tail = rx_head;
    if (status.initialized) {
        record_fifo_flags();
        /* Bounded even under continuous traffic. Newly arriving frames after
         * the flush carry their own receive time and are new protocol input. */
        for (unsigned i = 0u; i < 3u && (CAN1->RF0R & CAN_RF0R_FMP0) != 0u; ++i) {
            CAN1->RF0R = CAN_RF0R_RFOM0;
            ++status.rx_count;
        }
    }
    Board_ExitCritical(key);
}

BoardCanStatus BoardCan_GetStatus(void)
{
    uint32_t key = Board_EnterCritical();
    if (status.initialized) refresh_status();
    BoardCanStatus result = status;
    Board_ExitCritical(key);
    return result;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if (!status.initialized) return;
    record_fifo_flags();
    /* FIFO depth is three. Never let a busy bus create an unbounded ISR. */
    for (unsigned n = 0u; n < 3u && (CAN1->RF0R & CAN_RF0R_FMP0) != 0u; ++n) {
        BoardCanFrame frame;
        uint32_t identifier = CAN1->sFIFOMailBox[0].RIR;
        uint32_t length = CAN1->sFIFOMailBox[0].RDTR & 0xfu;
        uint32_t low = CAN1->sFIFOMailBox[0].RDLR;
        uint32_t high = CAN1->sFIFOMailBox[0].RDHR;
        frame.received_ms = Board_Millis();
        frame.extended = (identifier & CAN_RI0R_IDE) != 0u;
        frame.remote = (identifier & CAN_RI0R_RTR) != 0u;
        frame.id = frame.extended ? (identifier >> 3) & 0x1fffffffu : identifier >> 21;
        frame.dlc = (uint8_t)length;
        for (unsigned i = 0u; i < 4u; ++i) {
            frame.data[i] = (uint8_t)(low >> (i * 8u));
            frame.data[i + 4u] = (uint8_t)(high >> (i * 8u));
        }
        CAN1->RF0R = CAN_RF0R_RFOM0;
        ++status.rx_count;
        uint8_t next = (uint8_t)((rx_head + 1u) & (CAN_RX_SIZE - 1u));
        if (frame.extended || frame.remote || length > 8u ||
            (frame.id != 0x200u + BoardCan_GetNodeId() && frame.id != CAN_ESTOP_ID) || next == rx_tail) {
            ++status.rx_dropped;
        } else {
            rx_queue[rx_head] = frame;
            __DMB();
            rx_head = next;
        }
    }
    record_fifo_flags();
}

void CAN1_SCE_IRQHandler(void)
{
    if (!status.initialized) return;
    refresh_status(); /* Bus-off immediately requests abort of ALL pending TX. */
    CAN1->MSR = CAN_MSR_ERRI;
}
