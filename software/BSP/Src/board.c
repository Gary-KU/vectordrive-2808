#include "board.h"
#include "stm32f1xx.h"
#include <stddef.h>

/* SCH_SimpleFOC103_2026-10-24: STM32F103C8T6 + DRV8313 + AS5600.
 * Deliberately uses no HAL, heap, printf, DMA or I2C work in the current ISR.
 */
#define EN_PIN              (1u << 4)
#define LED_PIN             (1u << 13)
#define BUTTON_PIN          (1u << 12)
#define I2C_SCL             (1u << 6)
#define I2C_SDA             (1u << 7)
#define I2C_HALF_CYCLES     ((BOARD_CORE_HZ / 1000000u) * BOARD_I2C_HALF_PERIOD_US)
#define I2C_TIMEOUT_CYCLES  (BOARD_CORE_HZ / 500u)
#define AS5600_ADDRESS      0x36u
#define TX_SIZE             1024u
#define RX_SIZE             128u

static volatile uint32_t faults;
static uint32_t reset_cause;
static volatile bool initialized;
static volatile bool sampling;
static volatile bool driver_enabled;
static volatile uint32_t millis;
static volatile uint64_t elapsed_cycles;
static volatile uint32_t epoch_cycle;
static volatile uint32_t adc_sequence;
static volatile uint32_t last_adc_cycle;
/* Debugger-readable timing diagnostics. These count software IRQ entries and
 * observed pending flags, not every physical ADC trigger (JEOC is only one bit).
 * Durations cover the C ISR body including the application callback; exception
 * entry/exit, compiler prologue/epilogue and final diagnostic stores are extra.
 * Read while halted for a coherent snapshot; no diagnostic changes protection.
 */
volatile uint32_t board_adc_irq_count;
volatile uint32_t board_adc_spurious_count;
volatile uint32_t board_adc_isr_last_cycles;
volatile uint32_t board_adc_isr_max_cycles;
volatile uint32_t board_adc_entry_last_cycles;
volatile uint32_t board_adc_entry_min_cycles;
volatile uint32_t board_adc_entry_max_cycles;
volatile uint32_t board_adc_pending_count;
volatile uint32_t board_adc_elapsed_overrun_count;
volatile uint32_t board_adc_sync_error_count;
static uint32_t adc_previous_entry_cycle;
static uint32_t last_fed_sequence;
static bool watchdog_started;
static volatile uint16_t tx_head, tx_tail, rx_head, rx_tail;
static char tx_buffer[TX_SIZE], rx_buffer[RX_SIZE];
static volatile uint32_t tx_dropped, rx_dropped;
static BoardEncoderStatus encoder_status = BOARD_ENCODER_NOT_READ;
static bool as5600_filter_configured;

uint32_t Board_EnterCritical(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return saved;
}

void Board_PrepareConfigurationReset(void)
{
    __disable_irq();
    GPIOA->BRR = EN_PIN;
    driver_enabled = false;
    sampling = false;
    TIM2->CCER = 0u;
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    ADC1->CR1 &= ~ADC_CR1_JEOCIE;
    ADC1->CR2 &= ~(ADC_CR2_JEXTTRIG | ADC_CR2_ADON);
    ADC2->CR2 &= ~(ADC_CR2_JEXTTRIG | ADC_CR2_ADON);
    NVIC_DisableIRQ(ADC1_2_IRQn);
    /* This feed is legal only in this terminal path: physical output is off,
     * ADC is intentionally stopped, and reset follows within the flash bound. */
    if (watchdog_started) IWDG->KR = 0xaaaau;
    __DSB();
}

void Board_ExitCritical(uint32_t previous_primask)
{
    __DMB();
    __set_PRIMASK(previous_primask);
}

static void shutdown(uint32_t reason)
{
    /* The GPIO clock is enabled before anything that may fail at boot. */
    GPIOA->BRR = EN_PIN;
    driver_enabled = false;
    TIM2->CCER = 0u;
    faults |= reason;
}

static bool wait_bit(volatile uint32_t *reg, uint32_t bit, bool set)
{
    /* Also works before DWT and the system clock are configured. Bounded even
     * with a failed crystal; the exact startup timeout is not a timebase. */
    uint32_t attempts = 2000000u;
    while (((*reg & bit) != 0u) != set) {
        if (--attempts == 0u) { return false; }
    }
    return true;
}

static bool clock_init(void)
{
    RCC->CR |= RCC_CR_HSION;
    if (!wait_bit(&RCC->CR, RCC_CR_HSIRDY, true)) { return false; }
    RCC->CFGR &= ~RCC_CFGR_SW;
    uint32_t attempts = 2000000u;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {
        if (--attempts == 0u) { return false; }
    }
    RCC->CR &= ~RCC_CR_PLLON;
    if (!wait_bit(&RCC->CR, RCC_CR_PLLRDY, false)) { return false; }
    RCC->CR |= RCC_CR_HSEON;
    if (!wait_bit(&RCC->CR, RCC_CR_HSERDY, true)) { return false; }
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;
    /* HSE 8 MHz * 9 = 72 MHz; APB1 36 MHz; APB2 72 MHz; ADC 12 MHz. */
    RCC->CFGR = RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9 |
                RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6;
    RCC->CR |= RCC_CR_PLLON;
    if (!wait_bit(&RCC->CR, RCC_CR_PLLRDY, true)) { return false; }
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    attempts = 2000000u;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        if (--attempts == 0u) { return false; }
    }
    RCC->CR |= RCC_CR_CSSON; /* Loss of HSE causes NMI and an immediate EN low. */
    SystemCoreClock = BOARD_CORE_HZ;
    return true;
}

static void delay_cycles(uint32_t clocks)
{
    uint32_t started = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - started) < clocks) { __NOP(); }
}

static bool adc_wait(ADC_TypeDef *adc, uint32_t mask)
{
    uint32_t started = DWT->CYCCNT;
    while ((adc->CR2 & mask) != 0u) {
        if ((uint32_t)(DWT->CYCCNT - started) > BOARD_CORE_HZ / 100u) {
            return false;
        }
    }
    return true;
}

static bool adc_calibrate(ADC_TypeDef *adc)
{
    adc->CR2 = ADC_CR2_ADON;
    delay_cycles(BOARD_CORE_HZ / 100000u); /* 10 us, > two ADC clocks. */
    adc->CR2 |= ADC_CR2_RSTCAL;
    if (!adc_wait(adc, ADC_CR2_RSTCAL)) { return false; }
    adc->CR2 |= ADC_CR2_CAL;
    return adc_wait(adc, ADC_CR2_CAL);
}

static void gpio_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN |
                     RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    GPIOA->BRR = EN_PIN;
    /* PA0/1/2: TIM2 CH1/2/3, AF push-pull 2 MHz. PA4: common EN. */
    GPIOA->CRL = (GPIOA->CRL & ~0x000f0fffu) | 0x00020aaau;
    /* PA9 TX AF push-pull, PA10 RX floating input. */
    GPIOA->CRH = (GPIOA->CRH & ~0x00000ff0u) | 0x000004a0u;
    GPIOB->BSRR = I2C_SCL | I2C_SDA | BUTTON_PIN;
    /* PB0/PB1 remain analog. Released PB6/PB7 select weak pull-up inputs on
     * STM32F1; the software I2C helpers switch to open-drain only to drive low.
     */
#if BOARD_I2C_INTERNAL_PULLUPS
    GPIOB->CRL = (GPIOB->CRL & ~0xff0000ffu) | 0x88000000u;
#else
    GPIOB->CRL = (GPIOB->CRL & ~0xff0000ffu) | 0x66000000u;
#endif
    GPIOB->CRH = (GPIOB->CRH & ~(0xfu << 16)) | (0x8u << 16);
    GPIOC->BRR = LED_PIN;
    GPIOC->CRH = (GPIOC->CRH & ~(0xfu << 20)) | (0x2u << 20);
}

static void pwm_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM3RST;
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST | RCC_APB1RSTR_TIM3RST);
    TIM2->PSC = 0u;
    TIM2->ARR = BOARD_PWM_PERIOD;
    TIM2->CCR1 = BOARD_PWM_PERIOD / 2u;
    TIM2->CCR2 = BOARD_PWM_PERIOD / 2u;
    TIM2->CCR3 = BOARD_PWM_PERIOD / 2u;
    /* OC4REF rises on the up-count at 95%. At 82% maximum phase duty,
     * both sampled low sides have settled for 2.708 us.
     * The ADC uses 13.5+12.5 clocks at 12 MHz = 2.167 us, plus the internal
     * TIM3/ADC trigger synchronization delay. Keep this low-side window.
     * PWM2's down-count transition is falling: ONE rising TRGO/carrier cycle.
     */
    TIM2->CCR4 = (BOARD_PWM_PERIOD * BOARD_ADC_SAMPLE_PERCENT) / 100u;
    TIM2->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCMR2 = (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE |
                  (7u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM2->CR2 = (7u << TIM_CR2_MMS_Pos); /* TRGO = OC4REF, not update event. */
    TIM2->CR1 = TIM_CR1_CMS_0 | TIM_CR1_CMS_1 | TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0u;

    /* RM0008 TIM3 ITR1 = TIM2_TRGO. External clock mode 1 (SMS=111) counts
     * only TRGI rising edges. With ARR=5/CCR4=5, CC4 occurs every six PWM
     * pulses; CNT=0 makes the first sample occur on pulse five. On this board
     * the ADC's TIM3_CC4 trigger needs a rising CH4 waveform with CC4E enabled:
     * bench isolation showed CC4IF in frozen mode does NOT trigger the ADC,
     * including with CC4E set. PWM2 rises at CNT=5 and falls on overflow.
     * PB1 remains analog, so this internal channel is not driven onto its pin.
     * No TIM3 interrupt is enabled.
     */
    TIM3->PSC = 0u;
    TIM3->ARR = (BOARD_PWM_HZ / BOARD_ADC_HZ) - 1u;
    TIM3->CCR4 = TIM3->ARR;
    TIM3->CCMR2 = (7u << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    TIM3->CCER = TIM_CCER_CC4E;
    TIM3->DIER = 0u;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->SMCR = (1u << TIM_SMCR_TS_Pos) | (7u << TIM_SMCR_SMS_Pos);
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CNT = 0u;
    TIM3->SR = 0u;
}

static bool adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST | RCC_APB2RSTR_ADC2RST;
    RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADC1RST | RCC_APB2RSTR_ADC2RST);
    /* Calibrate independently before selecting dual mode. */
    if (!adc_calibrate(ADC1) || !adc_calibrate(ADC2)) { return false; }
    ADC1->CR2 &= ~ADC_CR2_ADON;
    ADC2->CR2 &= ~ADC_CR2_ADON;
#if defined(MOTOR_ADC_CH_SWAP)
    /* Diagnostic: swap U/W ADC channels to localize a dead input.
     * raw_u then carries PB1 (W) and raw_w carries PB0 (U). */
    ADC1->SMPR2 = 2u << (9u * 3u); /* ADC1 IN9 (PB1) */
    ADC2->SMPR2 = 2u << (8u * 3u); /* ADC2 IN8 (PB0) */
    ADC1->JSQR = 9u << ADC_JSQR_JSQ4_Pos;
    ADC2->JSQR = 8u << ADC_JSQR_JSQ4_Pos;
#else
    ADC1->SMPR2 = 2u << (8u * 3u); /* ADC1 IN8, 13.5 sample clocks. */
    ADC2->SMPR2 = 2u << (9u * 3u); /* ADC2 IN9, identical sample time. */
    /* With JL=0, a single injected conversion uses JSQ4, NOT JSQ1. */
    ADC1->JSQR = 8u << ADC_JSQR_JSQ4_Pos;
    ADC2->JSQR = 9u << ADC_JSQR_JSQ4_Pos;
#endif
    ADC1->CR1 = (5u << ADC_CR1_DUALMOD_Pos) | ADC_CR1_JEOCIE;
    ADC2->CR1 = 0u;
    /* RM0008 dual-mode rule: master selects hardware trigger, slave selects
     * software trigger, and BOTH JEXTTRIG bits must be enabled when started.
     * ADC1 injected JEXTSEL=100 is TIM3_CC4; slave 111 is JSWSTART. */
    ADC1->CR2 = ADC_CR2_ADON | (4u << ADC_CR2_JEXTSEL_Pos);
    ADC2->CR2 = ADC_CR2_ADON | (7u << ADC_CR2_JEXTSEL_Pos);
    delay_cycles(BOARD_CORE_HZ / 100000u);
    ADC1->SR = 0u;
    ADC2->SR = 0u;
    NVIC_SetPriority(ADC1_2_IRQn, 0u);
    NVIC_ClearPendingIRQ(ADC1_2_IRQn);
    NVIC_EnableIRQ(ADC1_2_IRQn);
    return true;
}

static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
    USART1->BRR = 625u; /* 72 MHz / 115200 exactly. */
    USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 2u);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
}

bool Board_Init(void)
{
    /* First software operation affecting the motor is force-low common EN. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->BRR = EN_PIN;
    GPIOA->CRL = (GPIOA->CRL & ~(0xfu << 16)) | (0x2u << 16);
    reset_cause = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    if (!clock_init()) { shutdown(BOARD_FAULT_CLOCK); return false; }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    epoch_cycle = DWT->CYCCNT;
    if (SysTick_Config(BOARD_CORE_HZ / 1000u) != 0u) {
        shutdown(BOARD_FAULT_CLOCK); return false;
    }
    NVIC_SetPriority(SysTick_IRQn, 15u);
    gpio_init();
    pwm_init();
    uart_init();
    if (!adc_init()) { shutdown(BOARD_FAULT_ADC_INIT); return false; }
    initialized = true;
    return true;
}

bool Board_StartSampling(void)
{
    if (!initialized || faults != 0u) { return false; }
    if (sampling) { return true; }
#if BOARD_WATCHDOG_ENABLE
    RCC->CSR |= RCC_CSR_LSION;
    if (!wait_bit(&RCC->CSR, RCC_CSR_LSIRDY, true)) {
        shutdown(BOARD_FAULT_WATCHDOG); return false;
    }
    /* Prescaler /64, 63 counts: 100.8 ms nominal at 40 kHz LSI. */
    /* Start first, then configure and wait for the kernel register updates,
     * matching ST's STM32F1 IWDG initialization sequence. */
    IWDG->KR = 0xccccu;
    IWDG->KR = 0x5555u;
    IWDG->PR = 4u;
    IWDG->RLR = 62u;
    uint32_t started = DWT->CYCCNT;
    while (IWDG->SR != 0u) {
        if ((uint32_t)(DWT->CYCCNT - started) > BOARD_CORE_HZ / 100u) {
            shutdown(BOARD_FAULT_WATCHDOG); return false;
        }
    }
    IWDG->KR = 0xaaaau;
    watchdog_started = true;
#endif
    last_fed_sequence = adc_sequence;
    ADC1->SR = 0u;
    ADC2->SR = 0u;
    sampling = true;
    ADC2->CR2 |= ADC_CR2_JEXTTRIG;
    ADC1->CR2 |= ADC_CR2_JEXTTRIG;
    TIM3->CNT = 0u;
    TIM3->SR = 0u;
    TIM3->CR1 |= TIM_CR1_CEN; /* Arm the divider before the first PWM edge. */
    TIM2->CNT = 0u;
    TIM2->CR1 |= TIM_CR1_CEN;
    return true;
}

void Board_SetPwm(float u, float v, float w)
{
    /* Ordered comparisons also reject NaN and +/-infinity without libm. */
    if (!(u >= BOARD_PWM_MIN_DUTY && u <= BOARD_PWM_MAX_DUTY &&
          v >= BOARD_PWM_MIN_DUTY && v <= BOARD_PWM_MAX_DUTY &&
          w >= BOARD_PWM_MIN_DUTY && w <= BOARD_PWM_MAX_DUTY)) {
        shutdown(BOARD_FAULT_PWM);
        return;
    }
    /* Volatile integer temporaries require all three software-floating-point
     * conversions to finish before UDIS is asserted, with either toolchain. */
    volatile const uint32_t compare_u = (uint32_t)(u * (float)BOARD_PWM_PERIOD + 0.5f);
    volatile const uint32_t compare_v = (uint32_t)(v * (float)BOARD_PWM_PERIOD + 0.5f);
    volatile const uint32_t compare_w = (uint32_t)(w * (float)BOARD_PWM_PERIOD + 0.5f);
    /* Atomic three-phase preload transfer: suppress a possible update event
     * during these writes, rather than exposing a partially updated vector. */
    TIM2->CR1 |= TIM_CR1_UDIS;
    TIM2->CCR1 = compare_u;
    TIM2->CCR2 = compare_v;
    TIM2->CCR3 = compare_w;
    __DSB();
    TIM2->CR1 &= ~TIM_CR1_UDIS;
}

void Board_SetDriverEnabled(bool enabled)
{
    if (!enabled) {
        GPIOA->BRR = EN_PIN;
        driver_enabled = false;
        return;
    }
    uint32_t key = Board_EnterCritical();
    if (initialized && sampling && faults == 0u && adc_sequence != 0u &&
        (uint32_t)(DWT->CYCCNT - last_adc_cycle) < BOARD_DRIVER_FRESH_CYCLES) {
        GPIOA->BSRR = EN_PIN;
        driver_enabled = true;
    }
    Board_ExitCritical(key);
}

bool Board_DriverEnabled(void) { return driver_enabled; }
uint32_t Board_FaultFlags(void) { return faults; }
uint32_t Board_ResetCause(void) { return reset_cause; }
uint32_t Board_CycleCount(void) { return DWT->CYCCNT; }
uint32_t Board_Millis(void) { return millis; }

uint32_t Board_Micros32(void)
{
    uint32_t key = Board_EnterCritical();
    uint64_t cycles = elapsed_cycles + (uint32_t)(DWT->CYCCNT - epoch_cycle);
    Board_ExitCritical(key);
    return (uint32_t)(cycles / (BOARD_CORE_HZ / 1000000u));
}

void Board_FeedWatchdog(void)
{
    if (watchdog_started && faults == 0u && adc_sequence != last_fed_sequence &&
        (uint32_t)(DWT->CYCCNT - last_adc_cycle) < 4u * (BOARD_CORE_HZ / BOARD_ADC_HZ)) {
        last_fed_sequence = adc_sequence;
        IWDG->KR = 0xaaaau;
    }
}

bool Board_ButtonPressed(void) { return (GPIOB->IDR & BUTTON_PIN) == 0u; }
void Board_SetLed(bool on) { GPIOC->BSRR = on ? LED_PIN : LED_PIN << 16; }

bool Board_UartRead(char *character)
{
    if (character == NULL || rx_head == rx_tail) { return false; }
    *character = rx_buffer[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) & (RX_SIZE - 1u));
    return true;
}

void Board_UartDiscardInput(void)
{
    uint32_t key = Board_EnterCritical();
    rx_tail = rx_head;
    Board_ExitCritical(key);
}

void Board_UartWrite(const char *text)
{
    if (text == NULL) { return; }
    while (*text != '\0') {
        uint16_t next = (uint16_t)((tx_head + 1u) & (TX_SIZE - 1u));
        if (next == tx_tail) {
            while (*text++ != '\0') { ++tx_dropped; }
            return;
        }
        tx_buffer[tx_head] = *text++;
        __DMB();
        tx_head = next;
        USART1->CR1 |= USART_CR1_TXEIE;
    }
}

uint32_t Board_UartTxDropped(void) { return tx_dropped; }
uint32_t Board_UartRxDropped(void) { return rx_dropped; }

void SysTick_Handler(void)
{
    /* ADC can preempt SysTick and itself call Micros32. Keep the 64-bit epoch
     * and 32-bit reference coherent with one very short critical section. */
    uint32_t key = Board_EnterCritical();
    uint32_t now = DWT->CYCCNT;
    elapsed_cycles += (uint32_t)(now - epoch_cycle);
    epoch_cycle = now;
    ++millis;
    Board_ExitCritical(key);
}

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0u) {
        char received = (char)USART1->DR; /* SR then DR clears receive errors. */
        if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0u) {
            ++rx_dropped;
        } else if ((status & USART_SR_RXNE) != 0u) {
            uint16_t next = (uint16_t)((rx_head + 1u) & (RX_SIZE - 1u));
            if (next != rx_tail) {
                rx_buffer[rx_head] = received;
                __DMB();
                rx_head = next;
            } else { ++rx_dropped; }
        }
    }
    if ((status & USART_SR_TXE) != 0u && (USART1->CR1 & USART_CR1_TXEIE) != 0u) {
        if (tx_tail != tx_head) {
            USART1->DR = (uint8_t)tx_buffer[tx_tail];
            tx_tail = (uint16_t)((tx_tail + 1u) & (TX_SIZE - 1u));
        } else { USART1->CR1 &= ~USART_CR1_TXEIE; }
    }
}

static void adc_record_duration(uint32_t entry_cycle)
{
    uint32_t elapsed = (uint32_t)(DWT->CYCCNT - entry_cycle);
    board_adc_isr_last_cycles = elapsed;
    if (elapsed > board_adc_isr_max_cycles) board_adc_isr_max_cycles = elapsed;
}

void ADC1_2_IRQHandler(void)
{
    uint32_t entry_cycle = DWT->CYCCNT;
    if (++board_adc_irq_count > 1u) {
        uint32_t interval = (uint32_t)(entry_cycle - adc_previous_entry_cycle);
        board_adc_entry_last_cycles = interval;
        if (board_adc_entry_min_cycles == 0u || interval < board_adc_entry_min_cycles)
            board_adc_entry_min_cycles = interval;
        if (interval > board_adc_entry_max_cycles) board_adc_entry_max_cycles = interval;
    }
    adc_previous_entry_cycle = entry_cycle;
    if ((ADC1->SR & ADC_SR_JEOC) == 0u) {
        ++board_adc_spurious_count;
        adc_record_duration(entry_cycle);
        return;
    }
    /* Both sequences are one channel at identical sample time. Allow at most
     * 1 us for a slave flag visibility skew; never use stale ADC2 data. */
    while ((ADC2->SR & ADC_SR_JEOC) == 0u) {
        if ((uint32_t)(DWT->CYCCNT - entry_cycle) >= BOARD_CORE_HZ / 1000000u) {
            ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
            ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
            ++board_adc_sync_error_count;
            shutdown(BOARD_FAULT_ADC_SYNC);
            adc_record_duration(entry_cycle);
            return;
        }
    }
    uint16_t raw_u = (uint16_t)ADC1->JDR1;
    uint16_t raw_w = (uint16_t)ADC2->JDR1;
    ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT);
    last_adc_cycle = entry_cycle;
    ++adc_sequence;
    App_OnAdcSample(raw_u, raw_w);
    bool pending = (ADC1->SR & ADC_SR_JEOC) != 0u;
    bool elapsed_overrun = (uint32_t)(DWT->CYCCNT - entry_cycle) >= BOARD_CORE_HZ / BOARD_ADC_HZ;
    if (pending) ++board_adc_pending_count;
    if (elapsed_overrun) ++board_adc_elapsed_overrun_count;
#if BOARD_ADC_OVERRUN_TOLERANT
    /* Bring-up: a pending JEOC is self-correcting (the NVIC re-enters this
     * handler). Do not latch an overrun fault while validating spin. */
#else
    if (pending || elapsed_overrun) {
        shutdown(BOARD_FAULT_ADC_OVERRUN);
    }
#endif
    adc_record_duration(entry_cycle);
}

static void exception_stop(void)
{
    __disable_irq();
    shutdown(BOARD_FAULT_EXCEPTION);
    /* IWDG resets a running system. Before sampling starts EN stays low. */
    for (;;) { __NOP(); }
}

void NMI_Handler(void) { exception_stop(); }
void HardFault_Handler(void) { exception_stop(); }
void MemManage_Handler(void) { exception_stop(); }
void BusFault_Handler(void) { exception_stop(); }
void UsageFault_Handler(void) { exception_stop(); }

/* AS5600 software I2C ------------------------------------------------------ */
static bool i2c_expired(uint32_t began)
{
    return (uint32_t)(DWT->CYCCNT - began) >= I2C_TIMEOUT_CYCLES;
}

static void i2c_release_line(uint32_t pin, unsigned shift)
{
    /* Set the latch first: input mode CNF=10/MODE=00 uses ODR=1 for pull-up.
     * Output open-drain on this MCU cannot retain its internal pull-up.
     */
    GPIOB->BSRR = pin;
#if BOARD_I2C_INTERNAL_PULLUPS
    GPIOB->CRL = (GPIOB->CRL & ~(0xfu << shift)) | (0x8u << shift);
#else
    (void)shift;
#endif
}

static void i2c_drive_low(uint32_t pin, unsigned shift)
{
    GPIOB->BRR = pin;
#if BOARD_I2C_INTERNAL_PULLUPS
    GPIOB->CRL = (GPIOB->CRL & ~(0xfu << shift)) | (0x6u << shift);
#else
    (void)shift;
#endif
}

static bool scl_high(uint32_t began)
{
    i2c_release_line(I2C_SCL, 24u);
    while ((GPIOB->IDR & I2C_SCL) == 0u) {
        if (i2c_expired(began)) { return false; }
    }
    return !i2c_expired(began);
}

static void scl_low(void) { i2c_drive_low(I2C_SCL, 24u); }
static void sda_release(void) { i2c_release_line(I2C_SDA, 28u); }
static void sda_low(void) { i2c_drive_low(I2C_SDA, 28u); }
static void i2c_delay(void) { delay_cycles(I2C_HALF_CYCLES); }

static bool i2c_stop(uint32_t began)
{
    scl_low();
    sda_low();
    i2c_delay();
    if (!scl_high(began)) { sda_release(); return false; }
    i2c_delay();
    sda_release();
    i2c_delay();
    return (GPIOB->IDR & (I2C_SDA | I2C_SCL)) == (I2C_SDA | I2C_SCL) &&
           !i2c_expired(began);
}

static bool i2c_recover(uint32_t began)
{
    sda_release();
    if (!scl_high(began)) { return false; }
    for (unsigned i = 0u; (GPIOB->IDR & I2C_SDA) == 0u && i < 9u; ++i) {
        scl_low();
        i2c_delay();
        if (!scl_high(began)) { return false; }
        i2c_delay();
    }
    return i2c_stop(began);
}

static bool i2c_start(uint32_t began)
{
    sda_release();
    i2c_delay();
    if (!scl_high(began)) { return false; }
    i2c_delay();
    if ((GPIOB->IDR & I2C_SDA) == 0u) { return false; }
    sda_low();
    i2c_delay();
    scl_low();
    return !i2c_expired(began);
}

static bool i2c_write(uint8_t data, uint32_t began)
{
    for (unsigned bit = 0u; bit < 8u; ++bit) {
        if ((data & 0x80u) != 0u) { sda_release(); } else { sda_low(); }
        data = (uint8_t)(data << 1);
        i2c_delay();
        if (!scl_high(began)) { return false; }
        i2c_delay();
        scl_low();
    }
    sda_release();
    i2c_delay();
    if (!scl_high(began)) { return false; }
    i2c_delay();
    bool ack = (GPIOB->IDR & I2C_SDA) == 0u;
    scl_low();
    return ack && !i2c_expired(began);
}

static bool i2c_read(uint8_t *data, bool ack, uint32_t began)
{
    uint8_t value = 0u;
    sda_release();
    for (unsigned bit = 0u; bit < 8u; ++bit) {
        i2c_delay();
        if (!scl_high(began)) { return false; }
        i2c_delay();
        value = (uint8_t)((value << 1) | ((GPIOB->IDR & I2C_SDA) != 0u ? 1u : 0u));
        scl_low();
    }
    if (ack) { sda_low(); } else { sda_release(); }
    i2c_delay();
    if (!scl_high(began)) { return false; }
    i2c_delay();
    scl_low();
    sda_release();
    *data = value;
    return !i2c_expired(began);
}

static bool as5600_read(uint8_t reg, uint8_t *data, unsigned size)
{
    const uint32_t began = DWT->CYCCNT;
    bool ok = i2c_start(began) && i2c_write((uint8_t)(AS5600_ADDRESS << 1), began) &&
              i2c_write(reg, began) && i2c_start(began) &&
              i2c_write((uint8_t)((AS5600_ADDRESS << 1) | 1u), began);
    for (unsigned i = 0u; ok && i < size; ++i) {
        ok = i2c_read(&data[i], i + 1u < size, began);
    }
    bool stopped = i2c_stop(began);
    return ok && stopped;
}

static bool as5600_write_conf_high(uint8_t value)
{
    const uint32_t began = DWT->CYCCNT;
    bool ok = i2c_start(began) && i2c_write((uint8_t)(AS5600_ADDRESS << 1), began) &&
              i2c_write(0x07u, began) && i2c_write(value, began);
    bool stopped = i2c_stop(began);
    return ok && stopped;
}

static bool as5600_configure_filter(void)
{
    uint8_t conf[2], verified[2], high;
    /* AS5600 DS000365, register map/CONF: SF is CONF[9:8], i.e. the low
     * two bits of address 0x07. SF=11 is 2x (fastest), NOT 4x (SF=10).
     * Preserve every other CONF/factory bit and leave 0x08 untouched.
     * Only volatile 0x07 is written; no BURN/OTP command is issued. */
    if (!as5600_read(0x07u, conf, 2u)) { return false; }
    high = (uint8_t)(conf[0] | 0x03u);
    if (high != conf[0] && !as5600_write_conf_high(high)) { return false; }
    if (!as5600_read(0x07u, verified, 2u) ||
        verified[0] != high || verified[1] != conf[1]) { return false; }
    as5600_filter_configured = true;
    return true;
}

bool Board_ReadEncoder(float *radians)
{
    uint32_t began = DWT->CYCCNT;
    uint8_t status, angle[2];
    if (!initialized || radians == NULL) { return false; }
    sda_release();
    i2c_release_line(I2C_SCL, 24u);
    i2c_delay(); /* Released SDA needs time to rise through the weak pull-up. */
    if ((GPIOB->IDR & (I2C_SDA | I2C_SCL)) != (I2C_SDA | I2C_SCL) &&
        !i2c_recover(began)) {
        as5600_filter_configured = false;
        encoder_status = BOARD_ENCODER_BUS_ERROR;
        return false;
    }
    if (!as5600_filter_configured && !as5600_configure_filter()) {
        encoder_status = BOARD_ENCODER_BUS_ERROR;
        return false;
    }
    if (!as5600_read(0x0bu, &status, 1u)) {
        as5600_filter_configured = false;
        encoder_status = BOARD_ENCODER_BUS_ERROR;
        return false;
    }
    if ((status & (1u << 5)) == 0u) {
        encoder_status = BOARD_ENCODER_NO_MAGNET;
        return false;
    }
    if ((status & (1u << 4)) != 0u) {
        encoder_status = BOARD_ENCODER_MAGNET_WEAK;
        return false;
    }
    if ((status & (1u << 3)) != 0u) {
        encoder_status = BOARD_ENCODER_MAGNET_STRONG;
        return false;
    }
    if (!as5600_read(0x0cu, angle, 2u)) {
        as5600_filter_configured = false;
        encoder_status = BOARD_ENCODER_BUS_ERROR;
        return false;
    }
    uint16_t raw = (uint16_t)((((uint16_t)angle[0] << 8) | angle[1]) & 0x0fffu);
    *radians = (float)raw * (6.2831853071795864769f / 4096.0f);
    encoder_status = BOARD_ENCODER_OK;
    return true;
}

BoardEncoderStatus Board_EncoderStatus(void) { return encoder_status; }
