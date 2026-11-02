#include "board.h"
#include "board_can.h"
#include "can_config.h"
#include "can_protocol.h"
#include "node_id_store.h"
#include "stm32f1xx.h"

#define STORE_ADDRESS 0x0800f800u
#define FLASH_TIMEOUT_CYCLES (BOARD_CORE_HZ / 20u)
#define REPLY_TIMEOUT_CYCLES (BOARD_CORE_HZ / 200u)

static NodeIdStoreState stored;
static uint8_t active_id = CAN_NODE_ID;
static bool loaded;

static uint32_t address(unsigned page, unsigned halfword)
{
    return STORE_ADDRESS + page * NODE_ID_STORE_PAGE_BYTES + halfword * 2u;
}

static uint16_t storage_read(void *context, unsigned page, unsigned halfword)
{
    (void)context;
    return *(const volatile uint16_t *)address(page, halfword);
}

static bool flash_wait(void)
{
    uint32_t start = Board_CycleCount();
    while ((FLASH->SR & FLASH_SR_BSY) != 0u)
        if ((uint32_t)(Board_CycleCount() - start) >= FLASH_TIMEOUT_CYCLES) return false;
    return (FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) == 0u;
}

static bool storage_erase(void *context, unsigned page)
{
    bool ok;
    (void)context;
    if (page >= 2u || !flash_wait()) return false;
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = address(page, 0u);
    FLASH->CR |= FLASH_CR_STRT;
    ok = flash_wait();
    FLASH->CR = 0u;
    return ok;
}

static bool storage_program(void *context, unsigned page, unsigned halfword, uint16_t value)
{
    bool ok;
    (void)context;
    if (page >= 2u || halfword >= NODE_ID_STORE_PAGE_BYTES / 2u || !flash_wait()) return false;
    if (storage_read(context, page, halfword) != 0xffffu) return false;
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PG;
    *(volatile uint16_t *)address(page, halfword) = value;
    ok = flash_wait();
    FLASH->CR = 0u;
    return ok && storage_read(context, page, halfword) == value;
}

static const NodeIdStoreIo io = {0, storage_read, storage_erase, storage_program};

void BoardCan_LoadNodeId(void)
{
    if (loaded) return;
    NodeIdStore_Load(&io, CAN_NODE_ID, &stored);
    active_id = stored.node_id;
    loaded = true;
}

uint8_t BoardCan_GetNodeId(void) { return active_id; }

BoardCanNodeInfo BoardCan_GetNodeInfo(void)
{
    BoardCanNodeInfo info = {active_id, stored.valid ? stored.node_id : active_id, 0u};
    if (stored.valid) info.flags |= BOARD_NODE_STORED;
    if (stored.storage_error) info.flags |= BOARD_NODE_STORAGE_ERROR;
    return info;
}

void BoardCan_CommitNodeIdAndReset(uint8_t new_id, BoardCanFrame ack, BoardCanFrame info)
{
    bool saved;
    BoardCanNodeInfo node;
    uint32_t start;
    Board_PrepareConfigurationReset();
    /* Abort unrelated telemetry before using the completion count to confirm
     * each configuration reply. No ISR/foreground producer remains active. */
    CAN1->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
    start = Board_CycleCount();
    while ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) !=
            (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) {
        (void)BoardCan_GetStatus();
        if ((uint32_t)(Board_CycleCount() - start) > BOARD_CORE_HZ / 500u) break;
    }
    (void)BoardCan_GetStatus();
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xcdef89abu;
    }
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
    saved = (FLASH->CR & FLASH_CR_LOCK) == 0u && NodeIdStore_Save(&io, new_id, &stored);
    FLASH->CR |= FLASH_CR_LOCK;
    if (!saved) stored.storage_error = true;
    node = BoardCan_GetNodeInfo();
    ack.data[3] = saved ? 0u : CAN_RESULT_STORAGE;
    info.data[3] = ack.data[3];
    info.data[4] = active_id;
    info.data[5] = node.stored_id;
    info.data[6] = 1u;
    info.data[7] = node.flags | BOARD_NODE_RESET_PENDING;
    (void)BoardCan_SendConfigurationReply(&ack, REPLY_TIMEOUT_CYCLES);
    (void)BoardCan_SendConfigurationReply(&info, REPLY_TIMEOUT_CYCLES);
    /* IRQs remain disabled and sampling is off. Neither a missing old-ID ACK
     * nor a failed write may strand the board in a half-maintenance state. */
    NVIC_SystemReset();
    for (;;) { }
}
