#include "board_calibration.h"
#include "board.h"
#include "calibration_store.h"
#include "can_protocol.h"
#include "stm32f1xx.h"

#define STORE_ADDRESS 0x0800f000u
#define FLASH_TIMEOUT_CYCLES (BOARD_CORE_HZ / 20u)
#define REPLY_TIMEOUT_CYCLES (BOARD_CORE_HZ / 200u)

static CalibrationStoreState stored;
static bool loaded;

static uint32_t address(unsigned page, unsigned halfword)
{
    return STORE_ADDRESS + page * CALIBRATION_STORE_PAGE_BYTES + halfword * 2u;
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
    if (page >= 2u || halfword >= CALIBRATION_STORE_PAGE_BYTES / 2u || !flash_wait()) return false;
    if (storage_read(context, page, halfword) != 0xffffu) return false;
    FLASH->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PG;
    *(volatile uint16_t *)address(page, halfword) = value;
    ok = flash_wait();
    FLASH->CR = 0u;
    return ok && storage_read(context, page, halfword) == value;
}

static const CalibrationStoreIo io = {0, storage_read, storage_erase, storage_program};

void BoardCalibration_Load(void)
{
    if (loaded) return;
    CalibrationStore_Load(&io, &stored);
    loaded = true;
}

BoardCalibrationInfo BoardCalibration_GetInfo(void)
{
    BoardCalibrationInfo info;
    BoardCalibration_Load();
    info.valid = stored.valid;
    info.storage_error = stored.storage_error;
    info.motor_model = stored.motor_model;
    info.pole_pairs = stored.pole_pairs;
    info.sensor_direction = stored.sensor_direction;
    info.electrical_zero_rad = stored.electrical_zero_rad;
    return info;
}

static uint16_t zero_milliradians(float radians)
{
    uint32_t value = (uint32_t)(radians * 1000.0f + 0.5f);
    return (uint16_t)(value > 6283u ? 6283u : value);
}

void BoardCalibration_CommitAndReset(bool clear, uint16_t motor_model,
    uint8_t pole_pairs, int8_t sensor_direction, float electrical_zero_rad,
    BoardCanFrame ack, BoardCanFrame info)
{
    bool saved;
    uint32_t start;
    Board_PrepareConfigurationReset();
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
    saved = (FLASH->CR & FLASH_CR_LOCK) == 0u &&
        (clear ? CalibrationStore_Clear(&io, &stored) :
         CalibrationStore_Save(&io, motor_model, pole_pairs, sensor_direction,
                               electrical_zero_rad, &stored));
    FLASH->CR |= FLASH_CR_LOCK;
    if (!saved) stored.storage_error = true;
    ack.data[3] = saved ? 0u : CAN_RESULT_STORAGE;
    info.data[3] = ack.data[3];
    info.data[4] = (stored.valid ? BOARD_CALIBRATION_STORED : 0u) |
        (stored.storage_error ? BOARD_CALIBRATION_STORAGE_ERROR : 0u) |
        BOARD_CALIBRATION_RESET_PENDING;
    info.data[5] = stored.valid ? (uint8_t)stored.sensor_direction : 0u;
    {
        const uint16_t zero = stored.valid ? zero_milliradians(stored.electrical_zero_rad) : 0u;
        info.data[6] = (uint8_t)zero; info.data[7] = (uint8_t)(zero >> 8);
    }
    (void)BoardCan_SendConfigurationReply(&ack, REPLY_TIMEOUT_CYCLES);
    (void)BoardCan_SendConfigurationReply(&info, REPLY_TIMEOUT_CYCLES);
    NVIC_SystemReset();
    for (;;) { }
}
