#ifndef GIMBAL_BOARD_H
#define GIMBAL_BOARD_H

#include <stdbool.h>
#include <stdint.h>
#include "board_config.h"

/* Set to 0 to skip IWDG startup on build-up boards where the LSI/IWDG update
 * never completes (PVU/RVU stuck). When 0, Board_FeedWatchdog() is a no-op.
 * Production images keep the watchdog enabled (default 1). */
#ifndef BOARD_WATCHDOG_ENABLE
#define BOARD_WATCHDOG_ENABLE 1
#endif

#ifndef BOARD_DRIVER_FRESH_CYCLES
#define BOARD_DRIVER_FRESH_CYCLES (2u * (BOARD_CORE_HZ / BOARD_ADC_HZ))
#endif

/* Bring-up: treat a pending JEOC at the end of the ADC ISR as self-correcting
 * instead of latching BOARD_FAULT_ADC_OVERRUN. Production keeps it strict. */
#ifndef BOARD_ADC_OVERRUN_TOLERANT
#define BOARD_ADC_OVERRUN_TOLERANT 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOARD_ENCODER_OK = 0,
    BOARD_ENCODER_NOT_READ,
    BOARD_ENCODER_BUS_ERROR,
    BOARD_ENCODER_NO_MAGNET,
    BOARD_ENCODER_MAGNET_WEAK,
    BOARD_ENCODER_MAGNET_STRONG
} BoardEncoderStatus;

enum {
    BOARD_FAULT_CLOCK       = (1u << 0),
    BOARD_FAULT_ADC_INIT    = (1u << 1),
    BOARD_FAULT_ADC_SYNC    = (1u << 2),
    BOARD_FAULT_PWM         = (1u << 3),
    BOARD_FAULT_EXCEPTION   = (1u << 4),
    BOARD_FAULT_WATCHDOG    = (1u << 5),
    BOARD_FAULT_ADC_OVERRUN = (1u << 6)
};

/* Init leaves DRV8313 EN low. StartSampling starts TIM2 PWM, TIM3's hardware
 * trigger divider, the PWM-synchronous ADC pair stream and IWDG.
 * Failed hardware checks latch a fault; resetting the MCU is required to clear it.
 * The application owns final motor-enable and current-offset calibration policy.
 */
bool Board_Init(void);
bool Board_StartSampling(void);
void Board_SetPwm(float u, float v, float w);
void Board_SetDriverEnabled(bool enabled);
bool Board_DriverEnabled(void);
uint32_t Board_FaultFlags(void);
/* Raw RCC_CSR reset flags captured before RMVF clears them. */
uint32_t Board_ResetCause(void);
/* RCC_CSR software/watchdog reset flags. A one-shot power-on motion must not
 * repeat after a firmware-triggered or watchdog recovery reset. */
#define BOARD_RESET_AUTOTEST_INHIBIT_MASK ((1u << 28) | (1u << 29) | (1u << 30))
#define BOARD_RESET_POWER_ON_MASK         (1u << 27)
uint32_t Board_EnterCritical(void);
void Board_ExitCritical(uint32_t previous_primask);

/* Unsigned differences are valid across each 32-bit wrap. Micros32 is extended
 * from DWT at every SysTick, so it does NOT jump every 59.65 seconds of DWT wrap.
 * CycleCount wraps at 2^32 core clocks; use unsigned subtraction for durations.
 */
uint32_t Board_Micros32(void);
uint32_t Board_Millis(void);
uint32_t Board_CycleCount(void);

/* Foreground only. Read is nonblocking; Write enqueues what fits and never waits
 * for the line. UART is 115200 8N1. Overflow is counted, not allowed to stall FOC.
 */
bool Board_UartRead(char *character);
void Board_UartDiscardInput(void); /* Drop ALL buffered RX bytes after a receive error. */
void Board_UartWrite(const char *text);
uint32_t Board_UartTxDropped(void);
uint32_t Board_UartRxDropped(void);
bool Board_ButtonPressed(void);
void Board_SetLed(bool on);

/* Foreground only; software I2C never drives high push-pull. With
 * BOARD_I2C_INTERNAL_PULLUPS=1, release selects input pull-up and low selects
 * open-drain output (STM32F1 needs the mode change). Nominal clock <=50 kHz
 * at the default 10 us half period; each transaction has a 2 ms DWT deadline.
 * First read (and retry after a bus fault) sets volatile
 * CONF SF=11 (2x, fastest), preserves other bits and verifies both CONF bytes;
 * failure returns false/BUS_ERROR. No OTP write or per-frame CONF read.
 * Checks AS5600 MD/ML/MH on EVERY read before returning its 12-bit RAW ANGLE
 * in [0, 2*pi). Never call from the ADC ISR.
 */
bool Board_ReadEncoder(float *radians);
BoardEncoderStatus Board_EncoderStatus(void);

/* Call from a healthy foreground path only. Hardware also refuses a feed unless
 * at least one new ADC pair has completed since the last feed. Nominal timeout
 * is ~101 ms at LSI=40 kHz (LSI tolerance affects the actual timeout).
 */
void Board_FeedWatchdog(void);

/* Terminal stopped-maintenance path only: EN/PWM/ADC off, IRQs disabled and
 * watchdog refreshed once. Caller must finish a bounded flash operation and
 * reset; it must never return to normal application service. */
void Board_PrepareConfigurationReset(void);

/* Called in the highest-priority ADC ISR at BOARD_ADC_HZ (default 2 kHz),
 * once per twelve carrier cycles at the default 24 kHz PWM rate. It must return
 * within 1/BOARD_ADC_HZ (500 us); no formatting, blocking calls or I2C.
 * U = PB0/ADC1_IN8; W = PB1/ADC2_IN9. Reconstruct V in application code.
 */
void App_OnAdcSample(uint16_t raw_u, uint16_t raw_w);

#ifdef __cplusplus
}
#endif
#endif
