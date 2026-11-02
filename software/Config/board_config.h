#ifndef GIMBAL_BOARD_CONFIG_H
#define GIMBAL_BOARD_CONFIG_H

/* Board timing and sample-window limits. No peripheral APIs in this header. */
#define BOARD_CORE_HZ       72000000u
/* Keep the switching carrier above audible frequency. Synchronized sampling
 * matches the 2 kHz current loop: measured current PI can take about 235 us
 * on this software-floating-point MCU, leaving margin in a 500 us period.
 * TIM3 divides the TIM2 OC4REF rising-edge stream before triggering the ADC. */
#define BOARD_PWM_HZ        24000u
#define BOARD_ADC_HZ        2000u
#define BOARD_PWM_PERIOD    (BOARD_CORE_HZ / (2u * BOARD_PWM_HZ))
#define BOARD_PWM_MIN_DUTY_PERCENT  8u
#define BOARD_PWM_MAX_DUTY_PERCENT  82u
#define BOARD_ADC_SAMPLE_PERCENT    95u
#define BOARD_PWM_MIN_DUTY  ((float)BOARD_PWM_MIN_DUTY_PERCENT / 100.0f)
#define BOARD_PWM_MAX_DUTY  ((float)BOARD_PWM_MAX_DUTY_PERCENT / 100.0f)

#if BOARD_ADC_SAMPLE_PERCENT <= BOARD_PWM_MAX_DUTY_PERCENT
#error ADC sampling must occur after the maximum high-side duty edge
#endif

/* STM32F1 weak pulls exist only in input mode. Software I2C switches each
 * released line to input pull-up, and each driven-low line to open-drain.
 * Keep the weak pull as a fallback; external pull-ups provide the bus rise time.
 * Never drive a high level push-pull. Pull-up policy does not select bus speed.
 */
#ifndef BOARD_I2C_INTERNAL_PULLUPS
#define BOARD_I2C_INTERNAL_PULLUPS 1
#endif
#ifndef BOARD_I2C_HALF_PERIOD_US
/* Restore the normal bus timing after external pull-up repair. The temporary
 * 10 us bring-up setting exceeded a transaction's 2 ms wall-clock deadline
 * under the ~250 us / 500 us current-PI interrupt load. Do not relax encoder
 * freshness or disable ADC interrupts to hide this scheduling problem.
 * Override to 10 us only for stopped weak-pull diagnosis.
 */
#define BOARD_I2C_HALF_PERIOD_US 2u /* Nominal <=250 kHz, before code/IRQ overhead. */
#endif
#if (BOARD_I2C_INTERNAL_PULLUPS != 0) && (BOARD_I2C_INTERNAL_PULLUPS != 1)
#error BOARD_I2C_INTERNAL_PULLUPS must be 0 or 1
#endif
#if BOARD_I2C_HALF_PERIOD_US < 1u
#error Software I2C half period must be at least one microsecond
#endif

#if BOARD_PWM_HZ % BOARD_ADC_HZ != 0 || BOARD_PWM_HZ / BOARD_ADC_HZ < 2u
#error PWM rate must be at least twice the ADC rate with an integer ratio
#endif
#if BOARD_PWM_HZ / BOARD_ADC_HZ > 65536u
#error ADC trigger divider must fit the TIM3 16-bit counter
#endif
#if BOARD_CORE_HZ % (2u * BOARD_PWM_HZ) != 0
#error Center-aligned PWM must divide the timer clock exactly
#endif

#endif
