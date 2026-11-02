#ifndef GIMBAL_MOTOR_SETUP_H
#define GIMBAL_MOTOR_SETUP_H

#include "board_config.h"
#include "motor_profile.h"

/* Board/motor setup and fixed safeguards, not the nine CAN tuning parameters. */
#define MOTOR_OPEN_TEST_VOLTAGE_V        0.8f
#define MOTOR_OPEN_TEST_ELECTRICAL_RAD_S  20.0f
#ifndef MOTOR_ALIGN_VOLTAGE_V
#define MOTOR_ALIGN_VOLTAGE_V            3.0f
#endif
#define MOTOR_ALIGN_DURATION_MS          4000u

/* Production startup calibration.  After a true power-on reset the driver
 * first finishes zero-current ADC calibration, then applies the normal ALIGN
 * vector once and returns to STOP.  Software/watchdog resets never re-arm it.
 * Any explicit local/CAN request cancels a still-pending startup alignment. */
#ifndef MOTOR_BOOT_AUTO_ALIGN
#define MOTOR_BOOT_AUTO_ALIGN            1
#endif
#ifndef MOTOR_SENSOR_DIRECTION_DEFAULT
#define MOTOR_SENSOR_DIRECTION_DEFAULT   1
#endif
#ifndef MOTOR_BOOT_ALIGN_START_TIMEOUT_MS
#define MOTOR_BOOT_ALIGN_START_TIMEOUT_MS 2000u
#endif

#if MOTOR_SENSOR_DIRECTION_DEFAULT != 1 && MOTOR_SENSOR_DIRECTION_DEFAULT != -1
#error MOTOR_SENSOR_DIRECTION_DEFAULT must be +1 or -1
#endif
#if MOTOR_BOOT_ALIGN_START_TIMEOUT_MS < 1000u
#error Startup alignment timeout must leave time for ADC offset calibration
#endif

/* Fixed phase-current protection; never writable through the parameter API. */
#ifndef MOTOR_OVERCURRENT_TRIP_A
#if MOTOR_MODEL == MOTOR_MODEL_2808
#define MOTOR_OVERCURRENT_TRIP_A         2.0f
#else
#define MOTOR_OVERCURRENT_TRIP_A         2.2f
#endif
#endif
#define MOTOR_SPEED_FILTER_TAU_S         0.02f
#define MOTOR_ENCODER_PERIOD_US          1000u
#define MOTOR_ENCODER_TIMEOUT_US         5000u

#define MOTOR_CPU_HZ                     BOARD_CORE_HZ
#define MOTOR_PWM_HZ                     BOARD_PWM_HZ
#define MOTOR_ADC_HZ                     BOARD_ADC_HZ
#define MOTOR_CONTROL_HZ                 2000u
#define MOTOR_SPEED_LOOP_DIVIDER         2u
#define MOTOR_PWM_DUTY_MIN               BOARD_PWM_MIN_DUTY
#define MOTOR_PWM_DUTY_MAX               BOARD_PWM_MAX_DUTY
#ifndef MOTOR_COMMAND_TIMEOUT_MS
#define MOTOR_COMMAND_TIMEOUT_MS         2000u
#endif
/* Refresh local open/iq/speed commands at <2 s intervals. CAN has its own 100 ms deadline. */

#if MOTOR_CONTROL_HZ > MOTOR_ADC_HZ || MOTOR_ADC_HZ % MOTOR_CONTROL_HZ != 0
#error Control rate must divide the ADC sampling rate exactly
#endif
#if MOTOR_CONTROL_HZ % MOTOR_SPEED_LOOP_DIVIDER != 0
#error Speed loop divider must divide the control rate exactly
#endif

#endif
