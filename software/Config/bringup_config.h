#ifndef GIMBAL_BRINGUP_CONFIG_H
#define GIMBAL_BRINGUP_CONFIG_H

/* Debug-only open-loop switches.  They are separate from the production
 * power-on alignment in motor_setup.h and cannot be enabled together. */
#ifndef MOTOR_OPEN_AUTOTEST
#define MOTOR_OPEN_AUTOTEST               0
#endif
#define MOTOR_OPEN_AUTOTEST_DELAY_MS      2000u
#ifndef MOTOR_OPEN_AUTOTEST_CONTINUOUS
#define MOTOR_OPEN_AUTOTEST_CONTINUOUS    0
#endif
#ifndef MOTOR_OPEN_SPIN_ELECTRICAL_RAD_S
#define MOTOR_OPEN_SPIN_ELECTRICAL_RAD_S  40.0f
#endif
#ifndef MOTOR_OPEN_SPIN_VOLTAGE_V
#define MOTOR_OPEN_SPIN_VOLTAGE_V         2.0f
#endif
/* Only the separate spin profile overrides POR gating for SWD reset testing. */
#ifndef MOTOR_OPEN_AUTOTEST_IGNORE_RESET
#define MOTOR_OPEN_AUTOTEST_IGNORE_RESET  0
#endif
/* Production keeps ISR deadline violations fatal. */
#ifndef MOTOR_ADC_OVERRUN_TOLERANT
#define MOTOR_ADC_OVERRUN_TOLERANT       0
#endif
/* When set, never raise EN; PWM remains neutral while diagnostics run. */
#ifndef MOTOR_OUTPUT_DISABLED
#define MOTOR_OUTPUT_DISABLED            0
#endif
/* Fixed-size capture is allocated only in explicitly selected test profiles. */
#ifndef MOTOR_CONTROL_TRACE
#define MOTOR_CONTROL_TRACE              0
#endif

#endif
