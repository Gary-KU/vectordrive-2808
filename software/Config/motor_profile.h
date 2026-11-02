#ifndef GIMBAL_MOTOR_PROFILE_H
#define GIMBAL_MOTOR_PROFILE_H

/* GCC: build.ps1 -Motor 2808 / -Motor 4015.
 * Keil: set MOTOR_MODEL=2808 or MOTOR_MODEL=4015 in target preprocessor defines.
 * These profiles identify the two motors used on this bench; a model number
 * alone does not guarantee the pole count of another manufacturer's motor.
 */
#define MOTOR_MODEL_2808 2808
#define MOTOR_MODEL_4015 4015
#ifndef MOTOR_MODEL
#define MOTOR_MODEL MOTOR_MODEL_2808
#endif

/* Both profiles share the same 3.3 A request/parameter ceiling. The 1 A
 * ceiling that previously distinguished 4015 has been removed on request;
 * keep the profiles separate for pole-pair count and future per-model limits. */
#if MOTOR_MODEL == MOTOR_MODEL_2808
#define MOTOR_MODEL_NAME "2808"
#define MOTOR_POLE_PAIRS 7u
#define MOTOR_IQ_REQUEST_MAX_MA 3300u
#ifndef MOTOR_CURRENT_LIMIT_A
#define MOTOR_CURRENT_LIMIT_A 3.3f
#endif
#elif MOTOR_MODEL == MOTOR_MODEL_4015
#define MOTOR_MODEL_NAME "4015"
#define MOTOR_POLE_PAIRS 11u
#define MOTOR_IQ_REQUEST_MAX_MA 3300u
#ifndef MOTOR_CURRENT_LIMIT_A
#define MOTOR_CURRENT_LIMIT_A 2.0f
#endif
#else
#error Unsupported MOTOR_MODEL: select 2808 or 4015
#endif

/* Request encoding/parameter ceiling only, not a measured current capability.
 * Keep the independent fixed phase-current trip in motor_setup.h unchanged.
 * A request above the trip can saturate in voltage or trigger a real fault;
 * it is never silently reduced to the phase-protection threshold. */
#define MOTOR_IQ_REQUEST_MAX_A ((float)MOTOR_IQ_REQUEST_MAX_MA / 1000.0f)

/* Direction still depends on phase wiring and encoder mounting.  This board's
 * verified U/V/W and AS5600 mounting use +1 for both profiles; override
 * MOTOR_SENSOR_DIRECTION_DEFAULT at build time if the wiring is changed. */
#endif
