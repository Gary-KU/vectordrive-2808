#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

#include <stdbool.h>
#include <stdint.h>
#include "foc_parameters.h"

typedef enum {
    APP_CMD_STOP, APP_CMD_OPEN, APP_CMD_ALIGN, APP_CMD_CURRENT,
    APP_CMD_SPEED, APP_CMD_CLEAR, APP_CMD_CALIBRATE,
    APP_CMD_DIRECTION, APP_CMD_BUS, APP_CMD_ESTOP,
    APP_CMD_PARAM_READ, APP_CMD_PARAM_WRITE
} AppCommandKind;

typedef FocParameterId AppParameterId;
enum {
#define FOC_PARAMETER(symbol,id,field,default_macro,minimum,maximum,invalidates_zero) \
    APP_PARAM_##symbol = FOC_PARAMETER_##symbol,
#include "parameter_catalog.def"
#undef FOC_PARAMETER
    APP_PARAM_COUNT = FOC_PARAMETER_COUNT
};

typedef enum { APP_SOURCE_NONE = 0, APP_SOURCE_LOCAL, APP_SOURCE_CAN } AppCommandSource;

typedef enum {
    APP_RESULT_OK = 0, APP_RESULT_PENDING, APP_RESULT_BUSY,
    APP_RESULT_FAULT, APP_RESULT_CALIBRATING, APP_RESULT_NEED_STOP,
    APP_RESULT_SENSE_UNKNOWN, APP_RESULT_NEED_DIRECTION,
    APP_RESULT_NEED_ZERO, APP_RESULT_SENSOR, APP_RESULT_RANGE,
    APP_RESULT_CANCELLED, APP_RESULT_CONTROL_OWNED, APP_RESULT_STALE
} AppResult;

enum {
    APP_FAULT_BOARD = 1u << 16, APP_FAULT_OFFSET = 1u << 17,
    APP_FAULT_ADC_RAIL = 1u << 18, APP_FAULT_ENCODER = 1u << 19,
    APP_FAULT_ALIGNMENT = 1u << 20, APP_FAULT_OVERRUN = 1u << 21,
    APP_FAULT_CONFIG = 1u << 22, APP_FAULT_CAN = 1u << 23,
    APP_FAULT_ESTOP = 1u << 24
};

typedef struct {
    FocOutput foc;
    uint32_t faults, board_faults, adc_samples, max_isr_cycles;
    uint16_t raw_u, raw_w;
    float offset_u, offset_w;
    float angle_rad, velocity_rad_s, bus_voltage_v, electrical_zero_rad;
    int8_t sensor_direction;
    bool offset_ready, sense_confirmed, sensor_valid;
    bool direction_confirmed, zero_valid, command_timed_out;
    bool calibration_stored, calibration_storage_error, calibration_mismatch;
    AppResult last_result;
    AppCommandSource control_source;
    uint32_t can_completed_tag, can_session_epoch;
    AppResult can_result;
    float can_parameter_value;
    float parameters[APP_PARAM_COUNT]; /* Snapshot, indexed by parameter ID - 1. */
    bool driver_enabled;
} AppStatus;

bool App_Init(void); /* Board_Init must succeed first; sampling still stopped. */
void App_Poll(void); /* Foreground encoder/button/watchdog service; never in ISR. */
void App_OnAdcSample(uint16_t raw_u, uint16_t raw_w); /* PWM-synchronized ADC callback. */
AppResult App_Submit(AppCommandKind kind, float value, float auxiliary);
/* CAN commands use RX time, a 100 ms deadline and execution-correlated results. */
AppResult App_SubmitCan(AppCommandKind kind, float value, float auxiliary,
                        uint32_t request_tag, uint32_t received_ms);
/* Foreground/ISR-safe: immediately drops EN for an active/pending CAN motion;
 * the next ADC callback locks the transport fault and invalidates zero. */
void App_NotifyCanTransportFault(void);
/* Cancel any still-pending power-on movement before applying transport/session
 * gates.  An alignment already in progress is stopped through the normal STOP
 * command; this function only prevents a delayed surprise start. */
void App_CancelStartupMotion(void);
/* Foreground-only maintenance gate. No drive, active control source, queued
 * command, pending transport fault or scheduled startup motion is allowed.
 * A latched fault may remain: restoring diagnostics never clears/rearms it.
 * Current command producers run in foreground; ISR only consumes the queue.
 */
bool App_CanReinitializeCan(void);
void App_GetStatus(AppStatus *status);
const char *App_ResultName(AppResult result);
const char *App_ModeName(FocMode mode);

#endif
