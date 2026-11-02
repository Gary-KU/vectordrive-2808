#include "app.h"
#include "board.h"
#include "board_calibration.h"
#include "motor_config.h"
#include "app_trace.h"
#include "can_config.h"
#include <math.h>
#include <string.h>

/* Open-loop target used by the button and the power-on auto test. */
#if MOTOR_OPEN_AUTOTEST_CONTINUOUS
#define APP_DEFAULT_OPEN_VELOCITY_RAD_S MOTOR_OPEN_SPIN_ELECTRICAL_RAD_S
#define APP_DEFAULT_OPEN_VOLTAGE_V      MOTOR_OPEN_SPIN_VOLTAGE_V
#else
#define APP_DEFAULT_OPEN_VELOCITY_RAD_S MOTOR_OPEN_TEST_ELECTRICAL_RAD_S
#define APP_DEFAULT_OPEN_VOLTAGE_V      MOTOR_OPEN_TEST_VOLTAGE_V
#endif

typedef struct {
    AppCommandKind kind;
    float value, auxiliary;
    AppCommandSource source;
    uint32_t request_tag, received_ms;
} Command;
typedef struct { float angle, velocity; uint32_t time_us; bool valid; } Encoder;

static FocController motor;
static AppStatus status;
static Encoder encoder;
static volatile bool command_pending;
static Command command;
static volatile uint32_t sample_count;
static uint32_t offset_sum_u, offset_sum_w, offset_count;
static uint16_t offset_min_u, offset_min_w, offset_max_u, offset_max_w;
static float current_scale_u, current_scale_w;
static uint32_t next_encoder_us, last_command_ms, align_ticks;
static float alignment_check_angle;
static bool alignment_check_started;
static uint8_t fast_divider;
static uint32_t active_timeout_ms;
static volatile bool pending_can_fault;
static bool persistent_calibration_usable;

#if MOTOR_OPEN_AUTOTEST
static volatile bool open_autotest_pending;
static uint32_t open_autotest_started_ms;
#endif
#if MOTOR_BOOT_AUTO_ALIGN
static volatile bool boot_align_pending;
static uint32_t boot_align_started_ms;
#endif

static bool motion_command(AppCommandKind kind)
{
    return kind == APP_CMD_OPEN || kind == APP_CMD_CURRENT || kind == APP_CMD_SPEED ||
           kind == APP_CMD_ALIGN;
}

static void cancel_pending_command(void)
{
    if (command_pending) status.last_result = APP_RESULT_CANCELLED;
    if (command_pending && command.source == APP_SOURCE_CAN) {
        status.can_completed_tag = command.request_tag;
        status.can_result = APP_RESULT_CANCELLED;
    }
    command_pending = false;
}

static void activate_source(const Command *c)
{
    status.control_source = c->source;
    last_command_ms = c->source == APP_SOURCE_CAN ? c->received_ms : Board_Millis();
    active_timeout_ms = c->source == APP_SOURCE_CAN ? CAN_MOTION_TIMEOUT_MS : MOTOR_COMMAND_TIMEOUT_MS;
    status.command_timed_out = false;
}

#if MOTOR_CURRENT_SENSE_CONFIRMED
static bool configure_current_scale(float shunt, float gain, float polarity,
                                    float reference, float full_scale, float *scale)
{
    float denominator;
    if (!isfinite(shunt) || !isfinite(gain) || !isfinite(reference) ||
        !isfinite(full_scale) || shunt <= 0.0f || gain <= 0.0f ||
        reference <= 0.0f || full_scale <= 0.0f ||
        (polarity != 1.0f && polarity != -1.0f)) return false;
    denominator = full_scale * shunt * gain;
    if (!isfinite(denominator) || denominator <= 0.0f) return false;
    *scale = polarity * reference / denominator;
    return isfinite(*scale) && *scale != 0.0f;
}
#endif

static float angle_delta(float a, float b)
{
    float d = a - b;
    if (d > FOC_PI_F) d -= FOC_TWO_PI_F;
    if (d < -FOC_PI_F) d += FOC_TWO_PI_F;
    return d;
}

static FocConfig default_config(void)
{
    FocConfig c;
    memset(&c, 0, sizeof(c));
    FocParameters_SetDefaults(&c);
    c.fast_dt_s = 1.0f / (float)MOTOR_CONTROL_HZ;
    c.speed_loop_divider = MOTOR_SPEED_LOOP_DIVIDER;
    c.pole_pairs = MOTOR_POLE_PAIRS;
    c.duty_min = MOTOR_PWM_DUTY_MIN; c.duty_max = MOTOR_PWM_DUTY_MAX;
    c.overcurrent_trip_a = MOTOR_OVERCURRENT_TRIP_A;
    c.open_voltage_slew_v_per_s = 1.0f;
    c.open_velocity_slew_rad_s2 = 20.0f;
    c.open_velocity_limit_rad_s = 100.0f;
    return c;
}

static void begin_offset_calibration(void)
{
    Board_SetDriverEnabled(false);
    Foc_Stop(&motor);
    status.control_source = APP_SOURCE_NONE;
    status.offset_ready = false;
    status.zero_valid = false;
    offset_count = offset_sum_u = offset_sum_w = 0u;
    offset_min_u = offset_min_w = 4095u;
    offset_max_u = offset_max_w = 0u;
}

static void trip(uint32_t fault)
{
#if MOTOR_OPEN_AUTOTEST
    open_autotest_pending = false;
#endif
#if MOTOR_BOOT_AUTO_ALIGN
    boot_align_pending = false;
#endif
    Board_SetDriverEnabled(false);
    Foc_Stop(&motor);
    if (status.faults == 0u) ++status.can_session_epoch;
    status.faults |= fault;
    status.control_source = APP_SOURCE_NONE;
    if (command_pending && motion_command(command.kind)) cancel_pending_command();
    status.foc.mode = FOC_MODE_FAULT;
    status.foc.pwm_enabled = false;
    status.foc.fault = status.faults;
    /* Every physical/numeric fault invalidates electrical calibration. */
    status.zero_valid = false;
}

bool App_Init(void)
{
    FocConfig c = default_config();
    BoardCalibrationInfo saved;
    bool calibration_matches;
    BoardCalibration_Load();
    saved = BoardCalibration_GetInfo();
    calibration_matches = saved.valid && saved.motor_model == (uint16_t)MOTOR_MODEL &&
        saved.pole_pairs == (uint8_t)MOTOR_POLE_PAIRS &&
        (saved.sensor_direction == 1 || saved.sensor_direction == -1);
    memset(&status, 0, sizeof(status));
    memset(&encoder, 0, sizeof(encoder));
    command_pending = false; sample_count = 0u; fast_divider = 0u;
    pending_can_fault = false; align_ticks = 0u; alignment_check_started = false;
#if MOTOR_OPEN_AUTOTEST
    open_autotest_started_ms = Board_Millis();
#if MOTOR_OPEN_AUTOTEST_IGNORE_RESET
    /* Bring-up spin build: run after any reset so a SWD flash + reset spins. */
    open_autotest_pending = true;
#else
    open_autotest_pending = (Board_ResetCause() & BOARD_RESET_POWER_ON_MASK) != 0u &&
        (Board_ResetCause() & BOARD_RESET_AUTOTEST_INHIBIT_MASK) == 0u;
#endif
#endif
#if MOTOR_BOOT_AUTO_ALIGN
    boot_align_started_ms = Board_Millis();
    boot_align_pending = (Board_ResetCause() & BOARD_RESET_POWER_ON_MASK) != 0u &&
        (Board_ResetCause() & BOARD_RESET_AUTOTEST_INHIBIT_MASK) == 0u &&
        !saved.valid && !saved.storage_error;
#endif
    active_timeout_ms = MOTOR_COMMAND_TIMEOUT_MS;
    next_encoder_us = Board_Micros32();
    persistent_calibration_usable = calibration_matches;
    status.calibration_stored = saved.valid;
    status.calibration_storage_error = saved.storage_error;
    status.calibration_mismatch = saved.valid && !calibration_matches;
    status.sensor_direction = calibration_matches ? saved.sensor_direction :
        MOTOR_SENSOR_DIRECTION_DEFAULT;
    /* The direction is a board wiring property, not the requested torque
     * sign.  Production auto-alignment uses the verified compile-time value. */
    status.direction_confirmed = calibration_matches ||
        (MOTOR_BOOT_AUTO_ALIGN != 0 && !saved.storage_error && !status.calibration_mismatch);
    status.bus_voltage_v = c.vbus_v;
    status.sense_confirmed = MOTOR_CURRENT_SENSE_CONFIRMED != 0;
    current_scale_u = current_scale_w = 0.0f;
    if (!Foc_Init(&motor, &c)) { trip(APP_FAULT_CONFIG); return false; }
#if MOTOR_CURRENT_SENSE_CONFIRMED
    {
        if (!configure_current_scale(MOTOR_SHUNT_U_OHM, MOTOR_SENSE_GAIN_U,
                                    MOTOR_SENSE_POLARITY_U, MOTOR_ADC_REFERENCE_V,
                                    MOTOR_ADC_FULL_SCALE, &current_scale_u) ||
            !configure_current_scale(MOTOR_SHUNT_W_OHM, MOTOR_SENSE_GAIN_W,
                                    MOTOR_SENSE_POLARITY_W, MOTOR_ADC_REFERENCE_V,
                                    MOTOR_ADC_FULL_SCALE, &current_scale_w)) {
            trip(APP_FAULT_CONFIG); return false;
        }
    }
#endif
    begin_offset_calibration();
    if (calibration_matches) {
        if (!Foc_SetElectricalZero(&motor, saved.electrical_zero_rad)) {
            trip(APP_FAULT_CONFIG); return false;
        }
        status.electrical_zero_rad = motor.electrical_zero_rad;
    }
    return true;
}

static AppResult submit_command(AppCommandKind kind, float value, float auxiliary,
                                AppCommandSource source, uint32_t request_tag,
                                uint32_t received_ms)
{
    uint32_t mask;
    if (!isfinite(value) || !isfinite(auxiliary)) return APP_RESULT_RANGE;
    mask = Board_EnterCritical();
#if MOTOR_OPEN_AUTOTEST
    /* Explicit local/CAN control always takes precedence over a startup test.
     * Even a request rejected as BUSY prevents a later surprise startup. */
    open_autotest_pending = false;
#endif
#if MOTOR_BOOT_AUTO_ALIGN
    boot_align_pending = false;
#endif
    if (kind == APP_CMD_STOP || kind == APP_CMD_ESTOP) {
        /* Stop takes priority over any command not yet consumed by the ISR. */
        Board_SetDriverEnabled(false);
        cancel_pending_command();
        if (source == APP_SOURCE_LOCAL || kind == APP_CMD_ESTOP) ++status.can_session_epoch;
    } else if (command_pending) {
        Board_ExitCritical(mask); return APP_RESULT_BUSY;
    }
    command.kind = kind; command.value = value; command.auxiliary = auxiliary;
    command.source = source; command.request_tag = request_tag;
    command.received_ms = received_ms;
    command_pending = true;
    status.last_result = APP_RESULT_PENDING;
    Board_ExitCritical(mask);
    return APP_RESULT_PENDING;
}

AppResult App_Submit(AppCommandKind kind, float value, float auxiliary)
{
    return submit_command(kind, value, auxiliary, APP_SOURCE_LOCAL, 0u, Board_Millis());
}

void App_CancelStartupMotion(void)
{
#if MOTOR_OPEN_AUTOTEST || MOTOR_BOOT_AUTO_ALIGN
    const uint32_t mask = Board_EnterCritical();
#if MOTOR_OPEN_AUTOTEST
    open_autotest_pending = false;
#endif
#if MOTOR_BOOT_AUTO_ALIGN
    boot_align_pending = false;
#endif
    Board_ExitCritical(mask);
#endif
}

AppResult App_SubmitCan(AppCommandKind kind, float value, float auxiliary,
                        uint32_t request_tag, uint32_t received_ms)
{
    if (request_tag == 0u) return APP_RESULT_RANGE;
    return submit_command(kind, value, auxiliary, APP_SOURCE_CAN, request_tag, received_ms);
}

void App_NotifyCanTransportFault(void)
{
    const uint32_t mask = Board_EnterCritical();
    if (status.control_source == APP_SOURCE_CAN ||
        (command_pending && command.source == APP_SOURCE_CAN && motion_command(command.kind))) {
        Board_SetDriverEnabled(false);
        pending_can_fault = true;
        cancel_pending_command();
    }
    Board_ExitCritical(mask);
}

static AppResult execute_command(const Command *c)
{
    const bool stopped = motor.mode == FOC_MODE_STOP && !Board_DriverEnabled();
    if (c->kind == APP_CMD_PARAM_READ) {
        const unsigned id = c->auxiliary >= 1.0f && c->auxiliary <= (float)APP_PARAM_COUNT ?
            (unsigned)c->auxiliary : 0u;
        if ((float)id != c->auxiliary || c->value != 0.0f ||
            !FocParameters_Read(&motor.config,id,&status.can_parameter_value))
            return APP_RESULT_RANGE;
        return APP_RESULT_OK; /* Read-only, including during motion/fault; no keepalive. */
    }
    if (c->kind == APP_CMD_STOP) {
        Foc_Stop(&motor); Board_SetDriverEnabled(false);
        if (align_ticks != 0u) status.zero_valid = false;
        align_ticks = 0u;
        status.control_source = APP_SOURCE_NONE;
        return APP_RESULT_OK;
    }
    if (c->kind == APP_CMD_ESTOP) {
        trip(APP_FAULT_ESTOP); align_ticks = 0u;
        return APP_RESULT_OK;
    }
    if (status.control_source != APP_SOURCE_NONE && status.control_source != c->source)
        return APP_RESULT_CONTROL_OWNED;
    if (c->source == APP_SOURCE_CAN && motion_command(c->kind) &&
        (uint32_t)(Board_Millis() - c->received_ms) > CAN_COMMAND_MAX_AGE_MS)
        return APP_RESULT_STALE;
    if (c->kind == APP_CMD_CLEAR) {
        if (Board_DriverEnabled() || (motor.mode != FOC_MODE_STOP && motor.mode != FOC_MODE_FAULT))
            return APP_RESULT_NEED_STOP;
        if (Board_FaultFlags() != 0u) return APP_RESULT_FAULT;
        Board_SetDriverEnabled(false);
        if (!Foc_ClearFault(&motor)) return APP_RESULT_FAULT;
        Foc_Stop(&motor); status.faults = 0u; status.command_timed_out = false;
        align_ticks = 0u;
        begin_offset_calibration();
        return APP_RESULT_OK;
    }
    if (status.faults != 0u) return APP_RESULT_FAULT;
    if (c->kind == APP_CMD_PARAM_WRITE) {
        FocConfig config = motor.config;
        const float previous_zero = motor.electrical_zero_rad;
        const bool zero_valid = status.zero_valid;
        const unsigned id = c->auxiliary >= 1.0f && c->auxiliary <= (float)APP_PARAM_COUNT ?
            (unsigned)c->auxiliary : 0u;
        const FocParameterDescriptor *parameter = FocParameters_Find(id);
        if (!stopped) return APP_RESULT_NEED_STOP;
        if (parameter == NULL || (float)id != c->auxiliary ||
            !FocParameters_WriteConfig(&config,id,c->value))
            return APP_RESULT_RANGE;
        /* Request range comes from the selected motor descriptor. A higher
         * request does not raise the independent, non-writable phase trip. */
        if (!Foc_Init(&motor, &config)) { trip(APP_FAULT_CONFIG); return APP_RESULT_FAULT; }
        /* Reinitialization clears all PI integrals and cached voltage constants.
         * Gain/limit changes preserve a real alignment; BUS follows its original rule. */
        if (parameter->invalidates_zero) {
            status.bus_voltage_v = config.vbus_v;
            status.zero_valid = false;
        } else {
            if (!Foc_SetElectricalZero(&motor, previous_zero)) {
                trip(APP_FAULT_CONFIG); return APP_RESULT_FAULT;
            }
            status.zero_valid = zero_valid;
            status.electrical_zero_rad = motor.electrical_zero_rad;
        }
        return APP_RESULT_OK;
    }
    if (c->kind == APP_CMD_CALIBRATE) {
        if (!stopped) return APP_RESULT_NEED_STOP;
        begin_offset_calibration(); return APP_RESULT_OK;
    }
    if (c->kind == APP_CMD_BUS) {
        FocConfig config;
        if (!stopped) return APP_RESULT_NEED_STOP;
        config = motor.config;
        if (!FocParameters_WriteConfig(&config,FOC_PARAMETER_BUS_VOLTAGE,c->value))
            return APP_RESULT_RANGE;
        if (!Foc_Init(&motor, &config)) return APP_RESULT_FAULT;
        status.bus_voltage_v = c->value; status.zero_valid = false;
        return APP_RESULT_OK;
    }
    if (c->kind == APP_CMD_DIRECTION) {
        if (!stopped) return APP_RESULT_NEED_STOP;
        if (c->value != 1.0f && c->value != -1.0f) return APP_RESULT_RANGE;
        status.sensor_direction = c->value > 0.0f ? 1 : -1;
        status.direction_confirmed = true; status.zero_valid = false;
        persistent_calibration_usable = false;
        encoder.valid = false;
        return APP_RESULT_OK;
    }
    if (!status.offset_ready) return APP_RESULT_CALIBRATING;
    if (c->kind == APP_CMD_OPEN) {
        if (!stopped && motor.mode != FOC_MODE_OPEN_LOOP) return APP_RESULT_NEED_STOP;
        if (fabsf(c->value) > 100.0f || c->auxiliary < 0.0f ||
            c->auxiliary > motor.config.voltage_limit_v) return APP_RESULT_RANGE;
        if (!Foc_SetOpenLoop(&motor, c->value, c->auxiliary)) return APP_RESULT_FAULT;
        activate_source(c);
        return APP_RESULT_OK;
    }
    if (c->kind == APP_CMD_ALIGN) {
        /* CAN alignment is kept alive with fresh ALIGN frames; repeated frames
         * continue the same alignment and do not restart its settling timer. */
        if (c->source == APP_SOURCE_CAN && motor.mode == FOC_MODE_ALIGNMENT) {
            if (!status.sensor_valid) return APP_RESULT_SENSOR;
            activate_source(c); return APP_RESULT_OK;
        }
        if (!stopped) return APP_RESULT_NEED_STOP;
        if (!status.direction_confirmed) return APP_RESULT_NEED_DIRECTION;
        if (!status.sensor_valid) return APP_RESULT_SENSOR;
        if (c->source == APP_SOURCE_CAN && status.zero_valid) return APP_RESULT_OK;
        status.zero_valid = false;
        if (!Foc_SetAlignment(&motor, 0.0f, MOTOR_ALIGN_VOLTAGE_V)) return APP_RESULT_FAULT;
        align_ticks = 1u; alignment_check_started = false;
        activate_source(c);
        return APP_RESULT_OK;
    }
    if (c->kind == APP_CMD_CURRENT || c->kind == APP_CMD_SPEED) {
        const FocMode wanted = c->kind == APP_CMD_CURRENT ? FOC_MODE_CURRENT : FOC_MODE_SPEED;
        if (!stopped && motor.mode != wanted) return APP_RESULT_NEED_STOP;
        if (!status.sense_confirmed) return APP_RESULT_SENSE_UNKNOWN;
        if (!status.direction_confirmed) return APP_RESULT_NEED_DIRECTION;
        if (!status.zero_valid) return APP_RESULT_NEED_ZERO;
        if (!status.sensor_valid) return APP_RESULT_SENSOR;
        if (c->kind == APP_CMD_CURRENT) {
            if (fabsf(c->value) > motor.config.iq_limit_a) return APP_RESULT_RANGE;
            if (!Foc_SetCurrent(&motor, c->value)) return APP_RESULT_FAULT;
        } else {
            if (fabsf(c->value) > motor.config.speed_limit_rad_s) return APP_RESULT_RANGE;
            if (!Foc_SetSpeed(&motor, c->value)) return APP_RESULT_FAULT;
        }
        activate_source(c);
        return APP_RESULT_OK;
    }
    return APP_RESULT_RANGE;
}

static void collect_offset(uint16_t u, uint16_t w)
{
    if (status.offset_ready || status.faults != 0u) return;
    offset_sum_u += u; offset_sum_w += w; ++offset_count;
    if (u < offset_min_u) offset_min_u = u;
    if (w < offset_min_w) offset_min_w = w;
    if (u > offset_max_u) offset_max_u = u;
    if (w > offset_max_w) offset_max_w = w;
    if (offset_count == MOTOR_OFFSET_SAMPLE_COUNT) {
        status.offset_u = (float)offset_sum_u / (float)offset_count;
        status.offset_w = (float)offset_sum_w / (float)offset_count;
        if (status.offset_u < 512.0f || status.offset_u > 3583.0f ||
            status.offset_w < 512.0f || status.offset_w > 3583.0f ||
            (uint32_t)(offset_max_u - offset_min_u) > MOTOR_OFFSET_MAX_NOISE_COUNTS ||
            (uint32_t)(offset_max_w - offset_min_w) > MOTOR_OFFSET_MAX_NOISE_COUNTS) {
            trip(APP_FAULT_OFFSET);
        } else {
            status.offset_ready = true;
            if (persistent_calibration_usable) {
                if (!Foc_SetElectricalZero(&motor, status.electrical_zero_rad)) {
                    trip(APP_FAULT_CONFIG); return;
                }
                status.zero_valid = true;
            }
        }
    }
}

void App_OnAdcSample(uint16_t u, uint16_t w)
{
    const uint32_t started = Board_CycleCount();
    FocInput in;
    uint32_t age, cycles;
    ++sample_count;
    status.raw_u = u; status.raw_w = w;
    if (pending_can_fault) { pending_can_fault = false; trip(APP_FAULT_CAN); }
    if (Board_FaultFlags() != 0u) trip(APP_FAULT_BOARD);
    collect_offset(u, w);
    /* Check ADC saturation and phase overcurrent on EVERY synchronized ADC sample. */
    if (status.sense_confirmed) {
        in.ia_a = ((float)u - status.offset_u) * current_scale_u;
        in.ic_a = ((float)w - status.offset_w) * current_scale_w;
    } else {
        /* An unknown shunt/gain has no amperes conversion. Avoid software
         * floating-point work on every ADC sample in voltage-only bring-up. */
        in.ia_a = 0.0f;
        in.ic_a = 0.0f;
    }
    if (Board_DriverEnabled() && status.sense_confirmed) {
        if (u < MOTOR_ADC_RAIL_MARGIN_COUNTS || w < MOTOR_ADC_RAIL_MARGIN_COUNTS ||
            u > 4095u - MOTOR_ADC_RAIL_MARGIN_COUNTS ||
            w > 4095u - MOTOR_ADC_RAIL_MARGIN_COUNTS) trip(APP_FAULT_ADC_RAIL);
        if (fabsf(in.ia_a) > MOTOR_OVERCURRENT_TRIP_A ||
            fabsf(in.ic_a) > MOTOR_OVERCURRENT_TRIP_A ||
            fabsf(in.ia_a + in.ic_a) > MOTOR_OVERCURRENT_TRIP_A) trip(FOC_FAULT_OVERCURRENT);
    }
    if (++fast_divider < (MOTOR_ADC_HZ / MOTOR_CONTROL_HZ)) return;
    fast_divider = 0u;
    age = Board_Micros32() - encoder.time_us;
    status.sensor_valid = encoder.valid && age <= MOTOR_ENCODER_TIMEOUT_US;
    in.sensor_valid = status.sensor_valid;
    in.mechanical_velocity_rad_s = status.sensor_valid ? encoder.velocity : 0.0f;
    in.mechanical_angle_rad = status.sensor_valid ? Foc_WrapAngle(encoder.angle +
        encoder.velocity * ((float)age * 0.000001f)) : 0.0f;
    status.angle_rad = encoder.angle; status.velocity_rad_s = encoder.velocity;
    if (command_pending) {
        const Command next = command;
        command_pending = false;
        status.last_result = execute_command(&next);
        if (next.source == APP_SOURCE_CAN) {
            if (next.kind == APP_CMD_PARAM_READ || next.kind == APP_CMD_PARAM_WRITE) {
                const unsigned id = next.auxiliary >= 1.0f && next.auxiliary <= (float)APP_PARAM_COUNT ?
                    (unsigned)next.auxiliary : 0u;
                status.can_parameter_value = 0.0f;
                (void)FocParameters_Read(&motor.config,id,&status.can_parameter_value);
            }
            status.can_completed_tag = next.request_tag;
            status.can_result = status.last_result;
        }
    }
    if (status.faults != 0u) return;
    if ((motor.mode == FOC_MODE_CURRENT || motor.mode == FOC_MODE_SPEED ||
         motor.mode == FOC_MODE_ALIGNMENT) && !status.sensor_valid) {
        trip(APP_FAULT_ENCODER); return;
    }
#if MOTOR_OPEN_AUTOTEST_CONTINUOUS
    /* Spin build: open loop keeps running until an explicit STOP or a fault. */
    const bool continuous_open_loop = (motor.mode == FOC_MODE_OPEN_LOOP);
#else
    const bool continuous_open_loop = false;
#endif
    if ((motor.mode == FOC_MODE_OPEN_LOOP || motor.mode == FOC_MODE_CURRENT ||
         motor.mode == FOC_MODE_SPEED ||
         (motor.mode == FOC_MODE_ALIGNMENT && status.control_source == APP_SOURCE_CAN)) &&
        !continuous_open_loop &&
        (uint32_t)(Board_Millis() - last_command_ms) >= active_timeout_ms) {
        Foc_Stop(&motor); Board_SetDriverEnabled(false);
        if (status.control_source == APP_SOURCE_CAN) ++status.can_session_epoch;
        if (align_ticks != 0u) status.zero_valid = false;
        align_ticks = 0u; status.control_source = APP_SOURCE_NONE;
        status.command_timed_out = true;
    }
    if (motor.mode == FOC_MODE_ALIGNMENT) {
        ++align_ticks;
        if (align_ticks >= (MOTOR_ALIGN_DURATION_MS - 300u) * (MOTOR_CONTROL_HZ / 1000u)) {
            if (!alignment_check_started) {
                alignment_check_angle = encoder.angle; alignment_check_started = true;
            }
            if (fabsf(angle_delta(encoder.angle, alignment_check_angle)) > 0.04f) {
                trip(APP_FAULT_ALIGNMENT); return;
            }
        }
        if (align_ticks >= MOTOR_ALIGN_DURATION_MS * (MOTOR_CONTROL_HZ / 1000u)) {
            if (!Foc_SetElectricalZero(&motor, (float)MOTOR_POLE_PAIRS * encoder.angle)) {
                trip(FOC_FAULT_COMMAND); return;
            }
            Foc_Stop(&motor); Board_SetDriverEnabled(false);
            status.control_source = APP_SOURCE_NONE;
            status.zero_valid = true;
            status.electrical_zero_rad = motor.electrical_zero_rad;
            align_ticks = 0u;
        }
    }
    if (!Foc_Step(&motor, &in, &status.foc)) {
        trip(status.foc.fault); return;
    }
#if MOTOR_OUTPUT_DISABLED
    /* Output disabled for bring-up: hold the driver off and PWM neutral. */
    Board_SetDriverEnabled(false);
    Board_SetPwm(0.45f, 0.45f, 0.45f);
#else
    if (status.foc.pwm_enabled && status.offset_ready) {
        Board_SetPwm(status.foc.duty[0], status.foc.duty[1], status.foc.duty[2]);
        Board_SetDriverEnabled(true);
    } else {
        Board_SetDriverEnabled(false);
        Board_SetPwm(0.45f, 0.45f, 0.45f);
    }
#endif
#if MOTOR_CONTROL_TRACE
    AppTrace_Record(sample_count, u, w, &status.foc,
                    in.mechanical_angle_rad, in.mechanical_velocity_rad_s);
#endif
    cycles = Board_CycleCount() - started;
    if (cycles > status.max_isr_cycles) status.max_isr_cycles = cycles;
#if !MOTOR_ADC_OVERRUN_TOLERANT
    if (cycles >= MOTOR_CPU_HZ / MOTOR_ADC_HZ) trip(APP_FAULT_OVERRUN);
#endif
}

static void poll_encoder(uint32_t now)
{
    float raw;
    Encoder next, previous;
    uint32_t mask, dt;
    int8_t direction;
    if ((int32_t)(now - next_encoder_us) < 0) return;
    next_encoder_us = now + MOTOR_ENCODER_PERIOD_US;
    mask = Board_EnterCritical();
    previous = encoder; direction = status.sensor_direction;
    Board_ExitCritical(mask);
    next = previous;
    if (Board_ReadEncoder(&raw) && isfinite(raw)) {
        /* Timestamp after transaction; interrupted I2C does not masquerade as fresh data. */
        next.time_us = Board_Micros32();
        next.angle = Foc_WrapAngle(raw * (float)direction);
        dt = next.time_us - previous.time_us;
        next.velocity = 0.0f;
        next.valid = true;
        if (previous.valid && dt > 0u && dt <= MOTOR_ENCODER_TIMEOUT_US) {
            float seconds = (float)dt * 0.000001f;
            const float measured = angle_delta(next.angle, previous.angle) / seconds;
            const float gain = seconds / (MOTOR_SPEED_FILTER_TAU_S + seconds);
            if (fabsf(measured) > 200.0f) next.valid = false;
            else next.velocity = previous.velocity + gain * (measured - previous.velocity);
        }
    } else next.valid = false;
    mask = Board_EnterCritical();
    if (status.sensor_direction == direction) encoder = next;
    else encoder.valid = false; /* A direction command interrupted the I2C transaction. */
    Board_ExitCritical(mask);
}

void App_Poll(void)
{
    static uint32_t last_feed_samples, button_change_ms;
    static bool button_raw, button_stable;
    const uint32_t now_ms = Board_Millis();
    const bool pressed = Board_ButtonPressed();
    poll_encoder(Board_Micros32());
#if MOTOR_BOOT_AUTO_ALIGN
    {
        bool start_alignment = false;
        const uint32_t mask = Board_EnterCritical();
        if (boot_align_pending) {
            const uint32_t elapsed = now_ms - boot_align_started_ms;
            if (elapsed >= MOTOR_BOOT_ALIGN_START_TIMEOUT_MS) {
                /* Consume the one-shot.  A missing encoder/ADC must not cause
                 * an unexpected start minutes later after a wiring repair. */
                boot_align_pending = false;
            } else if (status.offset_ready && status.sensor_valid &&
                       status.faults == 0u && Board_FaultFlags() == 0u &&
                       motor.mode == FOC_MODE_STOP &&
                       status.control_source == APP_SOURCE_NONE &&
                       !command_pending) {
                boot_align_pending = false;
                start_alignment = true;
            }
        }
        Board_ExitCritical(mask);
        if (start_alignment) {
            /* Reuse the same checked command path as manual/CAN alignment.
             * Completion stores the electrical zero in RAM, disables EN and
             * returns to STOP; it never writes Flash or starts torque mode. */
            (void)App_Submit(APP_CMD_ALIGN, 0.0f, 0.0f);
        }
    }
#endif
    if (pressed != button_raw) { button_raw = pressed; button_change_ms = now_ms; }
    if (button_stable != button_raw && (uint32_t)(now_ms - button_change_ms) >= 30u) {
        button_stable = button_raw;
        if (button_stable) {
            if (Board_DriverEnabled()) (void)App_Submit(APP_CMD_STOP, 0.0f, 0.0f);
            else (void)App_Submit(APP_CMD_OPEN, APP_DEFAULT_OPEN_VELOCITY_RAD_S,
                                 APP_DEFAULT_OPEN_VOLTAGE_V);
        }
    }
#if MOTOR_OPEN_AUTOTEST
    if (open_autotest_pending &&
        (uint32_t)(now_ms - open_autotest_started_ms) >= MOTOR_OPEN_AUTOTEST_DELAY_MS) {
        const uint32_t mask = Board_EnterCritical();
        if (open_autotest_pending) {
            /* Consume the attempt even when a prerequisite is missing.
             * CLEAR, later calibration, timeout and idle cannot re-arm it. */
            open_autotest_pending = false;
            if (status.offset_ready && status.faults == 0u && Board_FaultFlags() == 0u &&
                motor.mode == FOC_MODE_STOP && status.control_source == APP_SOURCE_NONE &&
                !command_pending) {
                (void)App_Submit(APP_CMD_OPEN, APP_DEFAULT_OPEN_VELOCITY_RAD_S,
                                 APP_DEFAULT_OPEN_VOLTAGE_V);
            }
        }
        Board_ExitCritical(mask);
    }
#endif
    Board_SetLed(status.faults != 0u ? ((now_ms / 100u) & 1u) != 0u :
                 (Board_DriverEnabled() || ((now_ms / 500u) & 1u) != 0u));
    if (sample_count != last_feed_samples) {
        last_feed_samples = sample_count;
        Board_FeedWatchdog();
    }
}

bool App_CanReinitializeCan(void)
{
    const uint32_t mask = Board_EnterCritical();
    bool allowed = (motor.mode == FOC_MODE_STOP || motor.mode == FOC_MODE_FAULT) &&
        !Board_DriverEnabled() && !command_pending && !pending_can_fault &&
        status.control_source == APP_SOURCE_NONE;
#if MOTOR_OPEN_AUTOTEST
    allowed = allowed && !open_autotest_pending;
#endif
#if MOTOR_BOOT_AUTO_ALIGN
    allowed = allowed && !boot_align_pending;
#endif
    Board_ExitCritical(mask);
    return allowed;
}

void App_GetStatus(AppStatus *destination)
{
    const uint32_t mask = Board_EnterCritical();
    if (destination != 0) {
        *destination = status;
        destination->adc_samples = sample_count;
        destination->board_faults = Board_FaultFlags();
        destination->driver_enabled = Board_DriverEnabled();
        for (unsigned id = 1u; id <= APP_PARAM_COUNT; ++id)
            (void)FocParameters_Read(&motor.config,id,&destination->parameters[id-1u]);
    }
    Board_ExitCritical(mask);
}

const char *App_ModeName(FocMode mode)
{
    switch (mode) {
    case FOC_MODE_STOP: return "STOP";
    case FOC_MODE_OPEN_LOOP: return "OPEN";
    case FOC_MODE_ALIGNMENT: return "ALIGN";
    case FOC_MODE_CURRENT: return "CURRENT";
    case FOC_MODE_SPEED: return "SPEED";
    default: return "FAULT";
    }
}

const char *App_ResultName(AppResult result)
{
    switch (result) {
    case APP_RESULT_OK: return "OK";
    case APP_RESULT_PENDING: return "QUEUED";
    case APP_RESULT_BUSY: return "BUSY: retry";
    case APP_RESULT_FAULT: return "FAULT: stop, inspect status, then clear";
    case APP_RESULT_CALIBRATING: return "WAIT: offset calibration";
    case APP_RESULT_NEED_STOP: return "STOP_REQUIRED";
    case APP_RESULT_SENSE_UNKNOWN: return "SENSE_UNKNOWN: configure shunts/gain/polarity";
    case APP_RESULT_NEED_DIRECTION: return "DIRECTION_REQUIRED: verify open-loop direction, then dir +/-1";
    case APP_RESULT_NEED_ZERO: return "ZERO_REQUIRED: run align";
    case APP_RESULT_SENSOR: return "ENCODER_INVALID: check AS5600 magnet/bus";
    case APP_RESULT_CANCELLED: return "CANCELLED_BY_STOP";
    case APP_RESULT_CONTROL_OWNED: return "CONTROL_OWNED: stop current controller first";
    case APP_RESULT_STALE: return "STALE_CAN_COMMAND";
    default: return "RANGE_OR_SYNTAX";
    }
}
