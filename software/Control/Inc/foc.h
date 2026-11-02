#ifndef GIMBAL_FOC_H
#define GIMBAL_FOC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_PI_F       3.14159265358979323846f
#define FOC_TWO_PI_F   6.28318530717958647692f

typedef enum {
    FOC_MODE_STOP = 0,
    FOC_MODE_OPEN_LOOP,
    FOC_MODE_ALIGNMENT,
    FOC_MODE_CURRENT,
    FOC_MODE_SPEED,
    FOC_MODE_FAULT
} FocMode;

typedef enum {
    FOC_FAULT_NONE = 0,
    FOC_FAULT_CONFIG = (1u << 0),
    FOC_FAULT_INPUT = (1u << 1),
    FOC_FAULT_SENSOR = (1u << 2),
    FOC_FAULT_OVERCURRENT = (1u << 3),
    FOC_FAULT_COMMAND = (1u << 4),
    FOC_FAULT_NUMERIC = (1u << 5)
} FocFault;

typedef struct {
    float fast_dt_s;                  /* Fixed Foc_Step period; board uses 0.0005 s. */
    uint16_t speed_loop_divider;       /* Board uses 2: speed PI period = 0.001 s. */
    uint16_t pole_pairs;              /* Selected motor profile; 2808=7, 4015=11. */
    float vbus_v;                     /* Measured/configured bus; no board divider. */
    float duty_min;                   /* Normally 0.08. */
    float duty_max;                   /* Normally 0.82; leaves a low-side sample window. */
    float voltage_limit_v;            /* dq vector magnitude; further limited by PWM span. */
    float iq_limit_a;
    float overcurrent_trip_a;          /* Trips on any reconstructed phase, all active modes. */
    float iq_slew_a_per_s;
    float open_voltage_slew_v_per_s;
    float open_velocity_slew_rad_s2;
    float open_velocity_limit_rad_s;  /* Electrical rad/s, independent of pole_pairs. */
    float speed_limit_rad_s;          /* Mechanical rad/s. */
    float current_kp_v_per_a;
    float current_ki_v_per_a_s;
    float speed_kp_a_per_rad_s;
    float speed_ki_a_per_rad;
} FocConfig;

typedef struct {
    float ia_a;                        /* U phase shunt current, after offset/gain calibration. */
    float ic_a;                        /* W phase shunt current, same sign convention as U. */
    float mechanical_angle_rad;        /* Encoder direction already applied, wrapped or bounded. */
    float mechanical_velocity_rad_s;   /* Encoder direction already applied. */
    bool sensor_valid;                /* Required only for CURRENT and SPEED. */
} FocInput;

typedef struct {
    float duty[3];                     /* U, V, W; neutral on STOP/FAULT. */
    bool pwm_enabled;                  /* Algorithm request; app must apply hardware interlocks. */
    FocMode mode;
    uint32_t fault;
    float electrical_angle_rad;        /* [0, 2*pi). */
    float ia_a;
    float ib_a;
    float ic_a;
    float id_a;
    float iq_a;
    float iq_reference_a;              /* Slew-limited current target. Id reference is always 0. */
    float vd_v;
    float vq_v;
    float open_velocity_rad_s;
} FocOutput;

typedef struct {
    FocConfig config;
    FocMode mode;
    uint32_t fault;
    bool initialized;
    float inverse_vbus_v;             /* Cached at initialization; avoid fast-loop division. */
    float effective_voltage_limit_v;
    float neutral_duty;
    float electrical_zero_rad;
    float electrical_angle_rad;
    float target_open_velocity_rad_s;
    float target_open_voltage_v;
    float open_velocity_rad_s;
    float open_voltage_v;
    float target_iq_a;
    float target_speed_rad_s;
    float iq_reference_a;
    float speed_iq_a;
    float integral_d_v;
    float integral_q_v;
    float integral_speed_a;
    uint16_t speed_counter;
} FocController;

/* Configuration is copied. All invalid configuration/NaN commands latch FAULT. */
bool Foc_Init(FocController *controller, const FocConfig *config);
void Foc_Stop(FocController *controller); /* Does not clear a latched fault. */
bool Foc_ClearFault(FocController *controller); /* Call only after app-level checks. */
bool Foc_SetElectricalZero(FocController *controller, float electrical_zero_rad);
bool Foc_SetOpenLoop(FocController *controller, float electrical_velocity_rad_s,
                     float voltage_v); /* q-axis voltage; both targets are slew-limited. */
bool Foc_SetAlignment(FocController *controller, float electrical_angle_rad,
                      float voltage_v); /* Stationary positive d-axis voltage, slew-limited. */
bool Foc_SetCurrent(FocController *controller, float iq_a);
bool Foc_SetSpeed(FocController *controller, float mechanical_velocity_rad_s);
/* Exactly one call per fast_dt_s. False means fault; output is always safe on failure. */
bool Foc_Step(FocController *controller, const FocInput *input, FocOutput *output);

/* Deterministic O(1) math helpers; invalid or |angle| > 1e6 returns 0. */
float Foc_WrapAngle(float angle_rad);
/* Quarter-wave LUT + linear interpolation; error < 6e-6 for wrapped [0, 2*pi) input.
 * Wrapping large multi-turn float angles necessarily loses input precision. */
void Foc_SinCos(float angle_rad, float *sine, float *cosine);

#ifdef __cplusplus
}
#endif
#endif
