#include "foc.h"
#include "motor_config.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static FocConfig test_config(void)
{
    FocConfig config = {
        .fast_dt_s = 0.0001f,
        .speed_loop_divider = 10u,
        .pole_pairs = 11u,
        .vbus_v = 12.0f,
        .duty_min = 0.08f,
        .duty_max = 0.82f,
        .voltage_limit_v = 6.0f,
        .iq_limit_a = 2.0f,
        .overcurrent_trip_a = 5.0f,
        .iq_slew_a_per_s = 200.0f,
        .open_voltage_slew_v_per_s = 10.0f,
        .open_velocity_slew_rad_s2 = 100.0f,
        .open_velocity_limit_rad_s = 500.0f,
        .speed_limit_rad_s = 100.0f,
        .current_kp_v_per_a = 0.8f,
        .current_ki_v_per_a_s = 120.0f,
        .speed_kp_a_per_rad_s = 0.2f,
        .speed_ki_a_per_rad = 2.0f
    };
    return config;
}

static void close_to(float actual, float expected, float tolerance)
{
    if (!(fabsf(actual - expected) <= tolerance)) {
        fprintf(stderr, "actual %.9g expected %.9g tolerance %.9g\n",
                (double)actual, (double)expected, (double)tolerance);
        assert(0);
    }
}

static void neutral(const FocOutput *out)
{
    assert(!out->pwm_enabled);
    close_to(out->duty[0], 0.45f, 1e-6f);
    close_to(out->duty[1], 0.45f, 1e-6f);
    close_to(out->duty[2], 0.45f, 1e-6f);
}

static void test_math(void)
{
    float max_error = 0.0f;
    int i;
    close_to(Foc_WrapAngle(0.0f), 0.0f, 0.0f);
    assert(signbit(Foc_WrapAngle(-0.0f))); /* Preserve signed-zero helper behavior. */
    close_to(Foc_WrapAngle(FOC_TWO_PI_F), 0.0f, 0.0f);
    close_to(Foc_WrapAngle(-0.25f), FOC_TWO_PI_F - 0.25f, 1e-6f);
    close_to(Foc_WrapAngle(-FOC_TWO_PI_F), 0.0f, 0.0f);
    close_to(Foc_WrapAngle(-FLT_MIN), 0.0f, 0.0f);
    close_to(Foc_WrapAngle(-1e-8f), 0.0f, 0.0f);
    close_to(Foc_WrapAngle(nextafterf(FOC_TWO_PI_F, 0.0f)),
             nextafterf(FOC_TWO_PI_F, 0.0f), 0.0f);
    close_to(Foc_WrapAngle(5.0f * FOC_TWO_PI_F + 0.3f), 0.3f, 3e-6f);
    close_to(Foc_WrapAngle(NAN), 0.0f, 0.0f);
    close_to(Foc_WrapAngle(INFINITY), 0.0f, 0.0f);
    close_to(Foc_WrapAngle(FLT_MAX), 0.0f, 0.0f);
    for (i = 0; i <= 100000; ++i) {
        float angle = FOC_TWO_PI_F * (float)i / 100000.0f;
        float s, c, error;
        Foc_SinCos(angle, &s, &c);
        error = fmaxf(fabsf(s - sinf(angle)), fabsf(c - cosf(angle)));
        if (error > max_error) max_error = error;
        assert(error < 6e-6f);
    }
    for (i = -500; i <= 500; ++i) {
        float angle = (float)i * 0.061f;
        float s, c;
        Foc_SinCos(angle, &s, &c);
        close_to(s, sinf(angle), 7e-6f);
        close_to(c, cosf(angle), 7e-6f);
    }
    printf("LUT maximum sin/cos absolute error: %.8g\n", (double)max_error);
}

static void test_stop_and_zero_voltage(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    assert(Foc_Init(&controller, &config));
    close_to(controller.inverse_vbus_v, 1.0f / config.vbus_v, 0.0f);
    close_to(controller.effective_voltage_limit_v,
             config.vbus_v * (config.duty_max - config.duty_min) / sqrtf(3.0f), 1e-6f);
    close_to(controller.neutral_duty, 0.45f, 1e-6f);
    assert(Foc_Step(&controller, NULL, &out)); /* STOP does not require sensor samples. */
    neutral(&out);
    assert(out.mode == FOC_MODE_STOP);
    assert(Foc_SetOpenLoop(&controller, 20.0f, 0.0f));
    assert(Foc_Step(&controller, &input, &out));
    assert(out.pwm_enabled);
    close_to(out.duty[0], 0.45f, 1e-6f);
    close_to(out.duty[1], 0.45f, 1e-6f);
    close_to(out.duty[2], 0.45f, 1e-6f);
    Foc_Stop(&controller);
    assert(Foc_Step(&controller, &input, &out));
    neutral(&out);
}

static void test_open_loop_slew(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    unsigned i;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetOpenLoop(&controller, 10.0f, 2.0f));
    assert(Foc_Step(&controller, &input, &out));
    close_to(out.open_velocity_rad_s, 0.01f, 1e-7f);
    close_to(out.vq_v, 0.001f, 1e-7f);
    close_to(out.electrical_angle_rad, 0.000001f, 1e-8f);
    for (i = 1u; i < 1000u; ++i) assert(Foc_Step(&controller, &input, &out));
    close_to(out.open_velocity_rad_s, 10.0f, 2e-4f);
    close_to(out.vq_v, 1.0f, 2e-5f);
    assert(Foc_SetOpenLoop(&controller, -10.0f, -2.0f));
    assert(Foc_Step(&controller, &input, &out));
    close_to(out.open_velocity_rad_s, 9.99f, 2e-4f);
    close_to(out.vq_v, 0.999f, 2e-5f);
    /* The open-loop electrical speed does not depend on the configured pole pairs. */
    config.pole_pairs = 1u;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetOpenLoop(&controller, 10.0f, 2.0f));
    assert(Foc_Step(&controller, &input, &out));
    close_to(out.open_velocity_rad_s, 0.01f, 1e-7f);
}

static void test_pwm_and_alignment(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    unsigned i, phase;
    float available;
    config.voltage_limit_v = config.vbus_v;
    config.open_voltage_slew_v_per_s = 100000.0f;
    config.open_velocity_slew_rad_s2 = 100000.0f;
    available = config.vbus_v * (config.duty_max - config.duty_min) / sqrtf(3.0f);
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetOpenLoop(&controller, 500.0f, 12.0f));
    for (i = 0u; i < 2000u; ++i) {
        float s, c, alpha, beta, actual_d, actual_q;
        assert(Foc_Step(&controller, &input, &out));
        for (phase = 0u; phase < 3u; ++phase) {
            assert(out.duty[phase] >= config.duty_min);
            assert(out.duty[phase] <= config.duty_max);
        }
        assert(out.electrical_angle_rad >= 0.0f && out.electrical_angle_rad < FOC_TWO_PI_F);
        assert(hypotf(out.vd_v, out.vq_v) <= available + 2e-6f);
        /* Remove common-mode by reconstructing alpha/beta from all three PWM phases. */
        alpha = config.vbus_v * (2.0f * out.duty[0] - out.duty[1] - out.duty[2]) / 3.0f;
        beta = config.vbus_v * (out.duty[1] - out.duty[2]) / sqrtf(3.0f);
        Foc_SinCos(out.electrical_angle_rad, &s, &c);
        actual_d = alpha * c + beta * s;
        actual_q = -alpha * s + beta * c;
        close_to(actual_d, out.vd_v, 1e-4f);
        close_to(actual_q, out.vq_v, 1e-4f);
    }
    assert(Foc_SetAlignment(&controller, FOC_PI_F / 2.0f, 1.0f));
    assert(Foc_Step(&controller, &input, &out));
    close_to(out.electrical_angle_rad, FOC_PI_F / 2.0f, 1e-6f);
    close_to(out.vd_v, 1.0f, 1e-6f);
    close_to(out.vq_v, 0.0f, 0.0f);
    assert(Foc_SetElectricalZero(&controller, 1.2f));
    assert(Foc_SetCurrent(&controller, 0.0f));
    input.sensor_valid = true;
    input.mechanical_angle_rad = 0.4f;
    assert(Foc_Step(&controller, &input, &out));
    close_to(out.electrical_angle_rad, 3.2f, 2e-6f);
}

static void test_current_closed_loop(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    float plant_id = 0.5f;
    float plant_iq = 0.0f;
    unsigned i;
    input.sensor_valid = true;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetCurrent(&controller, 1.0f));
    for (i = 0u; i < 10000u; ++i) {
        input.ia_a = plant_id;
        input.ic_a = -0.5f * plant_id - 0.86602540378f * plant_iq;
        assert(Foc_Step(&controller, &input, &out));
        if (i == 0u) {
            close_to(out.id_a, plant_id, 1e-6f);
            close_to(out.iq_a, plant_iq, 1e-6f);
            close_to(out.iq_reference_a, 0.02f, 1e-6f);
        }
        /* Stationary isotropic R/L plant: R = 0.6 ohm, L = 2 mH. */
        plant_id += config.fast_dt_s * (out.vd_v - 0.6f * plant_id) / 0.002f;
        plant_iq += config.fast_dt_s * (out.vq_v - 0.6f * plant_iq) / 0.002f;
    }
    close_to(plant_id, 0.0f, 1e-4f);
    close_to(plant_iq, 1.0f, 1e-4f);
    assert(Foc_SetCurrent(&controller, -1.0f));
    for (i = 0u; i < 10000u; ++i) {
        input.ia_a = plant_id;
        input.ic_a = -0.5f * plant_id - 0.86602540378f * plant_iq;
        assert(Foc_Step(&controller, &input, &out));
        plant_id += config.fast_dt_s * (out.vd_v - 0.6f * plant_id) / 0.002f;
        plant_iq += config.fast_dt_s * (out.vq_v - 0.6f * plant_iq) / 0.002f;
    }
    close_to(plant_iq, -1.0f, 1e-4f);
    printf("R/L current loop: Id %.6f A, Iq %.6f A\n", (double)plant_id, (double)plant_iq);
}

static void test_requested_current_above_trip_keeps_actual_phase_protection(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    config.fast_dt_s = 0.0005f;
    config.iq_limit_a = 3.3f;
    config.overcurrent_trip_a = 1.5f;
    config.iq_slew_a_per_s = 1.0f;
    input.sensor_valid = true;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetCurrent(&controller, 3.3f));
    assert(controller.target_iq_a == 3.3f);
    /* Synthetic zero-current input verifies the requested reference only;
     * it does not claim that the physical power stage delivers 3.3 A. */
    for (unsigned n = 0u; n < 6800u; ++n) {
        assert(Foc_Step(&controller, &input, &out));
        if (n == 0u) close_to(out.iq_reference_a, 0.0005f, 1e-8f);
    }
    close_to(out.iq_reference_a, 3.3f, 1e-6f);
    assert(controller.config.overcurrent_trip_a == 1.5f);
    input.ia_a = 1.5001f;
    assert(!Foc_Step(&controller, &input, &out));
    assert(out.fault & FOC_FAULT_OVERCURRENT);
    neutral(&out);
}

typedef struct {
    float id;
    float iq;
    float resistance;
    float inductance;
    float mechanical_angle;
    float electrical_angle;
} LockedRotorPlant;

static void locked_rotor_step(FocController *controller, LockedRotorPlant *plant,
                              FocOutput *out)
{
    FocInput in = {0};
    const float sine = sinf(plant->electrical_angle);
    const float cosine = cosf(plant->electrical_angle);
    const float alpha = plant->id * cosine - plant->iq * sine;
    const float beta = plant->id * sine + plant->iq * cosine;
    float applied_alpha, applied_beta, applied_d, applied_q, decay;
    in.ia_a = alpha;
    in.ic_a = -0.5f * alpha - 0.86602540378f * beta;
    in.mechanical_angle_rad = plant->mechanical_angle;
    in.sensor_valid = true;
    if (!Foc_Step(controller, &in, out)) {
        fprintf(stderr, "Locked-rotor fault 0x%x: Id %.6f Iq %.6f R %.6f\n",
                (unsigned)out->fault, (double)plant->id, (double)plant->iq,
                (double)plant->resistance);
        assert(0);
    }
    assert(out->pwm_enabled && out->fault == FOC_FAULT_NONE);
    /* Drive an independent plant from PWM, rather than trusting dq telemetry.
     * This also exercises the transforms at a nonzero electrical angle. */
    applied_alpha = controller->config.vbus_v *
        (2.0f * out->duty[0] - out->duty[1] - out->duty[2]) / 3.0f;
    applied_beta = controller->config.vbus_v *
        (out->duty[1] - out->duty[2]) / sqrtf(3.0f);
    applied_d = applied_alpha * cosine + applied_beta * sine;
    applied_q = -applied_alpha * sine + applied_beta * cosine;
    /* Exact zero-order-hold R/L response avoids a numerical Euler stability
     * limit when the saturation test raises resistance. No PI law is copied. */
    decay = expf(-plant->resistance * controller->config.fast_dt_s / plant->inductance);
    plant->id = plant->id * decay + applied_d * (1.0f - decay) / plant->resistance;
    plant->iq = plant->iq * decay + applied_q * (1.0f - decay) / plant->resistance;
}

static void check_board_current_plant(const FocConfig *config, const char *name)
{
    FocController controller;
    FocOutput out;
    LockedRotorPlant plant = {0};
    const float target = config->iq_limit_a;
    const float tolerance = fmaxf(0.0002f, target * 0.01f);
    const unsigned settling_steps = (unsigned)ceilf(
        (2.0f * target / config->iq_slew_a_per_s + 1.0f) / config->fast_dt_s);
    unsigned i, saturated_samples = 0u;
    float recovery_peak = 0.0f, release_factor;
    assert(config->fast_dt_s == 0.0005f); /* Actual board current-loop period. */
    assert(Foc_Init(&controller, config));
    assert(Foc_SetElectricalZero(&controller, 0.25f));
    plant.mechanical_angle = 0.37f;
    plant.electrical_angle = config->pole_pairs * plant.mechanical_angle - 0.25f;
    plant.resistance = 0.6f;
    plant.inductance = 0.002f;
    plant.id = 0.25f * target;
    assert(Foc_SetCurrent(&controller, target));
    for (i = 0u; i < settling_steps; ++i) locked_rotor_step(&controller, &plant, &out);
    close_to(plant.id, 0.0f, tolerance);
    close_to(plant.iq, target, tolerance);
    assert(Foc_SetCurrent(&controller, -target));
    plant.id += 0.25f * target; /* A second independent d-axis disturbance. */
    for (i = 0u; i < settling_steps; ++i) locked_rotor_step(&controller, &plant, &out);
    close_to(plant.id, 0.0f, tolerance);
    close_to(plant.iq, -target, tolerance);

    /* A high-resistance load makes the requested current physically
     * unattainable at the voltage cap, without faking current feedback. */
    assert(Foc_SetCurrent(&controller, target));
    plant.resistance = 2.5f * controller.effective_voltage_limit_v / target;
    for (i = 0u; i < 5000u; ++i) {
        locked_rotor_step(&controller, &plant, &out);
        assert(hypotf(out.vd_v, out.vq_v) <= controller.effective_voltage_limit_v + 2e-6f);
        if (hypotf(out.vd_v, out.vq_v) > 0.999f * controller.effective_voltage_limit_v)
            ++saturated_samples;
    }
    assert(saturated_samples > 100u);
    close_to(plant.iq, 0.4f * target, tolerance);
    /* Release the high-resistance load over 1 s, rather than imposing an
     * instantaneous winding-resistance jump. Keep the target and controller
     * running throughout: recovery cannot rely on STOP/integrator reset. */
    release_factor = powf(0.6f / plant.resistance, 1.0f / 2000.0f);
    for (i = 0u; i < 2000u; ++i) {
        plant.resistance *= release_factor;
        locked_rotor_step(&controller, &plant, &out);
        recovery_peak = fmaxf(recovery_peak, fabsf(plant.iq));
    }
    plant.resistance = 0.6f;
    for (i = 0u; i < 2000u; ++i) locked_rotor_step(&controller, &plant, &out);
    close_to(plant.id, 0.0f, tolerance);
    close_to(plant.iq, target, tolerance);
    assert(hypotf(out.vd_v, out.vq_v) < 0.9f * controller.effective_voltage_limit_v);
    printf("2 kHz %s R/L plant: +/-%.3f A, Id %.6f A, recovered Iq %.6f A, peak %.6f A\n",
           name, (double)target, (double)plant.id, (double)plant.iq, (double)recovery_peak);
}

static void test_board_rate_current_plants(void)
{
    FocConfig config = test_config();
    config.fast_dt_s = 0.0005f;
    config.speed_loop_divider = 2u;
    config.vbus_v = 14.0f;
    config.voltage_limit_v = 1.0f;
    config.iq_limit_a = 0.1f;
    config.overcurrent_trip_a = 0.5f;
    config.iq_slew_a_per_s = 1.0f;
    config.current_kp_v_per_a = 0.2f;
    config.current_ki_v_per_a_s = 20.0f;
    check_board_current_plant(&config, "CurrentLoopTest");

    /* Follow the actual configured board profile as production gains/limits
     * are tuned, in addition to retaining the dedicated test-image regression. */
    config.fast_dt_s = 1.0f / (float)MOTOR_CONTROL_HZ;
    config.speed_loop_divider = MOTOR_SPEED_LOOP_DIVIDER;
    config.pole_pairs = MOTOR_POLE_PAIRS;
    config.vbus_v = MOTOR_BUS_VOLTAGE_V;
    config.duty_min = MOTOR_PWM_DUTY_MIN;
    config.duty_max = MOTOR_PWM_DUTY_MAX;
    config.voltage_limit_v = MOTOR_VOLTAGE_LIMIT_V;
    /* The request ceiling may intentionally exceed the independent measured
     * phase-current trip. Exercise the production PI below that trip here;
     * protocol/range tests cover the full request ceiling separately. */
    config.iq_limit_a = fminf(MOTOR_CURRENT_LIMIT_A,
                              0.5f * MOTOR_OVERCURRENT_TRIP_A);
    config.overcurrent_trip_a = MOTOR_OVERCURRENT_TRIP_A;
    config.iq_slew_a_per_s = MOTOR_IQ_SLEW_A_PER_S;
    config.current_kp_v_per_a = MOTOR_CURRENT_KP;
    config.current_ki_v_per_a_s = MOTOR_CURRENT_KI;
    check_board_current_plant(&config, "motor_config");
}

typedef struct {
    float id, iq, mechanical_angle, peak_current;
} DrivenPmsmPlant;

static void driven_pmsm_step(FocController *controller, DrivenPmsmPlant *plant,
                             float speed, FocOutput *out)
{
    FocInput input = {0};
    const float poles = (float)controller->config.pole_pairs;
    const float angle = poles * plant->mechanical_angle;
    const float sine = sinf(angle), cosine = cosf(angle);
    const float alpha = plant->id * cosine - plant->iq * sine;
    const float beta = plant->id * sine + plant->iq * cosine;
    const float electrical_speed = poles * speed;
    const float dt = controller->config.fast_dt_s / 16.0f;
    float applied_alpha, applied_beta;
    unsigned substep;
    input.ia_a = alpha;
    input.ic_a = -0.5f * alpha - 0.86602540378f * beta;
    /* A finite sensor delay creates a speed-dependent d-axis back-EMF
     * component. Its magnitude is a test disturbance, not AS5600 calibration. */
    input.mechanical_angle_rad = Foc_WrapAngle(plant->mechanical_angle - speed * 0.0015f);
    input.mechanical_velocity_rad_s = speed;
    input.sensor_valid = true;
    assert(Foc_Step(controller, &input, out));
    assert(out->pwm_enabled && out->fault == FOC_FAULT_NONE);
    assert(hypotf(out->vd_v, out->vq_v) <= controller->effective_voltage_limit_v + 2e-6f);
    applied_alpha = controller->config.vbus_v *
        (2.0f * out->duty[0] - out->duty[1] - out->duty[2]) / 3.0f;
    applied_beta = controller->config.vbus_v *
        (out->duty[1] - out->duty[2]) / sqrtf(3.0f);
    /* Externally driven PMSM: R=.6 ohm, L=2 mH, flux=.01 Wb.
     * Hold the actual stationary-frame PWM voltage while the rotor moves.
     * Substeps keep the independent plant integration error small at 2 kHz. */
    for (substep = 0u; substep < 16u; ++substep) {
        const float physical_angle = poles * plant->mechanical_angle;
        const float s = sinf(physical_angle), c = cosf(physical_angle);
        const float vd = applied_alpha * c + applied_beta * s;
        const float vq = -applied_alpha * s + applied_beta * c;
        const float derivative_d = (vd - 0.6f * plant->id +
            electrical_speed * 0.002f * plant->iq) / 0.002f;
        const float derivative_q = (vq - 0.6f * plant->iq -
            electrical_speed * (0.002f * plant->id + 0.01f)) / 0.002f;
        plant->id += dt * derivative_d;
        plant->iq += dt * derivative_q;
        plant->mechanical_angle = Foc_WrapAngle(plant->mechanical_angle + speed * dt);
        plant->peak_current = fmaxf(plant->peak_current, hypotf(plant->id, plant->iq));
        assert(plant->peak_current < controller->config.overcurrent_trip_a);
    }
}

static void check_saturated_pmsm_id_priority(const FocConfig *profile, const char *name)
{
    FocConfig config = *profile;
    FocController controller;
    FocOutput out;
    DrivenPmsmPlant plant = {0};
    float mean_abs_id = 0.0f, mean_iq = 0.0f;
    const float target = config.iq_limit_a;
    const float high_speed = 0.99f * config.voltage_limit_v /
        ((float)config.pole_pairs * 0.01f);
    const float low_speed = high_speed * (5.0f / 18.0f);
    const float tolerance = fmaxf(0.002f, target * 0.01f);
    const unsigned settling_steps = (unsigned)ceilf(
        (2.0f * target / config.iq_slew_a_per_s + 1.0f) / config.fast_dt_s);
    unsigned i, saturated_samples = 0u;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetCurrent(&controller, target));
    for (i = 0u; i < settling_steps; ++i) driven_pmsm_step(&controller, &plant, 0.0f, &out);
    close_to(out.id_a, 0.0f, tolerance);
    close_to(out.iq_a, target, tolerance);
    for (i = 0u; i < 1200u; ++i)
        driven_pmsm_step(&controller, &plant, high_speed * (float)(i + 1u) / 1200.0f, &out);
    for (i = 0u; i < 2400u; ++i) {
        driven_pmsm_step(&controller, &plant, high_speed, &out);
        if (i >= 1600u) {
            mean_abs_id += fabsf(out.id_a) / 800.0f;
            mean_iq += out.iq_a / 800.0f;
            if (hypotf(out.vd_v, out.vq_v) > 0.9995f * config.voltage_limit_v)
                ++saturated_samples;
        }
    }
    printf("Saturated PMSM %s: mean |Id| %.6f A, Iq %.6f A, peak %.6f A\n",
           name, (double)mean_abs_id, (double)mean_iq, (double)plant.peak_current);
    /* Iq is physically unattainable at this speed. Nevertheless Id must keep
     * integrating toward zero; radial clipping + per-axis freeze fails here. */
    assert(saturated_samples > 700u);
    assert(mean_iq < 0.75f * target);
    assert(mean_abs_id < tolerance);
    for (i = 0u; i < 1200u; ++i)
        driven_pmsm_step(&controller, &plant,
            high_speed + (low_speed - high_speed) * (float)(i + 1u) / 1200.0f, &out);
    for (i = 0u; i < settling_steps; ++i) driven_pmsm_step(&controller, &plant, low_speed, &out);
    close_to(out.id_a, 0.0f, tolerance);
    close_to(out.iq_a, target, tolerance);
    /* Reverse torque without STOP or resetting either integrator. */
    assert(Foc_SetCurrent(&controller, -target));
    for (i = 0u; i < settling_steps; ++i) driven_pmsm_step(&controller, &plant, low_speed, &out);
    close_to(out.id_a, 0.0f, tolerance);
    close_to(out.iq_a, -target, tolerance);
}

static void test_saturated_pmsm_id_priority(void)
{
    static const float levels[] = {0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f};
    FocConfig config = test_config();
    FocController controller;
    FocOutput out;
    LockedRotorPlant plant = {0};
    unsigned i, level;
    config.fast_dt_s = 0.0005f;
    config.speed_loop_divider = 2u;
    config.vbus_v = 14.0f;
    config.voltage_limit_v = 2.0f;
    config.iq_limit_a = 0.2f;
    config.overcurrent_trip_a = 0.5f;
    config.iq_slew_a_per_s = 1.0f;
    config.current_kp_v_per_a = 0.2f;
    config.current_ki_v_per_a_s = 20.0f;
    check_saturated_pmsm_id_priority(&config, "regression");
    config.fast_dt_s = 1.0f / (float)MOTOR_CONTROL_HZ;
    config.pole_pairs = MOTOR_POLE_PAIRS;
    config.vbus_v = MOTOR_BUS_VOLTAGE_V;
    config.duty_min = MOTOR_PWM_DUTY_MIN;
    config.duty_max = MOTOR_PWM_DUTY_MAX;
    config.voltage_limit_v = MOTOR_VOLTAGE_LIMIT_V;
    config.iq_limit_a = fminf(MOTOR_CURRENT_LIMIT_A,
                              0.5f * MOTOR_OVERCURRENT_TRIP_A);
    config.overcurrent_trip_a = MOTOR_OVERCURRENT_TRIP_A;
    config.iq_slew_a_per_s = MOTOR_IQ_SLEW_A_PER_S;
    config.current_kp_v_per_a = MOTOR_CURRENT_KP;
    config.current_ki_v_per_a_s = MOTOR_CURRENT_KI;
    check_saturated_pmsm_id_priority(&config, "motor_config");
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetElectricalZero(&controller, 0.25f));
    plant.mechanical_angle = 0.37f;
    plant.electrical_angle = config.pole_pairs * plant.mechanical_angle - 0.25f;
    plant.resistance = 0.6f;
    plant.inductance = 0.002f;
    /* One uninterrupted current session tests both signs and crossings from
     * 10 mA through the configured upper limit, without integrator resets. */
    for (level = 0u; level < sizeof(levels) / sizeof(levels[0]); ++level) {
        const float target = levels[level];
        const unsigned settling_steps = (unsigned)ceilf(
            (2.0f * config.iq_limit_a / config.iq_slew_a_per_s + 1.0f) / config.fast_dt_s);
        if (target > config.iq_limit_a) continue;
        assert(Foc_SetCurrent(&controller, target));
        for (i = 0u; i < settling_steps; ++i) locked_rotor_step(&controller, &plant, &out);
        close_to(plant.id, 0.0f, 0.0002f);
        close_to(plant.iq, target, 0.0002f);
        assert(Foc_SetCurrent(&controller, -target));
        for (i = 0u; i < settling_steps; ++i) locked_rotor_step(&controller, &plant, &out);
        close_to(plant.id, 0.0f, 0.0002f);
        close_to(plant.iq, -target, 0.0002f);
    }
    printf("Configured R/L range: +/-10 mA through +/-%.3f A passed\n", (double)config.iq_limit_a);
}

static void test_pi_saturation_and_speed_divider(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    float first_integral;
    unsigned i;
    input.sensor_valid = true;
    config.current_kp_v_per_a = 100.0f;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetCurrent(&controller, 100.0f)); /* Finite targets clamp to Iq limit. */
    for (i = 0u; i < 20000u; ++i) assert(Foc_Step(&controller, &input, &out));
    close_to(out.iq_reference_a, 2.0f, 1e-6f);
    assert(fabsf(controller.integral_q_v) < 0.01f); /* No voltage wind-up. */
    assert(Foc_SetCurrent(&controller, 0.0f));
    for (i = 0u; i < 101u; ++i) assert(Foc_Step(&controller, &input, &out));
    assert(fabsf(out.vq_v) < 0.01f);

    config = test_config();
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetSpeed(&controller, 1.0f));
    assert(Foc_Step(&controller, &input, &out));
    first_integral = controller.integral_speed_a;
    close_to(first_integral, 0.002f, 1e-7f);
    for (i = 0u; i < 9u; ++i) {
        assert(Foc_Step(&controller, &input, &out));
        close_to(controller.integral_speed_a, first_integral, 0.0f);
    }
    assert(Foc_Step(&controller, &input, &out));
    close_to(controller.integral_speed_a, 0.004f, 1e-7f);

    assert(Foc_SetSpeed(&controller, 100.0f));
    first_integral = controller.integral_speed_a;
    for (i = 0u; i < 20000u; ++i) assert(Foc_Step(&controller, &input, &out));
    close_to(out.iq_reference_a, config.iq_limit_a, 1e-6f);
    close_to(controller.integral_speed_a, first_integral, 0.0f);
}

static void test_cascaded_speed_plant(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    float plant_id = 0.0f;
    float plant_iq = 0.0f;
    float speed = 0.0f;
    float mechanical_angle = 0.0f;
    unsigned i;
    input.sensor_valid = true;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetSpeed(&controller, 4.0f));
    for (i = 0u; i < 40000u; ++i) {
        float sine = sinf((float)config.pole_pairs * mechanical_angle);
        float cosine = cosf((float)config.pole_pairs * mechanical_angle);
        float alpha = plant_id * cosine - plant_iq * sine;
        float beta = plant_id * sine + plant_iq * cosine;
        float electrical_speed = (float)config.pole_pairs * speed;
        float derivative_d;
        float derivative_q;
        input.ia_a = alpha;
        input.ic_a = -0.5f * alpha - 0.86602540378f * beta;
        input.mechanical_angle_rad = mechanical_angle;
        input.mechanical_velocity_rad_s = speed;
        assert(Foc_Step(&controller, &input, &out));
        /* PMSM dq model: R=.6 ohm, L=2 mH, flux=.004 Wb,
         * J=.002 kg*m^2, viscous friction=.01 N*m/(rad/s). */
        derivative_d = (out.vd_v - 0.6f * plant_id +
                        electrical_speed * 0.002f * plant_iq) / 0.002f;
        derivative_q = (out.vq_v - 0.6f * plant_iq -
                        electrical_speed * (0.002f * plant_id + 0.004f)) / 0.002f;
        plant_id += config.fast_dt_s * derivative_d;
        plant_iq += config.fast_dt_s * derivative_q;
        speed += config.fast_dt_s *
            (1.5f * (float)config.pole_pairs * 0.004f * plant_iq - 0.01f * speed) / 0.002f;
        mechanical_angle = Foc_WrapAngle(mechanical_angle + speed * config.fast_dt_s);
    }
    close_to(speed, 4.0f, 0.002f);
    close_to(plant_id, 0.0f, 0.001f);
    close_to(plant_iq, 0.04f / (1.5f * 11.0f * 0.004f), 0.002f);
    printf("Cascaded speed/current plant: speed %.6f rad/s, Iq %.6f A\n",
           (double)speed, (double)plant_iq);
}

static void test_open_scalar_limit_and_current_telemetry(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    float available;
    config.voltage_limit_v = config.vbus_v;
    config.open_voltage_slew_v_per_s = 100000.0f;
    available = config.vbus_v * (config.duty_max - config.duty_min) / sqrtf(3.0f);
    input.ia_a = 1.0f; input.ic_a = -0.5f; /* At theta=0 this is Id=1, Iq=0. */
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetOpenLoop(&controller, 0.0f, -12.0f));
    assert(Foc_Step(&controller, &input, &out));
    assert(Foc_Step(&controller, &input, &out));
    close_to(out.vq_v, -available, 1e-6f);
    close_to(out.vd_v, 0.0f, 0.0f);
    close_to(out.id_a, 1.0f, 1e-6f);
    close_to(out.iq_a, 0.0f, 1e-6f);
    close_to(out.ib_a, -0.5f, 0.0f);
    close_to(controller.open_voltage_v, -12.0f, 0.0f);
    assert(Foc_SetAlignment(&controller, 0.0f, 12.0f));
    assert(Foc_Step(&controller, &input, &out));
    assert(Foc_Step(&controller, &input, &out));
    close_to(out.vd_v, available, 1e-6f);
    close_to(out.vq_v, 0.0f, 0.0f);
    close_to(out.id_a, 1.0f, 1e-6f);
    close_to(out.iq_a, 0.0f, 1e-6f);
}

static void test_binary32_finite_classification(void)
{
    static const uint32_t invalid_bits[] = {
        0x7f800000u, 0xff800000u, /* +/-infinity */
        0x7fc00000u, 0xffc00000u, /* signed quiet NaNs */
        0x7f800001u, 0xff800001u  /* signed signaling-NaN encodings */
    };
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    unsigned i;
    if (sizeof(float) != sizeof(uint32_t) || FLT_RADIX != 2 ||
        FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128) return;
    for (i = 0u; i < sizeof(invalid_bits) / sizeof(invalid_bits[0]); ++i) {
        float value;
        memcpy(&value, &invalid_bits[i], sizeof(value));
        assert(Foc_Init(&controller, &config));
        assert(!Foc_SetOpenLoop(&controller, value, 1.0f));
        assert(controller.fault & FOC_FAULT_COMMAND);
        assert(Foc_ClearFault(&controller));
        assert(Foc_SetOpenLoop(&controller, 0.0f, 1.0f));
        input.ia_a = value;
        assert(!Foc_Step(&controller, &input, &out));
        assert(out.fault & FOC_FAULT_INPUT);
        neutral(&out);
    }
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetOpenLoop(&controller, 0.0f, 1.0f));
    input.ia_a = -FLT_MIN; input.ic_a = 0.0f;
    assert(Foc_Step(&controller, &input, &out));
    assert(out.pwm_enabled && out.fault == FOC_FAULT_NONE);
    input.ia_a = FLT_MAX;
    assert(!Foc_Step(&controller, &input, &out));
    assert(out.fault & FOC_FAULT_OVERCURRENT); /* Finite maximum must not become NaN. */
    neutral(&out);
}

static void test_faults(void)
{
    FocController controller;
    FocConfig config = test_config();
    FocInput input = {0};
    FocOutput out;
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetCurrent(&controller, 1.0f));
    assert(!Foc_Step(&controller, &input, &out));
    assert(out.fault & FOC_FAULT_SENSOR);
    neutral(&out);
    Foc_Stop(&controller);
    assert(controller.mode == FOC_MODE_FAULT);
    assert(!Foc_SetOpenLoop(&controller, 1.0f, 1.0f));
    assert(Foc_ClearFault(&controller));
    assert(controller.mode == FOC_MODE_STOP);

    assert(Foc_SetOpenLoop(&controller, 1.0f, 1.0f));
    input.ia_a = 3.0f;
    input.ic_a = 3.0f; /* U/W each safe, reconstructed V trips at -6 A. */
    assert(!Foc_Step(&controller, &input, &out));
    assert(out.fault & FOC_FAULT_OVERCURRENT);
    neutral(&out);

    assert(Foc_ClearFault(&controller));
    assert(!Foc_SetOpenLoop(&controller, NAN, 1.0f));
    assert(controller.fault & FOC_FAULT_COMMAND);
    assert(Foc_ClearFault(&controller));
    assert(Foc_SetOpenLoop(&controller, 1.0f, 1.0f));
    input.ia_a = NAN;
    input.ic_a = 0.0f;
    assert(!Foc_Step(&controller, &input, &out));
    assert(out.fault & FOC_FAULT_INPUT);
    neutral(&out);

    assert(Foc_ClearFault(&controller));
    assert(Foc_SetCurrent(&controller, 1.0f));
    input.ia_a = 0.0f;
    input.sensor_valid = true;
    input.mechanical_angle_rad = INFINITY;
    assert(!Foc_Step(&controller, &input, &out));
    assert(out.fault & FOC_FAULT_SENSOR);

    config.pole_pairs = 0u;
    assert(!Foc_Init(&controller, &config));
    assert(controller.fault & FOC_FAULT_CONFIG);
    assert(!Foc_ClearFault(&controller));
    config = test_config();
    config.duty_max = config.duty_min;
    assert(!Foc_Init(&controller, &config));
    config = test_config();
    config.fast_dt_s = NAN;
    assert(!Foc_Init(&controller, &config));
    config = test_config();
    config.overcurrent_trip_a = config.iq_limit_a;
    assert(Foc_Init(&controller, &config)); /* Request and measured trip are independent. */
    config.overcurrent_trip_a = 0.0f;
    assert(!Foc_Init(&controller, &config));
    config = test_config();
    config.current_ki_v_per_a_s = INFINITY;
    assert(!Foc_Init(&controller, &config));
    assert(!Foc_Init(NULL, &config));
    assert(!Foc_Init(&controller, NULL));
    assert(!Foc_Step(NULL, NULL, &out));
    assert(!out.pwm_enabled);

    config = test_config();
    assert(Foc_Init(&controller, &config));
    assert(Foc_SetCurrent(&controller, 0.0f));
    assert(!Foc_SetElectricalZero(&controller, 0.2f)); /* Prevent running phase jumps. */
    assert(controller.fault & FOC_FAULT_COMMAND);
}

int main(void)
{
    test_math();
    test_stop_and_zero_voltage();
    test_open_loop_slew();
    test_pwm_and_alignment();
    test_current_closed_loop();
    test_requested_current_above_trip_keeps_actual_phase_protection();
    test_board_rate_current_plants();
    test_saturated_pmsm_id_priority();
    test_pi_saturation_and_speed_divider();
    test_cascaded_speed_plant();
    test_open_scalar_limit_and_current_telemetry();
    test_binary32_finite_classification();
    test_faults();
    puts("All FOC host tests passed.");
    return 0;
}

