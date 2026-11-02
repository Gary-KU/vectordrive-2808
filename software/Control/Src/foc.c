#include "foc.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define INV_TWO_PI_F 0.15915494309189533577f
#define TWO_OVER_PI_F 0.63661977236758134308f
#define INV_SQRT3_F 0.57735026918962576451f
#define SQRT3_OVER_2_F 0.86602540378443864676f

/* ARMCC uses the ARM ABI's 32-bit float; GCC/Clang advertise its storage size.
 * Other representations keep the portable arithmetic implementation. */
#if FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128 && \
    (defined(__CC_ARM) || (defined(__SIZEOF_FLOAT__) && __SIZEOF_FLOAT__ == 4))
#define FOC_BINARY32_STORAGE 1
typedef char FocFloatStorageMustBe32[(sizeof(float) == sizeof(uint32_t)) ? 1 : -1];
#else
#define FOC_BINARY32_STORAGE 0
#endif

/* 1 KiB flash. Interpolation avoids libm sin/cos in the control interrupt. */
static const float sine_quarter[257] = {
    0.0000000000f, 0.0061358846f, 0.0122715383f, 0.0184067299f, 0.0245412285f, 0.0306748032f, 0.0368072229f, 0.0429382569f,
    0.0490676743f, 0.0551952443f, 0.0613207363f, 0.0674439196f, 0.0735645636f, 0.0796824380f, 0.0857973123f, 0.0919089565f,
    0.0980171403f, 0.1041216339f, 0.1102222073f, 0.1163186309f, 0.1224106752f, 0.1284981108f, 0.1345807085f, 0.1406582393f,
    0.1467304745f, 0.1527971853f, 0.1588581433f, 0.1649131205f, 0.1709618888f, 0.1770042204f, 0.1830398880f, 0.1890686641f,
    0.1950903220f, 0.2011046348f, 0.2071113762f, 0.2131103199f, 0.2191012402f, 0.2250839114f, 0.2310581083f, 0.2370236060f,
    0.2429801799f, 0.2489276057f, 0.2548656596f, 0.2607941179f, 0.2667127575f, 0.2726213554f, 0.2785196894f, 0.2844075372f,
    0.2902846773f, 0.2961508882f, 0.3020059493f, 0.3078496400f, 0.3136817404f, 0.3195020308f, 0.3253102922f, 0.3311063058f,
    0.3368898534f, 0.3426607173f, 0.3484186802f, 0.3541635254f, 0.3598950365f, 0.3656129978f, 0.3713171940f, 0.3770074102f,
    0.3826834324f, 0.3883450467f, 0.3939920401f, 0.3996241998f, 0.4052413140f, 0.4108431711f, 0.4164295601f, 0.4220002708f,
    0.4275550934f, 0.4330938189f, 0.4386162385f, 0.4441221446f, 0.4496113297f, 0.4550835871f, 0.4605387110f, 0.4659764958f,
    0.4713967368f, 0.4767992301f, 0.4821837721f, 0.4875501601f, 0.4928981922f, 0.4982276670f, 0.5035383837f, 0.5088301425f,
    0.5141027442f, 0.5193559902f, 0.5245896827f, 0.5298036247f, 0.5349976199f, 0.5401714727f, 0.5453249884f, 0.5504579729f,
    0.5555702330f, 0.5606615762f, 0.5657318108f, 0.5707807459f, 0.5758081914f, 0.5808139581f, 0.5857978575f, 0.5907597019f,
    0.5956993045f, 0.6006164794f, 0.6055110414f, 0.6103828063f, 0.6152315906f, 0.6200572118f, 0.6248594881f, 0.6296382389f,
    0.6343932842f, 0.6391244449f, 0.6438315429f, 0.6485144010f, 0.6531728430f, 0.6578066933f, 0.6624157776f, 0.6669999223f,
    0.6715589548f, 0.6760927036f, 0.6806009978f, 0.6850836678f, 0.6895405447f, 0.6939714609f, 0.6983762494f, 0.7027547445f,
    0.7071067812f, 0.7114321957f, 0.7157308253f, 0.7200025080f, 0.7242470830f, 0.7284643904f, 0.7326542717f, 0.7368165689f,
    0.7409511254f, 0.7450577854f, 0.7491363945f, 0.7531867990f, 0.7572088465f, 0.7612023855f, 0.7651672656f, 0.7691033376f,
    0.7730104534f, 0.7768884657f, 0.7807372286f, 0.7845565972f, 0.7883464276f, 0.7921065773f, 0.7958369046f, 0.7995372691f,
    0.8032075315f, 0.8068475535f, 0.8104571983f, 0.8140363297f, 0.8175848132f, 0.8211025150f, 0.8245893028f, 0.8280450453f,
    0.8314696123f, 0.8348628750f, 0.8382247056f, 0.8415549774f, 0.8448535652f, 0.8481203448f, 0.8513551931f, 0.8545579884f,
    0.8577286100f, 0.8608669386f, 0.8639728561f, 0.8670462455f, 0.8700869911f, 0.8730949784f, 0.8760700942f, 0.8790122264f,
    0.8819212643f, 0.8847970984f, 0.8876396204f, 0.8904487232f, 0.8932243012f, 0.8959662498f, 0.8986744657f, 0.9013488470f,
    0.9039892931f, 0.9065957045f, 0.9091679831f, 0.9117060320f, 0.9142097557f, 0.9166790599f, 0.9191138517f, 0.9215140393f,
    0.9238795325f, 0.9262102421f, 0.9285060805f, 0.9307669611f, 0.9329927988f, 0.9351835099f, 0.9373390119f, 0.9394592236f,
    0.9415440652f, 0.9435934582f, 0.9456073254f, 0.9475855910f, 0.9495281806f, 0.9514350210f, 0.9533060404f, 0.9551411683f,
    0.9569403357f, 0.9587034749f, 0.9604305194f, 0.9621214043f, 0.9637760658f, 0.9653944417f, 0.9669764710f, 0.9685220943f,
    0.9700312532f, 0.9715038910f, 0.9729399522f, 0.9743393828f, 0.9757021300f, 0.9770281427f, 0.9783173707f, 0.9795697657f,
    0.9807852804f, 0.9819638691f, 0.9831054874f, 0.9842100924f, 0.9852776424f, 0.9863080972f, 0.9873014182f, 0.9882575677f,
    0.9891765100f, 0.9900582103f, 0.9909026354f, 0.9917097537f, 0.9924795346f, 0.9932119492f, 0.9939069700f, 0.9945645707f,
    0.9951847267f, 0.9957674145f, 0.9963126122f, 0.9968202993f, 0.9972904567f, 0.9977230666f, 0.9981181129f, 0.9984755806f,
    0.9987954562f, 0.9990777278f, 0.9993223846f, 0.9995294175f, 0.9996988187f, 0.9998305818f, 0.9999247018f, 0.9999811753f,
    1.0000000000f
};

static bool finite_f(float value)
{
#if FOC_BINARY32_STORAGE
    uint32_t bits;
    /* memcpy is alias-safe; ARMCC/GCC reduce this to register operations. */
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
#else
    return value <= FLT_MAX && value >= -FLT_MAX;
#endif
}

static float abs_f(float value)
{
#if FOC_BINARY32_STORAGE
    uint32_t bits;
    uint32_t magnitude;
    memcpy(&bits, &value, sizeof(bits));
    magnitude = bits & 0x7fffffffu;
    /* Preserve the original helper's NaN and signed-zero behavior. */
    if (magnitude == 0u || magnitude > 0x7f800000u) return value;
    memcpy(&value, &magnitude, sizeof(value));
    return value;
#else
    return value < 0.0f ? -value : value;
#endif
}

static float clamp_f(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static float slew_f(float value, float target, float delta)
{
    return value + clamp_f(target - value, -delta, delta);
}

float Foc_WrapAngle(float angle_rad)
{
    float wrapped;
    int32_t turns;
#if FLT_RADIX == 2 && FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128
    if (sizeof(float) == sizeof(uint32_t)) {
        uint32_t bits;
        uint32_t two_pi_bits;
        const float two_pi = FOC_TWO_PI_F;
        memcpy(&bits, &angle_rad, sizeof(bits));
        memcpy(&two_pi_bits, &two_pi, sizeof(two_pi_bits));
        /* Positive binary32 bit order is numeric order; no soft-float compares
         * are needed for the wrapped electrical angle in every fast step. */
        if (bits < two_pi_bits || (bits & 0x7fffffffu) == 0u) return angle_rad;
        if ((bits & 0x80000000u) != 0u &&
            (bits & 0x7fffffffu) <= two_pi_bits) {
            wrapped = angle_rad + FOC_TWO_PI_F;
            memcpy(&bits, &wrapped, sizeof(bits));
            return bits < two_pi_bits ? wrapped : 0.0f;
        }
    }
#endif
    /* Encoder and integrated electrical angles almost always take these paths.
     * Ordered comparisons also keep NaN/Inf out of the fast return. */
    if (angle_rad >= 0.0f && angle_rad < FOC_TWO_PI_F) {
        return angle_rad;
    }
    if (angle_rad < 0.0f && angle_rad >= -FOC_TWO_PI_F) {
        wrapped = angle_rad + FOC_TWO_PI_F;
        /* Tiny negative inputs can round upward to exactly 2*pi. */
        return wrapped < FOC_TWO_PI_F ? wrapped : 0.0f;
    }
    if (!finite_f(angle_rad) || abs_f(angle_rad) > 1000000.0f) {
        return 0.0f;
    }
    turns = (int32_t)(angle_rad * INV_TWO_PI_F);
    wrapped = angle_rad - (float)turns * FOC_TWO_PI_F;
    if (wrapped < 0.0f) {
        wrapped += FOC_TWO_PI_F;
    }
    if (wrapped >= FOC_TWO_PI_F) {
        wrapped -= FOC_TWO_PI_F;
    }
    return wrapped;
}

void Foc_SinCos(float angle_rad, float *sine, float *cosine)
{
    /* Scale directly to the full 1024-segment cycle. Scaling by 256 is exact
     * in binary, so this preserves the former quarter-wave interpolation but
     * needs one fewer float/integer conversion and one fewer subtraction. */
    float position = Foc_WrapAngle(angle_rad) * (TWO_OVER_PI_F * 256.0f);
    uint32_t full_index = (uint32_t)position;
    uint32_t quadrant;
    uint32_t index;
    float fraction;
    float s;
    float c;
    if (full_index >= 1024u) {
        full_index = 0u;
        position = 0.0f;
    }
    quadrant = full_index >> 8;
    index = full_index & 255u;
    fraction = position - (float)full_index;
    s = sine_quarter[index] +
        fraction * (sine_quarter[index + 1u] - sine_quarter[index]);
    c = sine_quarter[256u - index] +
        fraction * (sine_quarter[255u - index] - sine_quarter[256u - index]);
    switch (quadrant) {
    case 0u: break;
    case 1u: { float old_s = s; s = c; c = -old_s; break; }
    case 2u: s = -s; c = -c; break;
    default: { float old_s = s; s = -c; c = old_s; break; }
    }
    if (sine != NULL) {
        *sine = s;
    }
    if (cosine != NULL) {
        *cosine = c;
    }
}

static bool positive_bounded(float value)
{
    return finite_f(value) && value > 0.0f && value <= 1000000.0f;
}

static bool nonnegative_bounded(float value)
{
    return finite_f(value) && value >= 0.0f && value <= 1000000.0f;
}

static bool config_valid(const FocConfig *config)
{
    if (config == NULL) {
        return false;
    }
    return positive_bounded(config->fast_dt_s) && config->fast_dt_s <= 0.01f &&
           config->speed_loop_divider >= 1u && config->speed_loop_divider <= 1000u &&
           config->pole_pairs >= 1u && config->pole_pairs <= 128u &&
           positive_bounded(config->vbus_v) && config->vbus_v <= 1000.0f &&
           finite_f(config->duty_min) && finite_f(config->duty_max) &&
           config->duty_min >= 0.0f && config->duty_max <= 1.0f &&
           config->duty_max - config->duty_min >= 0.01f &&
           positive_bounded(config->voltage_limit_v) &&
           config->voltage_limit_v <= config->vbus_v &&
           positive_bounded(config->iq_limit_a) &&
           positive_bounded(config->overcurrent_trip_a) &&
           /* Request and measured-phase protection are independent. A large
            * request may voltage-saturate; crossing the trip still faults. */
           positive_bounded(config->iq_slew_a_per_s) &&
           positive_bounded(config->open_voltage_slew_v_per_s) &&
           positive_bounded(config->open_velocity_slew_rad_s2) &&
           positive_bounded(config->open_velocity_limit_rad_s) &&
           config->open_velocity_limit_rad_s * config->fast_dt_s <= FOC_PI_F &&
           positive_bounded(config->speed_limit_rad_s) &&
           nonnegative_bounded(config->current_kp_v_per_a) &&
           nonnegative_bounded(config->current_ki_v_per_a_s) &&
           nonnegative_bounded(config->speed_kp_a_per_rad_s) &&
           nonnegative_bounded(config->speed_ki_a_per_rad);
}

static void reset_motion(FocController *controller)
{
    controller->target_open_velocity_rad_s = 0.0f;
    controller->target_open_voltage_v = 0.0f;
    controller->open_velocity_rad_s = 0.0f;
    controller->open_voltage_v = 0.0f;
    controller->target_iq_a = 0.0f;
    controller->target_speed_rad_s = 0.0f;
    controller->iq_reference_a = 0.0f;
    controller->speed_iq_a = 0.0f;
    controller->integral_d_v = 0.0f;
    controller->integral_q_v = 0.0f;
    controller->integral_speed_a = 0.0f;
    controller->speed_counter = 0u;
}

static bool latch_fault(FocController *controller, uint32_t fault)
{
    if (controller != NULL) {
        reset_motion(controller);
        controller->fault |= fault;
        controller->mode = FOC_MODE_FAULT;
    }
    return false;
}

static bool ready(FocController *controller)
{
    if (controller == NULL) {
        return false;
    }
    if (!controller->initialized) {
        return latch_fault(controller, FOC_FAULT_CONFIG);
    }
    return controller->fault == FOC_FAULT_NONE;
}

static void enter_mode(FocController *controller, FocMode mode)
{
    if (controller->mode != mode) {
        reset_motion(controller);
        controller->mode = mode;
    }
}

static bool cache_constants(FocController *controller)
{
    const FocConfig *config = &controller->config;
    float available = config->vbus_v * (config->duty_max - config->duty_min) * INV_SQRT3_F;
    controller->inverse_vbus_v = 1.0f / config->vbus_v;
    controller->effective_voltage_limit_v =
        config->voltage_limit_v < available ? config->voltage_limit_v : available;
    controller->neutral_duty = 0.5f * (config->duty_min + config->duty_max);
    return finite_f(controller->inverse_vbus_v) && controller->inverse_vbus_v > 0.0f &&
           finite_f(controller->effective_voltage_limit_v) &&
           controller->effective_voltage_limit_v > 0.0f;
}

bool Foc_Init(FocController *controller, const FocConfig *config)
{
    if (controller == NULL) {
        return false;
    }
    memset(controller, 0, sizeof(*controller));
    if (config != NULL) {
        controller->config = *config;
    }
    if (!config_valid(config)) {
        return latch_fault(controller, FOC_FAULT_CONFIG);
    }
    if (!cache_constants(controller)) {
        return latch_fault(controller, FOC_FAULT_CONFIG);
    }
    controller->initialized = true;
    controller->mode = FOC_MODE_STOP;
    return true;
}

void Foc_Stop(FocController *controller)
{
    if (controller == NULL) {
        return;
    }
    reset_motion(controller);
    controller->mode = controller->fault != FOC_FAULT_NONE ? FOC_MODE_FAULT : FOC_MODE_STOP;
}

bool Foc_ClearFault(FocController *controller)
{
    if (controller == NULL) {
        return false;
    }
    if (!controller->initialized || !config_valid(&controller->config)) {
        return latch_fault(controller, FOC_FAULT_CONFIG);
    }
    if (!cache_constants(controller)) {
        return latch_fault(controller, FOC_FAULT_CONFIG);
    }
    controller->fault = FOC_FAULT_NONE;
    controller->mode = FOC_MODE_STOP;
    reset_motion(controller);
    return true;
}

bool Foc_SetElectricalZero(FocController *controller, float electrical_zero_rad)
{
    if (!ready(controller)) {
        return false;
    }
    if (!finite_f(electrical_zero_rad) || abs_f(electrical_zero_rad) > 1000000.0f ||
        controller->mode == FOC_MODE_CURRENT || controller->mode == FOC_MODE_SPEED) {
        return latch_fault(controller, FOC_FAULT_COMMAND);
    }
    controller->electrical_zero_rad = Foc_WrapAngle(electrical_zero_rad);
    return true;
}

bool Foc_SetOpenLoop(FocController *controller, float electrical_velocity_rad_s,
                     float voltage_v)
{
    if (!ready(controller)) {
        return false;
    }
    if (!finite_f(electrical_velocity_rad_s) || !finite_f(voltage_v)) {
        return latch_fault(controller, FOC_FAULT_COMMAND);
    }
    enter_mode(controller, FOC_MODE_OPEN_LOOP);
    controller->target_open_velocity_rad_s = clamp_f(electrical_velocity_rad_s,
        -controller->config.open_velocity_limit_rad_s,
         controller->config.open_velocity_limit_rad_s);
    controller->target_open_voltage_v = clamp_f(voltage_v,
        -controller->config.voltage_limit_v, controller->config.voltage_limit_v);
    return true;
}

bool Foc_SetAlignment(FocController *controller, float electrical_angle_rad,
                      float voltage_v)
{
    if (!ready(controller)) {
        return false;
    }
    if (!finite_f(electrical_angle_rad) || abs_f(electrical_angle_rad) > 1000000.0f ||
        !finite_f(voltage_v) || voltage_v < 0.0f) {
        return latch_fault(controller, FOC_FAULT_COMMAND);
    }
    enter_mode(controller, FOC_MODE_ALIGNMENT);
    controller->electrical_angle_rad = Foc_WrapAngle(electrical_angle_rad);
    controller->target_open_voltage_v = clamp_f(voltage_v, 0.0f,
                                                controller->config.voltage_limit_v);
    return true;
}

bool Foc_SetCurrent(FocController *controller, float iq_a)
{
    if (!ready(controller)) {
        return false;
    }
    if (!finite_f(iq_a)) {
        return latch_fault(controller, FOC_FAULT_COMMAND);
    }
    enter_mode(controller, FOC_MODE_CURRENT);
    controller->target_iq_a = clamp_f(iq_a, -controller->config.iq_limit_a,
                                     controller->config.iq_limit_a);
    return true;
}

bool Foc_SetSpeed(FocController *controller, float mechanical_velocity_rad_s)
{
    if (!ready(controller)) {
        return false;
    }
    if (!finite_f(mechanical_velocity_rad_s)) {
        return latch_fault(controller, FOC_FAULT_COMMAND);
    }
    enter_mode(controller, FOC_MODE_SPEED);
    controller->target_speed_rad_s = clamp_f(mechanical_velocity_rad_s,
        -controller->config.speed_limit_rad_s, controller->config.speed_limit_rad_s);
    return true;
}

static void safe_output(const FocController *controller, FocOutput *output)
{
    float neutral = 0.5f;
    memset(output, 0, sizeof(*output));
    output->mode = FOC_MODE_FAULT;
    output->fault = FOC_FAULT_CONFIG;
    if (controller == NULL) {
        return;
    }
    if (controller->initialized) {
        neutral = controller->neutral_duty;
    }
    output->duty[0] = neutral;
    output->duty[1] = neutral;
    output->duty[2] = neutral;
    output->mode = controller->mode;
    output->fault = controller->fault;
    output->electrical_angle_rad = controller->electrical_angle_rad;
}

static void limit_current_voltage(float *vd, float *vq, float limit)
{
    float remaining_squared;
    /* Give Id regulation the requested d-axis voltage before allocating the
     * remaining voltage circle to Iq. Radial scaling also clips vd whenever
     * q saturates; the per-axis conditional integrator then freezes Id's
     * correction even when rotating the saturated vector could remove it. */
    *vd = clamp_f(*vd, -limit, limit);
    remaining_squared = limit * limit - *vd * *vd;
    if (*vq * *vq > remaining_squared) {
        float q_limit = sqrtf(remaining_squared);
        *vq = *vq < 0.0f ? -q_limit : q_limit;
    }
}

static float update_integral(float integral, float error, float raw,
                             float applied, float ki_dt, float limit)
{
    /* Conditional integration: freeze only when it would deepen saturation. */
    if ((raw - applied) * error <= 0.0f) {
        integral += ki_dt * error;
    }
    return clamp_f(integral, -limit, limit);
}

static void svpwm(const FocController *controller, float vd, float vq,
                   float sine, float cosine, float duty[3])
{
    const FocConfig *config = &controller->config;
    float alpha = vd * cosine - vq * sine;
    float beta = vd * sine + vq * cosine;
    float phase[3];
    float highest;
    float lowest;
    float common;
    float center = controller->neutral_duty;
    unsigned i;
    phase[0] = alpha;
    phase[1] = -0.5f * alpha + SQRT3_OVER_2_F * beta;
    phase[2] = -0.5f * alpha - SQRT3_OVER_2_F * beta;
    highest = phase[0];
    lowest = phase[0];
    for (i = 1u; i < 3u; ++i) {
        if (phase[i] > highest) highest = phase[i];
        if (phase[i] < lowest) lowest = phase[i];
    }
    common = -0.5f * (highest + lowest);
    for (i = 0u; i < 3u; ++i) {
        duty[i] = clamp_f(center + (phase[i] + common) * controller->inverse_vbus_v,
                          config->duty_min, config->duty_max);
    }
}

bool Foc_Step(FocController *controller, const FocInput *input, FocOutput *output)
{
    const FocConfig *config;
    float ib;
    float alpha;
    float beta;
    float sine;
    float cosine;
    float id;
    float iq;
    float vd = 0.0f;
    float vq = 0.0f;
    float voltage_limit;
    if (output == NULL) {
        return latch_fault(controller, FOC_FAULT_INPUT);
    }
    safe_output(controller, output);
    if (!ready(controller)) {
        safe_output(controller, output);
        return false;
    }
    if (controller->mode == FOC_MODE_STOP) {
        return true;
    }
    config = &controller->config;
    if (input == NULL || !finite_f(input->ia_a) || !finite_f(input->ic_a)) {
        latch_fault(controller, FOC_FAULT_INPUT);
        safe_output(controller, output);
        return false;
    }
    if (abs_f(input->ia_a) > config->overcurrent_trip_a ||
        abs_f(input->ic_a) > config->overcurrent_trip_a) {
        latch_fault(controller, FOC_FAULT_OVERCURRENT);
        safe_output(controller, output);
        return false;
    }
    ib = -input->ia_a - input->ic_a;
    if (abs_f(ib) > config->overcurrent_trip_a) {
        latch_fault(controller, FOC_FAULT_OVERCURRENT);
        safe_output(controller, output);
        return false;
    }
    if ((controller->mode == FOC_MODE_CURRENT || controller->mode == FOC_MODE_SPEED) &&
        !input->sensor_valid) {
        latch_fault(controller, FOC_FAULT_SENSOR);
        safe_output(controller, output);
        return false;
    }
    if (input->sensor_valid && (!finite_f(input->mechanical_angle_rad) ||
        abs_f(input->mechanical_angle_rad) > 1000000.0f ||
        !finite_f(input->mechanical_velocity_rad_s) ||
        abs_f(input->mechanical_velocity_rad_s) > 1000000.0f)) {
        latch_fault(controller, FOC_FAULT_SENSOR);
        safe_output(controller, output);
        return false;
    }
    voltage_limit = controller->effective_voltage_limit_v;
    if (controller->mode == FOC_MODE_OPEN_LOOP) {
        controller->open_velocity_rad_s = slew_f(controller->open_velocity_rad_s,
            controller->target_open_velocity_rad_s,
            config->open_velocity_slew_rad_s2 * config->fast_dt_s);
        controller->electrical_angle_rad = Foc_WrapAngle(controller->electrical_angle_rad +
            controller->open_velocity_rad_s * config->fast_dt_s);
        controller->open_voltage_v = slew_f(controller->open_voltage_v,
            controller->target_open_voltage_v, config->open_voltage_slew_v_per_s * config->fast_dt_s);
        vq = controller->open_voltage_v;
    } else if (controller->mode == FOC_MODE_ALIGNMENT) {
        controller->open_voltage_v = slew_f(controller->open_voltage_v,
            controller->target_open_voltage_v, config->open_voltage_slew_v_per_s * config->fast_dt_s);
        vd = controller->open_voltage_v;
    } else if (controller->mode == FOC_MODE_CURRENT || controller->mode == FOC_MODE_SPEED) {
        /* Wrap first: avoids losing precision when mechanical_angle is multi-turn. */
        controller->electrical_angle_rad = Foc_WrapAngle(
            Foc_WrapAngle(input->mechanical_angle_rad) * (float)config->pole_pairs -
            controller->electrical_zero_rad);
    } else {
        latch_fault(controller, FOC_FAULT_CONFIG);
        safe_output(controller, output);
        return false;
    }
    Foc_SinCos(controller->electrical_angle_rad, &sine, &cosine);
    alpha = input->ia_a;
    beta = (input->ia_a + 2.0f * ib) * INV_SQRT3_F;
    id = alpha * cosine + beta * sine;
    iq = -alpha * sine + beta * cosine;
    if (controller->mode == FOC_MODE_CURRENT || controller->mode == FOC_MODE_SPEED) {
        float error_d;
        float error_q;
        float raw_d;
        float raw_q;
        if (controller->mode == FOC_MODE_SPEED) {
            if (controller->speed_counter == 0u) {
                float speed_error = controller->target_speed_rad_s - input->mechanical_velocity_rad_s;
                float raw_iq = config->speed_kp_a_per_rad_s * speed_error +
                               controller->integral_speed_a;
                controller->speed_iq_a = clamp_f(raw_iq, -config->iq_limit_a, config->iq_limit_a);
                controller->integral_speed_a = update_integral(controller->integral_speed_a,
                    speed_error, raw_iq, controller->speed_iq_a,
                    config->speed_ki_a_per_rad * config->fast_dt_s * (float)config->speed_loop_divider,
                    config->iq_limit_a);
            }
            ++controller->speed_counter;
            if (controller->speed_counter >= config->speed_loop_divider) {
                controller->speed_counter = 0u;
            }
            controller->target_iq_a = controller->speed_iq_a;
        }
        controller->iq_reference_a = slew_f(controller->iq_reference_a,
            controller->target_iq_a, config->iq_slew_a_per_s * config->fast_dt_s);
        error_d = -id;
        error_q = controller->iq_reference_a - iq;
        raw_d = config->current_kp_v_per_a * error_d + controller->integral_d_v;
        raw_q = config->current_kp_v_per_a * error_q + controller->integral_q_v;
        vd = raw_d;
        vq = raw_q;
        limit_current_voltage(&vd, &vq, voltage_limit);
        controller->integral_d_v = update_integral(controller->integral_d_v,
            error_d, raw_d, vd, config->current_ki_v_per_a_s * config->fast_dt_s, voltage_limit);
        controller->integral_q_v = update_integral(controller->integral_q_v,
            error_q, raw_q, vq, config->current_ki_v_per_a_s * config->fast_dt_s, voltage_limit);
    } else if (controller->mode == FOC_MODE_OPEN_LOOP) {
        /* One-axis vectors have an exact scalar limit; squaring/sqrt/dividing
         * adds cost and rounding without changing the permitted voltage. */
        vq = clamp_f(vq, -voltage_limit, voltage_limit);
    } else {
        vd = clamp_f(vd, -voltage_limit, voltage_limit);
    }
    if (!finite_f(vd) || !finite_f(vq) || !finite_f(controller->integral_d_v) ||
        !finite_f(controller->integral_q_v) || !finite_f(controller->integral_speed_a)) {
        latch_fault(controller, FOC_FAULT_NUMERIC);
        safe_output(controller, output);
        return false;
    }
    svpwm(controller, vd, vq, sine, cosine, output->duty);
    output->pwm_enabled = true;
    output->mode = controller->mode;
    output->fault = controller->fault;
    output->electrical_angle_rad = controller->electrical_angle_rad;
    output->ia_a = input->ia_a;
    output->ib_a = ib;
    output->ic_a = input->ic_a;
    output->id_a = id;
    output->iq_a = iq;
    output->iq_reference_a = controller->iq_reference_a;
    output->vd_v = vd;
    output->vq_v = vq;
    output->open_velocity_rad_s = controller->open_velocity_rad_s;
    return true;
}
