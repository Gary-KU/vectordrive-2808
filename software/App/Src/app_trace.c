#include "app_trace.h"

#if MOTOR_CONTROL_TRACE
#include <stddef.h>

/* Fixed debugger/export ABI, also checked by ARMCC builds without C11 asserts. */
typedef char AppTraceFloatMustBe32Bits[(sizeof(float) == 4u) ? 1 : -1];
typedef char AppTraceRecordMustBe52Bytes[(sizeof(AppTraceRecord) == 52u) ? 1 : -1];
typedef char AppTraceModeMustBeAt48[(offsetof(AppTraceRecord, mode) == 48u) ? 1 : -1];

volatile uint32_t app_trace_armed = 0u;
volatile uint32_t app_trace_count = 0u;
volatile uint32_t app_trace_divider = 0u;
volatile AppTraceRecord app_trace_buffer[APP_TRACE_CAPACITY];

void AppTrace_Record(uint32_t sample_count, uint16_t raw_u, uint16_t raw_w,
                     const FocOutput *out, float mechanical_angle,
                     float mechanical_velocity)
{
    uint32_t index;
    volatile AppTraceRecord *row;
    if (app_trace_armed == 0u || out == NULL || !out->pwm_enabled) return;
    index = app_trace_count;
    if (index >= APP_TRACE_CAPACITY) {
        app_trace_armed = 0u;
        return;
    }
#if APP_TRACE_DECIMATION > 1u
    if (app_trace_divider < APP_TRACE_DECIMATION - 1u) {
        ++app_trace_divider;
        return;
    }
#endif
    app_trace_divider = 0u;
    row = &app_trace_buffer[index];
    row->sample_count = sample_count;
    row->raw_u = raw_u;
    row->raw_w = raw_w;
    row->ia_a = out->ia_a;
    row->ic_a = out->ic_a;
    row->id_a = out->id_a;
    row->iq_a = out->iq_a;
    row->iq_reference_a = out->iq_reference_a;
    row->vd_v = out->vd_v;
    row->vq_v = out->vq_v;
    row->mechanical_angle_rad = mechanical_angle;
    row->mechanical_velocity_rad_s = mechanical_velocity;
    row->electrical_angle_rad = out->electrical_angle_rad;
    row->mode = (uint32_t)out->mode;
    if (index + 1u == APP_TRACE_CAPACITY) app_trace_armed = 0u;
    /* Publish count last, after every row/control store. count==CAPACITY is
     * the authoritative indication that the complete capture is available. */
    app_trace_count = index + 1u;
}
#else
/* Keep a valid C translation unit without allocating an object or emitting code. */
typedef char AppTraceDisabledTranslationUnit;
#endif
