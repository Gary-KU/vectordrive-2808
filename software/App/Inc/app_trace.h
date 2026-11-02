#ifndef GIMBAL_APP_TRACE_H
#define GIMBAL_APP_TRACE_H

#include "motor_config.h"

#if MOTOR_CONTROL_TRACE
#include <stdint.h>
#include "foc.h"

#ifndef APP_TRACE_CAPACITY
#define APP_TRACE_CAPACITY 128u
#endif
#ifndef APP_TRACE_DECIMATION
#define APP_TRACE_DECIMATION 8u
#endif
#if APP_TRACE_DECIMATION < 1u
#error Trace decimation must be positive
#endif

/* 52 bytes with 32-bit IEEE float: no enum/bool-dependent storage or pointers.
 * Offset: sample_count 0, raw_u 4, raw_w 6, ia_a 8 ... mode 48.
 * sample_count is the synchronized ADC callback count, not a clock timestamp.
 */
typedef struct {
    uint32_t sample_count;
    uint16_t raw_u;
    uint16_t raw_w;
    float ia_a;
    float ic_a;
    float id_a;
    float iq_a;
    float iq_reference_a;
    float vd_v;
    float vq_v;
    float mechanical_angle_rad;
    float mechanical_velocity_rad_s;
    float electrical_angle_rad;
    uint32_t mode;
} AppTraceRecord;

/* Halt before exporting or rearming. To start a fresh capture, set armed=0,
 * count=0, divider=0, then armed=1. Do not change controls while recording.
 * Only active (pwm_enabled) calls advance divider. The default records every
 * eighth call (4 ms at 2 kHz); CurrentLoopTest overrides this to 32 (16 ms).
 * CurrentLoopRunTest uses 256 rows at 4 ms, spanning up to 1.024 s.
 * Filling APP_TRACE_CAPACITY rows disarms
 * capture; existing rows are never overwritten unless count is explicitly reset.
 * pwm_enabled is the FOC request, not proof that the physical EN pin is high.
 */
extern volatile uint32_t app_trace_armed;
extern volatile uint32_t app_trace_count;
extern volatile uint32_t app_trace_divider;
extern volatile AppTraceRecord app_trace_buffer[APP_TRACE_CAPACITY];

void AppTrace_Record(uint32_t sample_count, uint16_t raw_u, uint16_t raw_w,
                     const FocOutput *out, float mechanical_angle,
                     float mechanical_velocity);

#else
/* Disabled builds allocate nothing, generate no call and evaluate no arguments. */
#define AppTrace_Record(sample_count, raw_u, raw_w, out, angle, velocity) ((void)0)
#endif

#endif
