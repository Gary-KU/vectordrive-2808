#include "calibration_store.h"
#include <math.h>
#include <string.h>

#define RECORD_MAGIC       0x434cu
#define RECORD_COMMIT      0xa55au
#define RECORD_VERSION     1u
#define RECORD_FLAG_VALID  1u
#define RECORD_HALFWORDS   (CALIBRATION_STORE_RECORD_BYTES / 2u)

static uint16_t crc16(const uint16_t *words)
{
    uint16_t crc = 0xffffu;
    for (unsigned byte = 0u; byte < 20u; ++byte) {
        crc ^= (uint16_t)(((words[byte / 2u] >> ((byte % 2u) * 8u)) & 0xffu) << 8);
        for (unsigned bit = 0u; bit < 8u; ++bit)
            crc = (uint16_t)((crc & 0x8000u) != 0u ?
                ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1);
    }
    return crc;
}

static bool read_record(const CalibrationStoreIo *io, unsigned page, unsigned slot,
                        uint16_t *words)
{
    bool erased = true;
    for (unsigned n = 0u; n < RECORD_HALFWORDS; ++n) {
        words[n] = io->read(io->context, page, slot * RECORD_HALFWORDS + n);
        if (words[n] != 0xffffu) erased = false;
    }
    return erased;
}

static uint32_t float_bits(float value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

static float bits_float(uint32_t value)
{
    float result;
    memcpy(&result, &value, sizeof(result));
    return result;
}

void CalibrationStore_Load(const CalibrationStoreIo *io, CalibrationStoreState *state)
{
    uint16_t words[RECORD_HALFWORDS];
    bool dirty = false;
    memset(state, 0, sizeof(*state));
    for (unsigned page = 0u; page < 2u; ++page) {
        for (unsigned slot = 0u; slot < CALIBRATION_STORE_RECORDS; ++slot) {
            uint32_t generation, inverse, zero_bits;
            uint8_t flags, pole_pairs;
            int8_t direction;
            float zero;
            if (read_record(io, page, slot, words)) continue;
            dirty = true;
            flags = (uint8_t)words[1];
            generation = (uint32_t)words[6] | ((uint32_t)words[7] << 16);
            inverse = (uint32_t)words[8] | ((uint32_t)words[9] << 16);
            if (words[0] != RECORD_MAGIC || (words[1] >> 8) != RECORD_VERSION ||
                (flags & ~RECORD_FLAG_VALID) != 0u || inverse != ~generation ||
                words[10] != crc16(words) || words[11] != RECORD_COMMIT) continue;
            pole_pairs = (uint8_t)words[3];
            direction = (int8_t)(words[3] >> 8);
            zero_bits = (uint32_t)words[4] | ((uint32_t)words[5] << 16);
            zero = bits_float(zero_bits);
            if ((flags & RECORD_FLAG_VALID) != 0u) {
                if (words[2] == 0u || pole_pairs == 0u || pole_pairs > 64u ||
                    (direction != 1 && direction != -1) || !isfinite(zero) ||
                    zero < 0.0f || zero >= 6.28318530717958647692f) continue;
            } else if (words[2] != 0u || words[3] != 0u || zero_bits != 0u) continue;
            if (!state->has_record || (int32_t)(generation - state->generation) > 0) {
                state->has_record = true;
                state->valid = (flags & RECORD_FLAG_VALID) != 0u;
                state->motor_model = state->valid ? words[2] : 0u;
                state->pole_pairs = state->valid ? pole_pairs : 0u;
                state->sensor_direction = state->valid ? direction : 0;
                state->electrical_zero_rad = state->valid ? zero : 0.0f;
                state->generation = generation;
                state->page = page;
            }
        }
    }
    state->storage_error = dirty && !state->has_record;
}

static bool append(const CalibrationStoreIo *io, bool valid, uint16_t motor_model,
                   uint8_t pole_pairs, int8_t direction, float zero,
                   CalibrationStoreState *state)
{
    uint16_t words[RECORD_HALFWORDS], scratch[RECORD_HALFWORDS];
    CalibrationStoreState loaded;
    uint32_t generation, zero_bits = valid ? float_bits(zero) : 0u;
    unsigned page = state->has_record ? state->page : 0u;
    unsigned slot;
    for (slot = 0u; slot < CALIBRATION_STORE_RECORDS; ++slot)
        if (read_record(io, page, slot, scratch)) break;
    if (slot == CALIBRATION_STORE_RECORDS) {
        page = state->has_record ? 1u - state->page : 1u;
        if (!io->erase(io->context, page)) return false;
        for (unsigned n = 0u; n < CALIBRATION_STORE_PAGE_BYTES / 2u; ++n)
            if (io->read(io->context, page, n) != 0xffffu) return false;
        slot = 0u;
    }
    generation = state->has_record ? state->generation + 1u : 1u;
    memset(words, 0, sizeof(words));
    words[0] = RECORD_MAGIC;
    words[1] = (uint16_t)((RECORD_VERSION << 8) | (valid ? RECORD_FLAG_VALID : 0u));
    words[2] = valid ? motor_model : 0u;
    words[3] = valid ? (uint16_t)((uint16_t)(uint8_t)direction << 8) | pole_pairs : 0u;
    words[4] = (uint16_t)zero_bits; words[5] = (uint16_t)(zero_bits >> 16);
    words[6] = (uint16_t)generation; words[7] = (uint16_t)(generation >> 16);
    words[8] = (uint16_t)~generation; words[9] = (uint16_t)(~generation >> 16);
    words[10] = crc16(words); words[11] = RECORD_COMMIT;
    for (unsigned n = 0u; n < RECORD_HALFWORDS; ++n)
        if (!io->program(io->context, page, slot * RECORD_HALFWORDS + n, words[n])) break;
    CalibrationStore_Load(io, &loaded);
    if (!loaded.has_record || loaded.generation != generation || loaded.valid != valid ||
        (valid && (loaded.motor_model != motor_model || loaded.pole_pairs != pole_pairs ||
                   loaded.sensor_direction != direction ||
                   float_bits(loaded.electrical_zero_rad) != zero_bits))) {
        state->storage_error = true;
        return false;
    }
    *state = loaded;
    return true;
}

bool CalibrationStore_Save(const CalibrationStoreIo *io, uint16_t motor_model,
                           uint8_t pole_pairs, int8_t sensor_direction,
                           float electrical_zero_rad, CalibrationStoreState *state)
{
    if (motor_model == 0u || pole_pairs == 0u || pole_pairs > 64u ||
        (sensor_direction != 1 && sensor_direction != -1) ||
        !isfinite(electrical_zero_rad) || electrical_zero_rad < 0.0f ||
        electrical_zero_rad >= 6.28318530717958647692f) return false;
    if (state->valid && state->motor_model == motor_model &&
        state->pole_pairs == pole_pairs && state->sensor_direction == sensor_direction &&
        float_bits(state->electrical_zero_rad) == float_bits(electrical_zero_rad)) return true;
    return append(io, true, motor_model, pole_pairs, sensor_direction,
                  electrical_zero_rad, state);
}

bool CalibrationStore_Clear(const CalibrationStoreIo *io, CalibrationStoreState *state)
{
    if (state->has_record && !state->valid) return true;
    return append(io, false, 0u, 0u, 0, 0.0f, state);
}
