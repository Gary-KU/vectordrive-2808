#ifndef GIMBAL_CALIBRATION_STORE_H
#define GIMBAL_CALIBRATION_STORE_H

#include <stdbool.h>
#include <stdint.h>

/* Two 1 KiB pages, append-only 24-byte records.  The previous page remains
 * intact until a new record has been fully programmed and committed. */
#define CALIBRATION_STORE_PAGE_BYTES   1024u
#define CALIBRATION_STORE_RECORD_BYTES 24u
#define CALIBRATION_STORE_RECORDS \
    (CALIBRATION_STORE_PAGE_BYTES / CALIBRATION_STORE_RECORD_BYTES)

typedef struct {
    void *context;
    uint16_t (*read)(void *context, unsigned page, unsigned halfword);
    bool (*erase)(void *context, unsigned page);
    bool (*program)(void *context, unsigned page, unsigned halfword, uint16_t value);
} CalibrationStoreIo;

typedef struct {
    bool has_record;          /* Includes a committed clear/tombstone record. */
    bool valid;               /* Latest committed record contains calibration. */
    bool storage_error;       /* Dirty storage without any committed record. */
    uint16_t motor_model;
    uint8_t pole_pairs;
    int8_t sensor_direction;
    float electrical_zero_rad;
    uint32_t generation;
    unsigned page;
} CalibrationStoreState;

void CalibrationStore_Load(const CalibrationStoreIo *io, CalibrationStoreState *state);
bool CalibrationStore_Save(const CalibrationStoreIo *io, uint16_t motor_model,
                           uint8_t pole_pairs, int8_t sensor_direction,
                           float electrical_zero_rad, CalibrationStoreState *state);
bool CalibrationStore_Clear(const CalibrationStoreIo *io, CalibrationStoreState *state);

#endif
