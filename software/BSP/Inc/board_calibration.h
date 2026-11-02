#ifndef GIMBAL_BOARD_CALIBRATION_H
#define GIMBAL_BOARD_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>
#include "board_can.h"

enum {
    BOARD_CALIBRATION_STORED = 1u << 0,
    BOARD_CALIBRATION_STORAGE_ERROR = 1u << 1,
    BOARD_CALIBRATION_PROFILE_MISMATCH = 1u << 2,
    BOARD_CALIBRATION_RUNTIME_VALID = 1u << 3,
    BOARD_CALIBRATION_RESET_PENDING = 1u << 4
};

typedef struct {
    bool valid;
    bool storage_error;
    uint16_t motor_model;
    uint8_t pole_pairs;
    int8_t sensor_direction;
    float electrical_zero_rad;
} BoardCalibrationInfo;

void BoardCalibration_Load(void);
BoardCalibrationInfo BoardCalibration_GetInfo(void);
/* Terminal stopped-maintenance operation. Production sends both supplied
 * frames at the current node ID and then resets; it never returns. */
void BoardCalibration_CommitAndReset(bool clear, uint16_t motor_model,
    uint8_t pole_pairs, int8_t sensor_direction, float electrical_zero_rad,
    BoardCanFrame ack, BoardCanFrame info);

#endif
