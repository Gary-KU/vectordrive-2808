#ifndef GIMBAL_MOTOR_CONFIG_H
#define GIMBAL_MOTOR_CONFIG_H

/* Configuration entry point. Board wiring stays in board_config.h.
 * Fixed setup/safety: motor_setup.h; ADC calibration: current_sense_config.h;
 * test switches: bringup_config.h. Edit the shared tuning source and regenerate
 * motor_tuning_defaults.h for the nine runtime parameter defaults.
 * Compiler -D profile overrides remain effective through #ifndef defaults. */
#include "motor_setup.h"
#include "current_sense_config.h"
#include "bringup_config.h"
#include "motor_tuning_defaults.h"

#if MOTOR_BOOT_AUTO_ALIGN && MOTOR_OPEN_AUTOTEST
#error Select startup alignment or the open-loop auto test, never both
#endif

#endif
