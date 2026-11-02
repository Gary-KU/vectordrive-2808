#ifndef GIMBAL_CURRENT_SENSE_CONFIG_H
#define GIMBAL_CURRENT_SENSE_CONFIG_H

/* Verified board: U/W shunts 10 mOhm, INA181A2IDBVT 50 V/V, IN+ at the
 * low-side shunt and IN- at ground. Mapping/polarity were checked after the
 * DRV8313 repair; nominal R/gain/VDDA are not precision source calibration.
 * Set CONFIRMED=0 for another board until its own mapping is verified.
 * An unconfirmed board has no calibrated current feedback/protection. */
#ifndef MOTOR_CURRENT_SENSE_CONFIRMED
#define MOTOR_CURRENT_SENSE_CONFIRMED     1
#endif
#ifndef MOTOR_SHUNT_U_OHM
#define MOTOR_SHUNT_U_OHM                0.01f
#endif
#ifndef MOTOR_SHUNT_W_OHM
#define MOTOR_SHUNT_W_OHM                0.01f
#endif
#ifndef MOTOR_SENSE_GAIN_U
#define MOTOR_SENSE_GAIN_U               50.0f
#endif
#ifndef MOTOR_SENSE_GAIN_W
#define MOTOR_SENSE_GAIN_W               50.0f
#endif
/* Positive phase current is inverter -> winding. Verify both channels. */
#define MOTOR_SENSE_POLARITY_U           (-1.0f)
#define MOTOR_SENSE_POLARITY_W           (-1.0f)
#define MOTOR_ADC_REFERENCE_V            3.3f
#define MOTOR_ADC_FULL_SCALE            4095.0f
#define MOTOR_OFFSET_SAMPLE_COUNT       1024u
#ifndef MOTOR_OFFSET_MAX_NOISE_COUNTS
#define MOTOR_OFFSET_MAX_NOISE_COUNTS    128u
#endif
#define MOTOR_ADC_RAIL_MARGIN_COUNTS     32u

#endif
