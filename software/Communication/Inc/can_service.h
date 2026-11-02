#ifndef GIMBAL_CAN_SERVICE_H
#define GIMBAL_CAN_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Foreground lifecycle: start the CAN peripheral and protocol even if the
 * optional transceiver is not ready. Poll retries a failed peripheral init at
 * bounded intervals, only when App permits stopped communication maintenance.
 * A successful retry starts a new protocol session requiring a fresh STOP.
 * Bus-off of an initialized peripheral keeps the existing ABOM/fault policy.
 */
void CanService_Init(void);
void CanService_Poll(void);

#ifdef __cplusplus
}
#endif
#endif
