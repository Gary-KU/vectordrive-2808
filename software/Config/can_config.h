#ifndef GIMBAL_CAN_CONFIG_H
#define GIMBAL_CAN_CONFIG_H

/* Classic CAN 2.0A. PA11/PA12, TJA1050, APB1 36 MHz. */
#define CAN_NODE_ID                     1u
/* Factory defaults below are retained for tooling/tests. Production routing
 * uses BoardCan_GetNodeId(), loaded from the reserved node-ID journal. */
#define CAN_BITRATE                     1000000u
#define CAN_COMMAND_ID                  (0x200u + CAN_NODE_ID)
#define CAN_ESTOP_ID                    0x080u
#define CAN_STATUS_ID                   (0x180u + CAN_NODE_ID)
#define CAN_DIAGNOSTIC_ID               (0x280u + CAN_NODE_ID)
#define CAN_ACK_ID                      (0x380u + CAN_NODE_ID)
#define CAN_PARAMETER_ID                (0x480u + CAN_NODE_ID)
#define CAN_TRACE_ID                    (0x580u + CAN_NODE_ID)
#define CAN_FAST_ID                     (0x700u + CAN_NODE_ID)
#define CAN_PROTOCOL_VERSION            0xA1u
#define CAN_FEEDBACK_PERIOD_MS          10u
#define CAN_FAST_FEEDBACK_PERIOD_US     5000u
#define CAN_COMMAND_MAX_AGE_MS          50u
#define CAN_MOTION_TIMEOUT_MS           100u
#define CAN_MAX_RX_PER_POLL             8u
#define CAN_INIT_RETRY_PERIOD_MS        500u

#if CAN_NODE_ID < 1 || CAN_NODE_ID > 63
#error CAN node ID must be between 1 and 63
#endif
#if CAN_BITRATE != 1000000u && CAN_BITRATE != 500000u && CAN_BITRATE != 250000u && CAN_BITRATE != 125000u
#error Supported CAN bitrates are 125k, 250k, 500k and 1M
#endif
#if CAN_COMMAND_MAX_AGE_MS >= CAN_MOTION_TIMEOUT_MS
#error CAN freshness limit must be shorter than the motion watchdog
#endif
#if CAN_INIT_RETRY_PERIOD_MS < 1u
#error CAN initialization retry interval must be positive
#endif
#if CAN_FAST_FEEDBACK_PERIOD_US < 1000u || CAN_FAST_FEEDBACK_PERIOD_US > 20000u || CAN_FAST_FEEDBACK_PERIOD_US % 1000u != 0u
#error FAST period must be whole milliseconds in 1..20 ms for mailbox expiry
#endif
#endif
