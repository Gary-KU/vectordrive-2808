#ifndef GIMBAL_CAN_PROTOCOL_H
#define GIMBAL_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_PROTOCOL_MAGIC 0xa1u
#define CAN_PROTOCOL_BROADCAST_ESTOP_ID 0x080u
#define CAN_PROTOCOL_COMMAND_BASE_ID 0x200u
#define CAN_PROTOCOL_STATE_BASE_ID 0x180u
#define CAN_PROTOCOL_DIAGNOSTIC_BASE_ID 0x280u
#define CAN_PROTOCOL_ACK_BASE_ID 0x380u
#define CAN_PROTOCOL_PARAMETER_BASE_ID 0x480u
#define CAN_PROTOCOL_TRACE_BASE_ID 0x580u
#define CAN_PROTOCOL_LIMITS_BASE_ID 0x600u
#define CAN_PROTOCOL_CALIBRATION_BASE_ID 0x640u
#define CAN_PROTOCOL_NODE_INFO_BASE_ID 0x680u
#define CAN_PROTOCOL_FAST_BASE_ID 0x700u
#define CAN_PROTOCOL_NODE_INFO_VERSION 1u
#define CAN_NODE_FLAG_STORED (1u << 0)
#define CAN_NODE_FLAG_STORAGE_ERROR (1u << 1)
#define CAN_NODE_FLAG_RESET_PENDING (1u << 2)
#define CAN_CALIBRATION_FLAG_STORED (1u << 0)
#define CAN_CALIBRATION_FLAG_STORAGE_ERROR (1u << 1)
#define CAN_CALIBRATION_FLAG_PROFILE_MISMATCH (1u << 2)
#define CAN_CALIBRATION_FLAG_RUNTIME_VALID (1u << 3)
#define CAN_CALIBRATION_FLAG_RESET_PENDING (1u << 4)

typedef enum {
    CAN_OPCODE_STOP = 0,
    CAN_OPCODE_IQ = 1,
    CAN_OPCODE_SPEED = 2,
    CAN_OPCODE_OPEN = 3,
    CAN_OPCODE_ALIGN = 4,
    CAN_OPCODE_CLEAR = 5,
    CAN_OPCODE_DIRECTION = 6,
    CAN_OPCODE_CALIBRATE = 7,
    CAN_OPCODE_PARAM_READ = 0x10,
    CAN_OPCODE_PARAM_WRITE = 0x11,
    CAN_OPCODE_NODE_READ = 0x12,
    CAN_OPCODE_NODE_SET = 0x13,
    CAN_OPCODE_LIMITS_READ = 0x14,
    CAN_OPCODE_CALIBRATION_READ = 0x15,
    CAN_OPCODE_CALIBRATION_SAVE = 0x16,
    CAN_OPCODE_CALIBRATION_CLEAR = 0x17,
    CAN_OPCODE_ESTOP = 0x7f
} CanProtocolOpcode;

/* Application results occupy 0x00..0x7f; protocol rejections occupy 0x80..0xff. */
typedef enum {
    CAN_RESULT_BAD_FRAME = 0x80,
    CAN_RESULT_RANGE = 0x81,
    CAN_RESULT_SEQUENCE = 0x82,
    CAN_RESULT_STALE = 0x83,
    CAN_RESULT_REQUIRE_STOP = 0x84,
    CAN_RESULT_TRANSPORT = 0x85,
    CAN_RESULT_STORAGE = 0x86,
    CAN_RESULT_UNSUPPORTED = 0x87
} CanProtocolResult;

enum {
    CAN_DIAG_BUS_OFF = 1u << 0,
    CAN_DIAG_ERROR_PASSIVE = 1u << 1,
    CAN_DIAG_RX_LOSS = 1u << 2,       /* Latched until protocol reinitialization. */
    CAN_DIAG_REQUIRE_STOP = 1u << 3,
    CAN_DIAG_REQUEST_PENDING = 1u << 4,
    CAN_DIAG_NOT_INITIALIZED = 1u << 5,
    CAN_DIAG_TX_LOSS = 1u << 6        /* Any failed protocol TX submission. */
};

typedef struct {
    uint32_t accepted_requests;
    uint32_t rejected_requests;
    uint32_t ignored_frames;
    uint32_t tx_dropped;
    uint32_t transport_faults;
    uint8_t last_sequence;
    uint8_t last_result;
    bool require_stop;
    bool request_pending;
} CanProtocolStats;

/* LIMITS_READ [A1,14,seq,0,0,0,0,0] is read-only, available while faulted
 * and before STOP. It returns execution ACK then 0x600+node:
 * [A1,14,seq,result,u16 request_cap_mA,u16 fixed_phase_trip_mA], little-endian.
 * A request ceiling above the trip does not promise achievable current.
 *
 * FAST 0x700+node: [A1, status_flags, u16 angle_mrad, i16 velocity_mrad_s,
 * i16 iq_mA]. Latest asynchronous App snapshot; no sample timestamp or mode.
 * sensor_valid must be checked before using angle; status/diagnostic remain
 * mandatory to determine CURRENT mode and transport/application health.
 *
 * Foreground only, fixed-size storage, no blocking or dynamic allocation.
 * BoardCan_Init and App_Init must precede this initialization. Initial motion
 * requires an explicitly executed STOP handshake. Poll receives at most the
 * configured eight frames, state/diagnostics every 10 ms and FAST every 5 ms.
 * ACKs have priority. FAST and trace each coalesce into one pending latest
 * value under backpressure; no catch-up burst of old samples is emitted.
 */
void CanProtocol_Init(void);
void CanProtocol_Poll(void);
void CanProtocol_GetStats(CanProtocolStats *destination);

#ifdef __cplusplus
}
#endif
#endif
