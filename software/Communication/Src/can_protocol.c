#include "can_protocol.h"
#include "app.h"
#include "board.h"
#include "board_can.h"
#include "board_calibration.h"
#include "can_config.h"
#include "motor_config.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct {
    bool active;
    bool handshake_revoked;
    uint32_t tag;
    uint32_t epoch;
    uint8_t opcode;
    uint8_t sequence;
    uint8_t parameter_id;
} PendingRequest;

static bool initialized;
static bool sequence_valid;
static uint8_t sequence;
static uint32_t next_tag;
static uint32_t observed_epoch;
static uint32_t observed_rx_dropped;
static uint32_t observed_bus_off_count;
static bool previous_transport_unhealthy;
static bool rx_loss_latched;
static uint32_t last_feedback_ms;
static uint8_t last_execution_result;
static uint8_t trace_page;
static PendingRequest pending;
static CanProtocolStats statistics;
#define REPLY_QUEUE_SIZE 8u
static BoardCanFrame reply_queue[REPLY_QUEUE_SIZE];
static uint32_t reply_created_ms[REPLY_QUEUE_SIZE];
static uint8_t reply_head, reply_tail, reply_count;
static BoardCanFrame fast_frame, trace_frame;
static bool fast_pending, trace_pending;
static uint32_t last_fast_us, trace_created_ms;

static uint32_t node_id(uint32_t base) { return base + BoardCan_GetNodeId(); }

static uint16_t read_u16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

static int32_t read_i16(const uint8_t *bytes)
{
    uint16_t raw = read_u16(bytes);
    return raw >= 0x8000u ? (int32_t)raw - 65536 : (int32_t)raw;
}

static float read_float(const uint8_t *bytes)
{
    const uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
        ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void write_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
    bytes[2] = (uint8_t)(value >> 16);
    bytes[3] = (uint8_t)(value >> 24);
}

static void send_frame(const BoardCanFrame *frame)
{
    if (!BoardCan_Send(frame)) ++statistics.tx_dropped;
}

static void queue_reply(const BoardCanFrame *frame)
{
    if (reply_count == REPLY_QUEUE_SIZE) { ++statistics.tx_dropped; return; }
    reply_queue[reply_head] = *frame;
    reply_created_ms[reply_head] = Board_Millis();
    reply_head = (uint8_t)((reply_head + 1u) % REPLY_QUEUE_SIZE);
    ++reply_count;
}

static void flush_replies(void)
{
    /* A temporarily full hardware mailbox must not discard an execution ACK.
     * Bounded foreground work, at most eight attempts; no busy wait. */
    for (unsigned n = 0u; n < REPLY_QUEUE_SIZE && reply_count != 0u; ++n) {
        if ((uint32_t)(Board_Millis() - reply_created_ms[reply_tail]) > CAN_COMMAND_MAX_AGE_MS) {
            ++statistics.tx_dropped;
            reply_tail = (uint8_t)((reply_tail + 1u) % REPLY_QUEUE_SIZE);
            --reply_count;
            continue;
        }
        if (!BoardCan_Send(&reply_queue[reply_tail])) break;
        reply_tail = (uint8_t)((reply_tail + 1u) % REPLY_QUEUE_SIZE);
        --reply_count;
    }
}

static void acknowledge(uint8_t opcode, uint8_t request_sequence, uint8_t result,
                        uint32_t faults)
{
    BoardCanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = node_id(CAN_PROTOCOL_ACK_BASE_ID);
    frame.dlc = 8u;
    frame.data[0] = CAN_PROTOCOL_MAGIC;
    frame.data[1] = opcode;
    frame.data[2] = request_sequence;
    frame.data[3] = result;
    write_u32(&frame.data[4], faults);
    statistics.last_result = result;
    queue_reply(&frame);
}

static bool parameter_opcode(uint8_t opcode)
{
    return opcode == CAN_OPCODE_PARAM_READ || opcode == CAN_OPCODE_PARAM_WRITE;
}

static void parameter_response(uint8_t id, uint8_t request_sequence, uint8_t result, float value)
{
    BoardCanFrame frame;
    uint32_t bits;
    memset(&frame, 0, sizeof(frame));
    frame.id = node_id(CAN_PROTOCOL_PARAMETER_BASE_ID); frame.dlc = 8u;
    frame.data[0] = CAN_PROTOCOL_MAGIC; frame.data[1] = id;
    frame.data[2] = request_sequence; frame.data[3] = result;
    memcpy(&bits, &value, sizeof(bits));
    write_u32(&frame.data[4], bits);
    queue_reply(&frame);
}

static void parameter_snapshot_response(uint8_t id, uint8_t request_sequence, uint8_t result)
{
    AppStatus status;
    App_GetStatus(&status);
    parameter_response(id, request_sequence, result,
        id >= 1u && id <= APP_PARAM_COUNT ? status.parameters[id-1u] : 0.0f);
}

static void cancel_pending(uint32_t faults)
{
    if (!pending.active) return;
    acknowledge(pending.opcode, pending.sequence, (uint8_t)APP_RESULT_CANCELLED, faults);
    if (parameter_opcode(pending.opcode))
        parameter_snapshot_response(pending.parameter_id, pending.sequence, (uint8_t)APP_RESULT_CANCELLED);
    pending.active = false;
}

static void lock_session(uint32_t faults)
{
    (void)faults;
    statistics.require_stop = true;
    sequence_valid = false;
    fast_pending = trace_pending = false;
    /* Session revocation does not invent an application execution result.
     * In particular, a pending STOP may still be stopping LOCAL ownership.
     * Keep its tag until the application reports the actual result. */
    if (pending.active) pending.handshake_revoked = true;
}

static bool observe_application(const AppStatus *status)
{
    /* A completion reports execution, never mere queue acceptance. A STOP only
     * unlocks the epoch in which it was submitted. A later local STOP/fault
     * cannot accidentally be acknowledged by an older CAN STOP completion. */
    if (pending.active && status->can_completed_tag == pending.tag) {
        const uint8_t result = (uint8_t)status->can_result;
        acknowledge(pending.opcode, pending.sequence, result, status->faults);
        if (parameter_opcode(pending.opcode))
            parameter_response(pending.parameter_id, pending.sequence, result, status->can_parameter_value);
        last_execution_result = result;
        if (pending.opcode == CAN_OPCODE_STOP && !pending.handshake_revoked &&
            result == (uint8_t)APP_RESULT_OK &&
            status->can_session_epoch == pending.epoch) {
            statistics.require_stop = false;
            sequence_valid = true;
            sequence = pending.sequence;
        }
        pending.active = false;
    }
    if (status->can_session_epoch != observed_epoch) {
        observed_epoch = status->can_session_epoch;
        lock_session(status->faults);
        /* Cached STOP frames belong to the revoked session too. */
        BoardCan_DiscardRx();
        return true;
    }
    return false;
}

static bool observe_transport(const BoardCanStatus *transport, const AppStatus *status)
{
    const bool unhealthy = !transport->initialized || transport->bus_off;
    const bool dropped = transport->rx_dropped != observed_rx_dropped;
    const bool bus_off_event = transport->bus_off_count != observed_bus_off_count;
    const bool event = dropped || bus_off_event || (unhealthy && !previous_transport_unhealthy);
    observed_rx_dropped = transport->rx_dropped;
    observed_bus_off_count = transport->bus_off_count;
    previous_transport_unhealthy = unhealthy;
    if (dropped) rx_loss_latched = true;
    if (event) {
        reply_head = reply_tail = reply_count = 0u; /* No old replies after recovery. */
        fast_pending = trace_pending = false;
        ++statistics.transport_faults;
        App_NotifyCanTransportFault();
        lock_session(status->faults);
        BoardCan_DiscardRx();
    }
    if (unhealthy) {
        reply_head = reply_tail = reply_count = 0u;
        fast_pending = trace_pending = false;
        statistics.require_stop = true;
        BoardCan_DiscardRx();
    }
    return !unhealthy && !event;
}

static bool motion_opcode(uint8_t opcode)
{
    return opcode == CAN_OPCODE_IQ || opcode == CAN_OPCODE_SPEED ||
           opcode == CAN_OPCODE_OPEN || opcode == CAN_OPCODE_ALIGN;
}

static uint8_t decode(const BoardCanFrame *frame, AppCommandKind *kind,
                      float *value, float *auxiliary)
{
    int32_t target;
    uint16_t aux;
    const uint8_t opcode = frame->data[1];
    if (frame->extended || frame->remote || frame->dlc != 8u ||
        frame->data[0] != CAN_PROTOCOL_MAGIC) return CAN_RESULT_BAD_FRAME;
    if (parameter_opcode(opcode)) {
        const uint8_t id = frame->data[3];
        const float parameter = read_float(&frame->data[4]);
        if (id < 1u || id > APP_PARAM_COUNT || !isfinite(parameter)) return CAN_RESULT_RANGE;
        if (opcode == CAN_OPCODE_PARAM_READ && parameter != 0.0f) return CAN_RESULT_BAD_FRAME;
        *kind = opcode == CAN_OPCODE_PARAM_READ ? APP_CMD_PARAM_READ : APP_CMD_PARAM_WRITE;
        *value = parameter; *auxiliary = (float)id;
        return 0u;
    }
    if (frame->data[3] != (motion_opcode(opcode) ? 1u : 0u)) return CAN_RESULT_BAD_FRAME;
    target = read_i16(&frame->data[4]);
    aux = read_u16(&frame->data[6]);
    *value = 0.0f;
    *auxiliary = 0.0f;
    if (opcode != CAN_OPCODE_OPEN && aux != 0u) return CAN_RESULT_BAD_FRAME;
    switch (opcode) {
    case CAN_OPCODE_STOP: *kind = APP_CMD_STOP; break;
    case CAN_OPCODE_ESTOP: *kind = APP_CMD_ESTOP; break;
    case CAN_OPCODE_ALIGN: *kind = APP_CMD_ALIGN; break;
    case CAN_OPCODE_CLEAR: *kind = APP_CMD_CLEAR; break;
    case CAN_OPCODE_CALIBRATE: *kind = APP_CMD_CALIBRATE; break;
    case CAN_OPCODE_DIRECTION:
        if (target != 1 && target != -1) return CAN_RESULT_RANGE;
        *kind = APP_CMD_DIRECTION; *value = (float)target; return 0u;
    case CAN_OPCODE_IQ:
        if (target < -(int32_t)MOTOR_IQ_REQUEST_MAX_MA ||
            target > (int32_t)MOTOR_IQ_REQUEST_MAX_MA) return CAN_RESULT_RANGE;
        *kind = APP_CMD_CURRENT; *value = (float)target / 1000.0f; return 0u;
    case CAN_OPCODE_SPEED:
        if (fabsf((float)target) > MOTOR_SPEED_LIMIT_RAD_S * 1000.0f) return CAN_RESULT_RANGE;
        *kind = APP_CMD_SPEED; *value = (float)target / 1000.0f; return 0u;
    case CAN_OPCODE_OPEN:
        if (target < -10000 || target > 10000 || aux > 2000u ||
            (float)aux > MOTOR_VOLTAGE_LIMIT_V * 1000.0f) return CAN_RESULT_RANGE;
        *kind = APP_CMD_OPEN;
        *value = (float)target / 100.0f;
        *auxiliary = (float)aux / 1000.0f;
        return 0u;
    default: return CAN_RESULT_BAD_FRAME;
    }
    return target == 0 ? 0u : CAN_RESULT_BAD_FRAME;
}

static void reject(const BoardCanFrame *frame, uint8_t result, uint32_t faults)
{
    ++statistics.rejected_requests;
    acknowledge(frame->data[1], frame->data[2], result, faults);
    if (parameter_opcode(frame->data[1]))
        parameter_snapshot_response(frame->data[3], frame->data[2], result);
}

static void limits_request(const BoardCanFrame *frame)
{
    BoardCanFrame response;
    AppStatus status;
    uint8_t result = 0u;
    const uint8_t request_sequence = frame->data[2];
    App_GetStatus(&status);
    if (frame->extended || frame->remote || frame->dlc != 8u ||
        frame->data[0] != CAN_PROTOCOL_MAGIC) result = CAN_RESULT_BAD_FRAME;
    else if ((uint32_t)(Board_Millis() - frame->received_ms) > CAN_COMMAND_MAX_AGE_MS)
        result = CAN_RESULT_STALE;
    else if (frame->data[3] != 0u || frame->data[4] != 0u || frame->data[5] != 0u ||
             frame->data[6] != 0u || frame->data[7] != 0u) result = CAN_RESULT_BAD_FRAME;
    else if (sequence_valid && (uint8_t)(request_sequence - sequence) >= 128u)
        result = CAN_RESULT_SEQUENCE;
    else if (sequence_valid && request_sequence == sequence) result = CAN_RESULT_SEQUENCE;
    if (result != 0u) reject(frame, result, status.faults);
    else {
        /* Immutable capability inspection executes here. It neither enters
         * App's command mailbox nor clears faults/changes driver ownership. */
        acknowledge(CAN_OPCODE_LIMITS_READ, request_sequence, 0u, status.faults);
        sequence = request_sequence; sequence_valid = true;
        statistics.last_sequence = request_sequence;
        ++statistics.accepted_requests;
    }
    memset(&response, 0, sizeof(response));
    response.id = node_id(CAN_PROTOCOL_LIMITS_BASE_ID); response.dlc = 8u;
    response.data[0] = CAN_PROTOCOL_MAGIC; response.data[1] = CAN_OPCODE_LIMITS_READ;
    response.data[2] = request_sequence; response.data[3] = result;
    write_u16(&response.data[4], (uint16_t)MOTOR_IQ_REQUEST_MAX_MA);
    write_u16(&response.data[6], (uint16_t)(MOTOR_OVERCURRENT_TRIP_A * 1000.0f + 0.5f));
    queue_reply(&response); /* Shares bounded transaction queue; ACK is first. */
}

static void node_request(const BoardCanFrame *frame, bool *barrier)
{
    BoardCanNodeInfo node = BoardCan_GetNodeInfo();
    BoardCanFrame info, ack;
    AppStatus status;
    uint8_t result = 0u;
    uint8_t opcode = frame->data[1];
    uint8_t request_sequence = frame->data[2];
    uint32_t key;
    memset(&info, 0, sizeof(info));
    info.id = node_id(CAN_PROTOCOL_NODE_INFO_BASE_ID); info.dlc = 8u;
    info.data[0] = CAN_PROTOCOL_MAGIC; info.data[1] = opcode;
    info.data[2] = request_sequence;
    info.data[4] = node.active_id; info.data[5] = node.stored_id;
    info.data[6] = CAN_PROTOCOL_NODE_INFO_VERSION; info.data[7] = node.flags;
    App_GetStatus(&status);
    if (frame->extended || frame->remote || frame->dlc != 8u ||
        frame->data[0] != CAN_PROTOCOL_MAGIC) result = CAN_RESULT_BAD_FRAME;
    else if ((uint32_t)(Board_Millis() - frame->received_ms) > CAN_COMMAND_MAX_AGE_MS)
        result = CAN_RESULT_STALE;
    else if (opcode == CAN_OPCODE_NODE_READ &&
        (frame->data[3] != 0u || frame->data[4] != 0u || frame->data[5] != 0u ||
         frame->data[6] != 0u || frame->data[7] != 0u)) result = CAN_RESULT_BAD_FRAME;
    else if (opcode == CAN_OPCODE_NODE_SET && (frame->data[3] < 1u || frame->data[3] > 63u))
        result = CAN_RESULT_RANGE;
    else if (opcode == CAN_OPCODE_NODE_SET &&
        (frame->data[4] != 0x43u || frame->data[5] != 0x41u ||
         frame->data[6] != 0x4eu || frame->data[7] != 0x31u)) result = CAN_RESULT_BAD_FRAME;
    else if (sequence_valid && (uint8_t)(request_sequence - sequence) >= 128u)
        result = CAN_RESULT_SEQUENCE;
    else if (sequence_valid && request_sequence == sequence) result = CAN_RESULT_SEQUENCE;
    if (result != 0u) {
        reject(frame, result, status.faults);
        info.data[3] = result;
        queue_reply(&info);
        return;
    }
    if (opcode == CAN_OPCODE_NODE_READ) {
        acknowledge(opcode, request_sequence, 0u, status.faults);
        queue_reply(&info);
        sequence = request_sequence; sequence_valid = true;
        statistics.last_sequence = request_sequence;
        ++statistics.accepted_requests;
        return;
    }
#if MOTOR_OPEN_AUTOTEST
    /* OpenLoopSpin intentionally starts after software resets. A node-ID
     * commit/reset must never turn a stopped motor back on in such an image. */
    reject(frame, CAN_RESULT_UNSUPPORTED, status.faults);
    info.data[3] = CAN_RESULT_UNSUPPORTED;
    queue_reply(&info);
    return;
#endif
    key = Board_EnterCritical();
    App_GetStatus(&status);
    if (observe_application(&status) || statistics.require_stop) result = CAN_RESULT_REQUIRE_STOP;
    else if (pending.active) result = (uint8_t)APP_RESULT_BUSY;
    else if (status.faults != 0u) result = (uint8_t)APP_RESULT_FAULT;
    else if (!App_CanReinitializeCan()) result = (uint8_t)APP_RESULT_NEED_STOP;
    if (result != 0u) {
        Board_ExitCritical(key);
        reject(frame, result, status.faults);
        info.data[3] = result;
        queue_reply(&info);
        return;
    }
    /* Actual flash result is filled by the stopped terminal BSP operation.
     * The old IDs stay in these two frames even after the record is committed. */
    memset(&ack, 0, sizeof(ack));
    ack.id = node_id(CAN_PROTOCOL_ACK_BASE_ID); ack.dlc = 8u;
    ack.data[0] = CAN_PROTOCOL_MAGIC; ack.data[1] = opcode; ack.data[2] = request_sequence;
    initialized = false;
    statistics.require_stop = true;
    reply_head = reply_tail = reply_count = 0u;
    BoardCan_DiscardRx();
    BoardCan_CommitNodeIdAndReset(frame->data[3], ack, info);
    Board_ExitCritical(key); /* Fake-board tests return; production resets. */
    *barrier = true;
}

static void calibration_snapshot(BoardCanFrame *info, uint8_t opcode,
                                 uint8_t request_sequence, uint8_t result,
                                 const AppStatus *status)
{
    const BoardCalibrationInfo saved = BoardCalibration_GetInfo();
    const bool mismatch = saved.valid &&
        (saved.motor_model != (uint16_t)MOTOR_MODEL ||
         saved.pole_pairs != (uint8_t)MOTOR_POLE_PAIRS);
    uint8_t flags = (saved.valid ? CAN_CALIBRATION_FLAG_STORED : 0u) |
        (saved.storage_error ? CAN_CALIBRATION_FLAG_STORAGE_ERROR : 0u) |
        (mismatch ? CAN_CALIBRATION_FLAG_PROFILE_MISMATCH : 0u) |
        (status->zero_valid ? CAN_CALIBRATION_FLAG_RUNTIME_VALID : 0u);
    uint16_t zero = 0u;
    if (saved.valid) {
        uint32_t rounded = (uint32_t)(saved.electrical_zero_rad * 1000.0f + 0.5f);
        zero = (uint16_t)(rounded > 6283u ? 6283u : rounded);
    }
    memset(info, 0, sizeof(*info));
    info->id = node_id(CAN_PROTOCOL_CALIBRATION_BASE_ID); info->dlc = 8u;
    info->data[0] = CAN_PROTOCOL_MAGIC; info->data[1] = opcode;
    info->data[2] = request_sequence; info->data[3] = result;
    info->data[4] = flags;
    info->data[5] = saved.valid ? (uint8_t)saved.sensor_direction : 0u;
    write_u16(&info->data[6], zero);
}

static void calibration_request(const BoardCanFrame *frame, bool *barrier)
{
    BoardCanFrame info, ack;
    AppStatus status;
    const uint8_t opcode = frame->data[1];
    const uint8_t request_sequence = frame->data[2];
    uint8_t result = 0u;
    uint32_t key;
    const bool save = opcode == CAN_OPCODE_CALIBRATION_SAVE;
    const bool clear = opcode == CAN_OPCODE_CALIBRATION_CLEAR;
    App_GetStatus(&status);
    if (frame->extended || frame->remote || frame->dlc != 8u ||
        frame->data[0] != CAN_PROTOCOL_MAGIC) result = CAN_RESULT_BAD_FRAME;
    else if ((uint32_t)(Board_Millis() - frame->received_ms) > CAN_COMMAND_MAX_AGE_MS)
        result = CAN_RESULT_STALE;
    else if (opcode == CAN_OPCODE_CALIBRATION_READ &&
        (frame->data[3] != 0u || frame->data[4] != 0u || frame->data[5] != 0u ||
         frame->data[6] != 0u || frame->data[7] != 0u)) result = CAN_RESULT_BAD_FRAME;
    else if (save && (frame->data[3] != 0u || frame->data[4] != 0x43u ||
        frame->data[5] != 0x41u || frame->data[6] != 0x4cu ||
        frame->data[7] != 0x31u)) result = CAN_RESULT_BAD_FRAME;
    else if (clear && (frame->data[3] != 0u || frame->data[4] != 0x43u ||
        frame->data[5] != 0x4cu || frame->data[6] != 0x52u ||
        frame->data[7] != 0x31u)) result = CAN_RESULT_BAD_FRAME;
    else if (sequence_valid && (uint8_t)(request_sequence - sequence) >= 128u)
        result = CAN_RESULT_SEQUENCE;
    else if (sequence_valid && request_sequence == sequence) result = CAN_RESULT_SEQUENCE;
    if (result != 0u) {
        reject(frame, result, status.faults);
        calibration_snapshot(&info, opcode, request_sequence, result, &status);
        queue_reply(&info);
        return;
    }
    if (opcode == CAN_OPCODE_CALIBRATION_READ) {
        acknowledge(opcode, request_sequence, 0u, status.faults);
        calibration_snapshot(&info, opcode, request_sequence, 0u, &status);
        queue_reply(&info);
        sequence = request_sequence; sequence_valid = true;
        statistics.last_sequence = request_sequence;
        ++statistics.accepted_requests;
        return;
    }
#if MOTOR_OPEN_AUTOTEST
    reject(frame, CAN_RESULT_UNSUPPORTED, status.faults);
    calibration_snapshot(&info, opcode, request_sequence, CAN_RESULT_UNSUPPORTED, &status);
    queue_reply(&info);
    return;
#endif
    App_CancelStartupMotion();
    key = Board_EnterCritical();
    App_GetStatus(&status);
    if (observe_application(&status) || statistics.require_stop) result = CAN_RESULT_REQUIRE_STOP;
    else if (pending.active) result = (uint8_t)APP_RESULT_BUSY;
    else if (!App_CanReinitializeCan()) result = (uint8_t)APP_RESULT_NEED_STOP;
    else if (save && !status.offset_ready) result = (uint8_t)APP_RESULT_CALIBRATING;
    else if (save && !status.sensor_valid) result = (uint8_t)APP_RESULT_SENSOR;
    else if (save && (!status.direction_confirmed || !status.zero_valid))
        result = (uint8_t)APP_RESULT_NEED_ZERO;
    if (result != 0u) {
        Board_ExitCritical(key);
        reject(frame, result, status.faults);
        calibration_snapshot(&info, opcode, request_sequence, result, &status);
        queue_reply(&info);
        return;
    }
    memset(&ack, 0, sizeof(ack));
    ack.id = node_id(CAN_PROTOCOL_ACK_BASE_ID); ack.dlc = 8u;
    ack.data[0] = CAN_PROTOCOL_MAGIC; ack.data[1] = opcode; ack.data[2] = request_sequence;
    calibration_snapshot(&info, opcode, request_sequence, 0u, &status);
    initialized = false;
    statistics.require_stop = true;
    reply_head = reply_tail = reply_count = 0u;
    BoardCan_DiscardRx();
    BoardCalibration_CommitAndReset(clear, (uint16_t)MOTOR_MODEL,
        (uint8_t)MOTOR_POLE_PAIRS, status.sensor_direction,
        status.electrical_zero_rad, ack, info);
    Board_ExitCritical(key);
    *barrier = true;
}

static void receive_one(const BoardCanFrame *frame, bool *barrier)
{
    AppCommandKind kind = APP_CMD_STOP;
    float value, auxiliary;
    uint8_t result;
    uint8_t opcode;
    uint8_t request_sequence;
    bool safety;
    AppStatus status;
    uint32_t key;
    uint32_t tag;
    AppResult submitted;
    *barrier = false;
    if (frame->id != node_id(CAN_PROTOCOL_COMMAND_BASE_ID) && frame->id != CAN_ESTOP_ID) {
        ++statistics.ignored_frames;
        return;
    }
    App_GetStatus(&status);
    opcode = frame->data[1];
    request_sequence = frame->data[2];
    if (frame->id != CAN_ESTOP_ID && opcode == CAN_OPCODE_LIMITS_READ) {
        limits_request(frame);
        return;
    }
    if (frame->id != CAN_ESTOP_ID &&
        (opcode == CAN_OPCODE_NODE_READ || opcode == CAN_OPCODE_NODE_SET)) {
        node_request(frame, barrier);
        return;
    }
    if (frame->id != CAN_ESTOP_ID &&
        (opcode == CAN_OPCODE_CALIBRATION_READ ||
         opcode == CAN_OPCODE_CALIBRATION_SAVE ||
         opcode == CAN_OPCODE_CALIBRATION_CLEAR)) {
        calibration_request(frame, barrier);
        return;
    }
    result = decode(frame, &kind, &value, &auxiliary);
    if (result != 0u || (frame->id == CAN_ESTOP_ID && opcode != CAN_OPCODE_ESTOP)) {
        reject(frame, result != 0u ? result : CAN_RESULT_BAD_FRAME, status.faults);
        return;
    }
    if ((uint32_t)(Board_Millis() - frame->received_ms) > CAN_COMMAND_MAX_AGE_MS) {
        reject(frame, CAN_RESULT_STALE, status.faults);
        return;
    }
    if (motion_opcode(opcode) &&
        (BoardCan_GetNodeInfo().flags & BOARD_NODE_STORAGE_ERROR) != 0u) {
        /* Corrupt/interrupted first assignment may fall back to factory ID 1.
         * Keep diagnostics available, but do not let that fallback drive a
         * different motor until the operator explicitly repairs its identity. */
        reject(frame, CAN_RESULT_STORAGE, status.faults);
        return;
    }
    safety = opcode == CAN_OPCODE_STOP || opcode == CAN_OPCODE_ESTOP;
    /* A valid fresh operator request takes precedence over a boot test, even
     * when its sequence/session/ownership will subsequently reject motion. */
    App_CancelStartupMotion();
    /* This closes the epoch-check/submission race with a local STOP in ADC.
     * The application critical sections are nested and restore PRIMASK. */
    key = Board_EnterCritical();
    App_GetStatus(&status);
    if (observe_application(&status)) {
        Board_ExitCritical(key);
        reject(frame, CAN_RESULT_REQUIRE_STOP, status.faults);
        *barrier = true;
        return;
    }
    if (!safety && sequence_valid) {
        const uint8_t delta = (uint8_t)(request_sequence - sequence);
        if (delta == 0u || delta >= 128u) {
            Board_ExitCritical(key);
            reject(frame, CAN_RESULT_SEQUENCE, status.faults);
            return;
        }
    }
    if (motion_opcode(opcode) && statistics.require_stop) {
        Board_ExitCritical(key);
        reject(frame, CAN_RESULT_REQUIRE_STOP, status.faults);
        return;
    }
    if (pending.active && !safety) {
        Board_ExitCritical(key);
        reject(frame, (uint8_t)APP_RESULT_BUSY, status.faults);
        return;
    }
    if (safety) {
        cancel_pending(status.faults);
        statistics.require_stop = true;
        sequence_valid = false;
    }
    tag = ++next_tag;
    if (tag == 0u) tag = ++next_tag;
    submitted = App_SubmitCan(kind, value, auxiliary, tag, frame->received_ms);
    if (submitted == APP_RESULT_PENDING) {
        AppStatus accepted_status;
        App_GetStatus(&accepted_status);
        pending.active = true;
        pending.handshake_revoked = false;
        pending.tag = tag;
        pending.epoch = accepted_status.can_session_epoch;
        pending.opcode = opcode;
        pending.sequence = request_sequence;
        pending.parameter_id = parameter_opcode(opcode) ? frame->data[3] : 0u;
        sequence = request_sequence;
        sequence_valid = true;
        statistics.last_sequence = request_sequence;
        ++statistics.accepted_requests;
        /* ESTOP submission itself revokes the epoch. It must not cancel its
         * own outstanding execution ACK when that synchronous change is seen. */
        if (opcode == CAN_OPCODE_ESTOP) observed_epoch = accepted_status.can_session_epoch;
    }
    Board_ExitCritical(key);
    if (submitted != APP_RESULT_PENDING) {
        last_execution_result = (uint8_t)submitted;
        reject(frame, (uint8_t)submitted, status.faults);
    }
    if (safety) {
        /* FIFO order is a safety boundary: queued motion after a STOP/ESTOP is
         * discarded as well. Restart requires a later, freshly received frame. */
        BoardCan_DiscardRx();
        *barrier = true;
    }
}

static int16_t telemetry_i16(float value, float factor)
{
    float scaled;
    if (!isfinite(value)) return 0;
    scaled = value * factor;
    if (scaled >= 32767.0f) return INT16_MAX;
    if (scaled <= -32768.0f) return INT16_MIN;
    return (int16_t)scaled;
}

static uint8_t feedback_flags(const AppStatus *status)
{
    uint8_t flags = 0u;
    if (status->driver_enabled) flags |= 1u << 0;
    if (status->sense_confirmed) flags |= 1u << 1;
    if (status->offset_ready) flags |= 1u << 2;
    if (status->sensor_valid) flags |= 1u << 3;
    if (status->direction_confirmed) flags |= 1u << 4;
    if (status->zero_valid) flags |= 1u << 5;
    if (status->command_timed_out) flags |= 1u << 6;
    if (status->control_source == APP_SOURCE_CAN) flags |= 1u << 7;
    return flags;
}

static void prepare_fast(const AppStatus *status)
{
    float angle = status->angle_rad;
    if (fast_pending) ++statistics.tx_dropped; /* Coalesce the superseded value. */
    memset(&fast_frame, 0, sizeof(fast_frame));
    fast_frame.id = node_id(CAN_PROTOCOL_FAST_BASE_ID); fast_frame.dlc = 8u;
    fast_frame.data[0] = CAN_PROTOCOL_MAGIC;
    fast_frame.data[1] = feedback_flags(status);
    if (!status->sensor_valid || !isfinite(angle) || angle < 0.0f || angle > FOC_TWO_PI_F) {
        angle = 0.0f;
        fast_frame.data[1] &= (uint8_t)~(1u << 3);
    }
    write_u16(&fast_frame.data[2], (uint16_t)(angle * 1000.0f));
    write_u16(&fast_frame.data[4], (uint16_t)telemetry_i16(status->velocity_rad_s, 1000.0f));
    write_u16(&fast_frame.data[6], (uint16_t)telemetry_i16(
        status->sense_confirmed ? status->foc.iq_a : 0.0f, 1000.0f));
    fast_pending = true;
}

static void flush_periodic(void)
{
    if (reply_count != 0u) return; /* Executed-command replies always go first. */
    if (fast_pending && BoardCan_Send(&fast_frame)) fast_pending = false;
    if (trace_pending &&
        (uint32_t)(Board_Millis() - trace_created_ms) >= CAN_FEEDBACK_PERIOD_MS) {
        trace_pending = false;
        ++statistics.tx_dropped;
    }
    if (!fast_pending && trace_pending && BoardCan_Send(&trace_frame)) trace_pending = false;
}

static void publish_feedback(const AppStatus *status, const BoardCanStatus *transport)
{
    BoardCanFrame frame;
    uint8_t flags = feedback_flags(status);
    uint8_t diagnostic = 0u;
    memset(&frame, 0, sizeof(frame));
    frame.id = node_id(CAN_PROTOCOL_STATE_BASE_ID); frame.dlc = 8u;
    frame.data[0] = CAN_PROTOCOL_MAGIC;
    frame.data[1] = (uint8_t)status->foc.mode;
    frame.data[2] = flags; frame.data[3] = statistics.last_sequence;
    write_u16(&frame.data[4], (uint16_t)telemetry_i16(
        status->sense_confirmed ? status->foc.iq_a : 0.0f, 1000.0f));
    write_u16(&frame.data[6], (uint16_t)telemetry_i16(status->velocity_rad_s, 1000.0f));
    send_frame(&frame);

    frame.id = node_id(CAN_PROTOCOL_DIAGNOSTIC_BASE_ID);
    frame.data[1] = last_execution_result;
    write_u32(&frame.data[2], status->faults);
    frame.data[6] = (uint8_t)status->board_faults;
    if (transport->bus_off) diagnostic |= CAN_DIAG_BUS_OFF;
    if (transport->error_passive) diagnostic |= CAN_DIAG_ERROR_PASSIVE;
    if (rx_loss_latched) diagnostic |= CAN_DIAG_RX_LOSS;
    if (statistics.require_stop) diagnostic |= CAN_DIAG_REQUIRE_STOP;
    if (pending.active) diagnostic |= CAN_DIAG_REQUEST_PENDING;
    if (!transport->initialized) diagnostic |= CAN_DIAG_NOT_INITIALIZED;
    if (statistics.tx_dropped != 0u) diagnostic |= CAN_DIAG_TX_LOSS;
    frame.data[7] = diagnostic;
    send_frame(&frame);

    memset(&frame, 0, sizeof(frame));
    frame.id = node_id(CAN_PROTOCOL_TRACE_BASE_ID); frame.dlc = 8u;
    frame.data[0] = CAN_PROTOCOL_MAGIC; frame.data[1] = trace_page;
    if (trace_page == 0u) {
        write_u16(&frame.data[2], (uint16_t)telemetry_i16(status->foc.id_a, 1000.0f));
        write_u16(&frame.data[4], (uint16_t)telemetry_i16(status->foc.iq_reference_a, 1000.0f));
        write_u16(&frame.data[6], (uint16_t)telemetry_i16(status->foc.vd_v, 1000.0f));
    } else if (trace_page == 1u) {
        write_u16(&frame.data[2], (uint16_t)telemetry_i16(status->foc.vq_v, 1000.0f));
        write_u16(&frame.data[4], status->raw_u);
        write_u16(&frame.data[6], status->raw_w);
    } else {
        write_u16(&frame.data[2], (uint16_t)telemetry_i16(status->angle_rad, 1000.0f));
        write_u16(&frame.data[4], (uint16_t)telemetry_i16((float)status->max_isr_cycles,
            1000000.0f / (float)MOTOR_CPU_HZ));
        write_u16(&frame.data[6], (uint16_t)telemetry_i16(status->bus_voltage_v, 1000.0f));
    }
    if (trace_pending) ++statistics.tx_dropped;
    trace_frame = frame;
    trace_created_ms = Board_Millis();
    trace_pending = true;
    trace_page = (uint8_t)((trace_page + 1u) % 3u);
}

void CanProtocol_Init(void)
{
    AppStatus status;
    BoardCanStatus transport = BoardCan_GetStatus();
    memset(&statistics, 0, sizeof(statistics));
    memset(&pending, 0, sizeof(pending));
    reply_head = reply_tail = reply_count = 0u;
    fast_pending = trace_pending = false;
    App_GetStatus(&status);
    sequence_valid = false; sequence = 0u; next_tag = status.can_completed_tag;
    observed_epoch = status.can_session_epoch;
    observed_rx_dropped = transport.rx_dropped;
    observed_bus_off_count = transport.bus_off_count;
    previous_transport_unhealthy = !transport.initialized || transport.bus_off;
    rx_loss_latched = transport.rx_dropped != 0u;
    statistics.require_stop = true;
    statistics.last_result = CAN_RESULT_REQUIRE_STOP;
    last_execution_result = CAN_RESULT_REQUIRE_STOP;
    trace_page = 0u;
    last_feedback_ms = Board_Millis();
    last_fast_us = Board_Micros32();
    BoardCan_DiscardRx();
    initialized = true;
}

void CanProtocol_Poll(void)
{
    AppStatus status;
    BoardCanStatus transport;
    BoardCanFrame frame;
    unsigned count;
    if (!initialized) return;
    App_GetStatus(&status);
    observe_application(&status);
    transport = BoardCan_GetStatus();
    if (observe_transport(&transport, &status)) {
        for (count = 0u; count < CAN_MAX_RX_PER_POLL && BoardCan_Read(&frame); ++count) {
            bool barrier;
            transport = BoardCan_GetStatus();
            App_GetStatus(&status);
            if (observe_application(&status)) break;
            if (!observe_transport(&transport, &status)) break;
            receive_one(&frame, &barrier);
            if (!initialized) return;
            if (barrier) break;
        }
    }
    flush_replies();
    if ((uint32_t)(Board_Micros32() - last_fast_us) >= CAN_FAST_FEEDBACK_PERIOD_US) {
        last_fast_us = Board_Micros32();
        App_GetStatus(&status);
        observe_application(&status);
        transport = BoardCan_GetStatus();
        (void)observe_transport(&transport, &status);
        flush_replies();
        if (transport.initialized && !transport.bus_off) prepare_fast(&status);
    }
    /* FAST uses the next mailbox before status/diagnostic/low-priority trace.
     * Three hardware mailboxes cannot fit all four due frames simultaneously;
     * the latest trace slot below is retried on the next foreground poll. */
    if (reply_count == 0u && fast_pending && BoardCan_Send(&fast_frame)) fast_pending = false;
    if ((uint32_t)(Board_Millis() - last_feedback_ms) >= CAN_FEEDBACK_PERIOD_MS) {
        last_feedback_ms = Board_Millis();
        App_GetStatus(&status);
        observe_application(&status);
        transport = BoardCan_GetStatus();
        (void)observe_transport(&transport, &status);
        flush_replies();
        if (reply_count == 0u) publish_feedback(&status, &transport);
    }
    flush_periodic();
}

void CanProtocol_GetStats(CanProtocolStats *destination)
{
    if (destination != NULL) {
        *destination = statistics;
        destination->request_pending = pending.active;
    }
}
