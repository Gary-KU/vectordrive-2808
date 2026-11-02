#include "can_protocol.h"
#include "app.h"
#include "board.h"
#include "board_can.h"
#include "board_calibration.h"
#include "can_config.h"
#include "motor_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CAPACITY 2048u
typedef struct {
    AppCommandKind kind;
    float value, auxiliary;
    uint32_t tag, received_ms;
} Submission;

static AppStatus app_status;
static BoardCanStatus transport;
static BoardCanFrame rx[CAPACITY], tx[CAPACITY];
static unsigned rx_head, rx_tail, tx_count;
static uint32_t now_ms;
static uint32_t now_us;
static unsigned discard_count, notify_count, submission_count;
static Submission submissions[CAPACITY];
static AppResult forced_submit_result;
static bool tx_accept;
static bool app_pending;
static bool epoch_on_critical;
static int mailbox_budget;
static bool tx_accepted[CAPACITY];
static uint8_t configured_node;
static unsigned node_commit_count;
static uint8_t node_flags;
static BoardCanFrame node_commit_ack, node_commit_info;
static BoardCalibrationInfo calibration_info;
static unsigned calibration_commit_count;
static bool calibration_commit_clear;
static uint16_t calibration_commit_model;
static uint8_t calibration_commit_poles;
static int8_t calibration_commit_direction;
static float calibration_commit_zero;
static BoardCanFrame calibration_commit_ack, calibration_commit_info;

uint8_t BoardCan_GetNodeId(void) { return configured_node; }
BoardCanNodeInfo BoardCan_GetNodeInfo(void)
{
    return (BoardCanNodeInfo){configured_node, configured_node, node_flags};
}
void BoardCan_CommitNodeIdAndReset(uint8_t id, BoardCanFrame ack, BoardCanFrame info)
{
    ++node_commit_count;
    node_commit_ack = ack; node_commit_info = info;
    configured_node = id; /* Next simulated boot loads the saved ID. */
}
void BoardCalibration_Load(void) { }
BoardCalibrationInfo BoardCalibration_GetInfo(void) { return calibration_info; }
void BoardCalibration_CommitAndReset(bool clear, uint16_t motor_model,
    uint8_t pole_pairs, int8_t sensor_direction, float electrical_zero_rad,
    BoardCanFrame ack, BoardCanFrame info)
{
    ++calibration_commit_count; calibration_commit_clear = clear;
    calibration_commit_model = motor_model; calibration_commit_poles = pole_pairs;
    calibration_commit_direction = sensor_direction; calibration_commit_zero = electrical_zero_rad;
    calibration_commit_ack = ack; calibration_commit_info = info;
}
bool App_CanReinitializeCan(void)
{
    return !app_status.driver_enabled && !app_pending &&
        app_status.control_source == APP_SOURCE_NONE &&
        (app_status.foc.mode == FOC_MODE_STOP || app_status.foc.mode == FOC_MODE_FAULT);
}

uint32_t Board_Millis(void) { return now_ms; }
uint32_t Board_Micros32(void) { return now_us; }
uint32_t Board_EnterCritical(void)
{
    if (epoch_on_critical) {
        ++app_status.can_session_epoch;
        epoch_on_critical = false;
    }
    return 0u;
}
void Board_ExitCritical(uint32_t key) { (void)key; }
BoardCanStatus BoardCan_GetStatus(void) { return transport; }
bool BoardCan_Read(BoardCanFrame *frame)
{
    if (rx_tail == rx_head) return false;
    *frame = rx[rx_tail++];
    return true;
}
void BoardCan_DiscardRx(void) { rx_tail = rx_head; ++discard_count; }
bool BoardCan_Send(const BoardCanFrame *frame)
{
    assert(tx_count < CAPACITY);
    tx[tx_count] = *frame;
    tx_accepted[tx_count] = tx_accept && mailbox_budget != 0;
    if (tx_accepted[tx_count] && mailbox_budget > 0) --mailbox_budget;
    return tx_accepted[tx_count++];
}
void App_GetStatus(AppStatus *status) { *status = app_status; }
void App_CancelStartupMotion(void) { /* Tested with the real App in startup tests. */ }
AppResult App_SubmitCan(AppCommandKind kind, float value, float auxiliary,
                        uint32_t tag, uint32_t received_ms)
{
    Submission *s;
    assert(submission_count < CAPACITY);
    s = &submissions[submission_count++];
    s->kind = kind; s->value = value; s->auxiliary = auxiliary;
    s->tag = tag; s->received_ms = received_ms;
    if (forced_submit_result != APP_RESULT_PENDING) return forced_submit_result;
    app_pending = true;
    if (kind == APP_CMD_STOP || kind == APP_CMD_ESTOP) app_status.driver_enabled = false;
    if (kind == APP_CMD_ESTOP) {
        ++app_status.can_session_epoch;
        app_status.driver_enabled = false;
    }
    return APP_RESULT_PENDING;
}
void App_NotifyCanTransportFault(void)
{
    const AppCommandKind kind = submission_count != 0u ?
        submissions[submission_count - 1u].kind : APP_CMD_STOP;
    const bool pending_motion = app_pending && (kind == APP_CMD_OPEN ||
        kind == APP_CMD_CURRENT || kind == APP_CMD_SPEED || kind == APP_CMD_ALIGN);
    ++notify_count;
    if (app_status.control_source == APP_SOURCE_CAN || pending_motion) {
        app_status.driver_enabled = false;
        app_status.control_source = APP_SOURCE_NONE;
        app_status.faults |= APP_FAULT_CAN;
        ++app_status.can_session_epoch;
        if (app_pending && submission_count != 0u) {
            app_status.can_completed_tag = submissions[submission_count - 1u].tag;
            app_status.can_result = APP_RESULT_CANCELLED;
            app_pending = false;
        }
    }
}

static void put_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value; bytes[1] = (uint8_t)(value >> 8);
}
static uint16_t get_u16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}
static uint32_t get_u32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}
static BoardCanFrame command(uint8_t opcode, uint8_t seq, int16_t target, uint16_t aux)
{
    BoardCanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = CAN_COMMAND_ID; frame.dlc = 8u; frame.received_ms = now_ms;
    frame.data[0] = CAN_PROTOCOL_MAGIC; frame.data[1] = opcode; frame.data[2] = seq;
    frame.data[3] = opcode >= CAN_OPCODE_IQ && opcode <= CAN_OPCODE_ALIGN ? 1u : 0u;
    put_u16(&frame.data[4], (uint16_t)target); put_u16(&frame.data[6], aux);
    return frame;
}
static void push(BoardCanFrame frame)
{
    assert(rx_head < CAPACITY);
    rx[rx_head++] = frame;
}
static void poll_frame(BoardCanFrame frame) { push(frame); CanProtocol_Poll(); }
static const BoardCanFrame *last_ack(void)
{
    unsigned i = tx_count;
    while (i != 0u) {
        --i;
        if (tx[i].id == CAN_ACK_ID) return &tx[i];
    }
    assert(0); return NULL;
}
static unsigned ack_count(void)
{
    unsigned i, count = 0u;
    for (i = 0u; i < tx_count; ++i) if (tx[i].id == CAN_ACK_ID) ++count;
    return count;
}
static const BoardCanFrame *last_frame(uint32_t id)
{
    unsigned i = tx_count;
    while (i != 0u) {
        --i;
        if (tx[i].id == id) return &tx[i];
    }
    assert(0); return NULL;
}
static CanProtocolStats stats(void)
{
    CanProtocolStats result;
    CanProtocol_GetStats(&result); return result;
}
static void reset(void)
{
    memset(&app_status, 0, sizeof(app_status));
    memset(&transport, 0, sizeof(transport));
    transport.initialized = true;
    rx_head = rx_tail = tx_count = 0u;
    now_ms = discard_count = notify_count = submission_count = 0u;
    now_us = 0u;
    forced_submit_result = APP_RESULT_PENDING;
    tx_accept = true; app_pending = false; epoch_on_critical = false;
    mailbox_budget = -1;
    configured_node = CAN_NODE_ID; node_commit_count = 0u; node_flags = 0u;
    memset(&calibration_info, 0, sizeof(calibration_info));
    calibration_commit_count = 0u; calibration_commit_clear = false;
    calibration_commit_model = 0u; calibration_commit_poles = 0u;
    calibration_commit_direction = 0; calibration_commit_zero = 0.0f;
    CanProtocol_Init();
    assert(stats().require_stop);
    assert(submission_count == 0u); /* No automatic startup motion. */
}
static void complete(AppResult result)
{
    const Submission *s;
    assert(submission_count != 0u);
    s = &submissions[submission_count - 1u];
    app_status.can_completed_tag = s->tag; app_status.can_result = result;
    app_status.last_result = result; app_pending = false;
    if (result == APP_RESULT_OK) {
        switch (s->kind) {
        case APP_CMD_STOP:
            app_status.control_source = APP_SOURCE_NONE;
            app_status.driver_enabled = false; app_status.foc.mode = FOC_MODE_STOP; break;
        case APP_CMD_ESTOP:
            app_status.control_source = APP_SOURCE_NONE;
            app_status.driver_enabled = false; app_status.foc.mode = FOC_MODE_FAULT;
            app_status.faults |= APP_FAULT_ESTOP; ++app_status.can_session_epoch; break;
        case APP_CMD_CURRENT: case APP_CMD_SPEED: case APP_CMD_OPEN: case APP_CMD_ALIGN:
            app_status.control_source = APP_SOURCE_CAN; app_status.driver_enabled = true;
            app_status.foc.mode = s->kind == APP_CMD_CURRENT ? FOC_MODE_CURRENT :
                (s->kind == APP_CMD_SPEED ? FOC_MODE_SPEED :
                (s->kind == APP_CMD_OPEN ? FOC_MODE_OPEN_LOOP : FOC_MODE_ALIGNMENT));
            break;
        case APP_CMD_PARAM_WRITE:
            app_status.parameters[(unsigned)s->auxiliary-1u] = s->value;
            app_status.can_parameter_value = s->value;
            break;
        case APP_CMD_PARAM_READ:
            app_status.can_parameter_value = app_status.parameters[(unsigned)s->auxiliary-1u];
            break;
        default: break;
        }
    }
    CanProtocol_Poll();
}
static void handshake(uint8_t seq)
{
    unsigned before = ack_count();
    poll_frame(command(CAN_OPCODE_STOP, seq, 0, 0u));
    assert(stats().request_pending && stats().require_stop);
    assert(ack_count() == before); /* PENDING must not be advertised as execution OK. */
    complete(APP_RESULT_OK);
    assert(!stats().require_stop && !stats().request_pending);
    assert(last_ack()->data[1] == CAN_OPCODE_STOP);
    assert(last_ack()->data[2] == seq && last_ack()->data[3] == APP_RESULT_OK);
}
static void reject_frame(BoardCanFrame frame, uint8_t expected)
{
    unsigned before = submission_count;
    poll_frame(frame);
    assert(submission_count == before + (expected == APP_RESULT_CONTROL_OWNED ? 1u : 0u));
    assert(last_ack()->data[3] == expected);
}

static void test_framing_and_units(void)
{
    BoardCanFrame frame;
    const int16_t current_limit_ma = (int16_t)MOTOR_IQ_REQUEST_MAX_MA;
    const uint16_t voltage_limit_mv = (uint16_t)fminf(2000.0f,
        floorf(MOTOR_VOLTAGE_LIMIT_V * 1000.0f));
    unsigned i;
    reset();
    reject_frame(command(CAN_OPCODE_IQ, 5u, 100, 0u), CAN_RESULT_REQUIRE_STOP);
    handshake(250u);
    frame = command(CAN_OPCODE_IQ, 251u, -50, 0u);
    for (i = 0u; i <= 8u; ++i) {
        if (i == 8u) continue;
        frame.dlc = (uint8_t)i; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    }
    frame.dlc = 9u; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame.dlc = 8u; frame.extended = true; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame.extended = false; frame.remote = true; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame.remote = false; frame.data[0] = 0xa0u; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame.data[0] = CAN_PROTOCOL_MAGIC; frame.data[3] = 3u; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame.data[3] = 0u; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame.data[3] = 1u; frame.data[6] = 1u; reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame.data[6] = 0u;
    poll_frame(frame);
    assert(submissions[submission_count - 1u].kind == APP_CMD_CURRENT);
    assert(fabsf(submissions[submission_count - 1u].value + 0.05f) < 1e-6f);
    complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_SPEED, 252u, -12345, 0u));
    assert(fabsf(submissions[submission_count - 1u].value + 12.345f) < 1e-5f);
    complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_OPEN, 253u, -2000, 800u));
    assert(submissions[submission_count - 1u].kind == APP_CMD_OPEN);
    assert(fabsf(submissions[submission_count - 1u].value + 20.0f) < 1e-6f);
    assert(fabsf(submissions[submission_count - 1u].auxiliary - 0.8f) < 1e-6f);
    complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_DIRECTION, 254u, -1, 0u));
    assert(submissions[submission_count - 1u].value == -1.0f);
    complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_IQ, 255u, current_limit_ma, 0u));
    assert(submissions[submission_count - 1u].value == (float)current_limit_ma / 1000.0f);
    complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_SPEED, 0u, 30000, 0u));
    assert(submissions[submission_count - 1u].value == 30.0f); complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_OPEN, 1u, 10000, voltage_limit_mv));
    assert(submissions[submission_count - 1u].value == 100.0f);
    assert(submissions[submission_count - 1u].auxiliary == (float)voltage_limit_mv / 1000.0f);
    complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_IQ, 2u, (int16_t)-current_limit_ma, 0u));
    assert(submissions[submission_count - 1u].value == -(float)current_limit_ma / 1000.0f);
    complete(APP_RESULT_OK);
    reject_frame(command(CAN_OPCODE_IQ, 255u, (int16_t)(current_limit_ma + 1), 0u), CAN_RESULT_RANGE);
    reject_frame(command(CAN_OPCODE_IQ, 255u, (int16_t)(-current_limit_ma - 1), 0u), CAN_RESULT_RANGE);
    reject_frame(command(CAN_OPCODE_SPEED, 255u, 30001, 0u), CAN_RESULT_RANGE);
    reject_frame(command(CAN_OPCODE_OPEN, 255u, 10001, 0u), CAN_RESULT_RANGE);
    reject_frame(command(CAN_OPCODE_OPEN, 255u, 10000, (uint16_t)(voltage_limit_mv + 1u)), CAN_RESULT_RANGE);
    reject_frame(command(CAN_OPCODE_DIRECTION, 255u, 0, 0u), CAN_RESULT_RANGE);
    reject_frame(command(CAN_OPCODE_CLEAR, 255u, 1, 0u), CAN_RESULT_BAD_FRAME);
    frame = command(0x55u, 255u, 0, 0u); reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame = command(CAN_OPCODE_STOP, 1u, 0, 0u); frame.data[3] = 1u;
    reject_frame(frame, CAN_RESULT_BAD_FRAME);
    frame = command(CAN_OPCODE_IQ, 255u, 100, 0u); frame.id = 0x123u;
    { unsigned before = ack_count(); poll_frame(frame); assert(ack_count() == before); }
}

static void test_sequence_and_ack(void)
{
    unsigned before;
    uint32_t tag;
    reset(); handshake(254u);
    poll_frame(command(CAN_OPCODE_IQ, 255u, 100, 0u));
    tag = submissions[submission_count - 1u].tag;
    before = ack_count();
    app_status.can_completed_tag = tag + 123u; app_status.can_result = APP_RESULT_OK;
    CanProtocol_Poll(); assert(ack_count() == before && stats().request_pending);
    reject_frame(command(CAN_OPCODE_IQ, 0u, 100, 0u), APP_RESULT_BUSY);
    complete(APP_RESULT_SENSE_UNKNOWN);
    assert(last_ack()->data[2] == 255u && last_ack()->data[3] == APP_RESULT_SENSE_UNKNOWN);
    reject_frame(command(CAN_OPCODE_IQ, 255u, 100, 0u), CAN_RESULT_SEQUENCE);
    poll_frame(command(CAN_OPCODE_IQ, 0u, -100, 0u)); complete(APP_RESULT_OK);
    reject_frame(command(CAN_OPCODE_IQ, 255u, 100, 0u), CAN_RESULT_SEQUENCE);
    reject_frame(command(CAN_OPCODE_IQ, 128u, 100, 0u), CAN_RESULT_SEQUENCE);
    poll_frame(command(CAN_OPCODE_IQ, 127u, 100, 0u)); complete(APP_RESULT_OK);
    /* STOP ignores old/repeated sequence and resets the window. */
    handshake(5u);
    poll_frame(command(CAN_OPCODE_OPEN, 6u, 100, 100u)); complete(APP_RESULT_OK);
    forced_submit_result = APP_RESULT_CONTROL_OWNED;
    reject_frame(command(CAN_OPCODE_IQ, 7u, 100, 0u), APP_RESULT_CONTROL_OWNED);
    forced_submit_result = APP_RESULT_PENDING;
    poll_frame(command(CAN_OPCODE_IQ, 7u, 100, 0u)); complete(APP_RESULT_OK);
}

static void test_safety_barriers_and_epoch(void)
{
    unsigned before;
    BoardCanFrame broadcast;
    reset(); handshake(10u);
    poll_frame(command(CAN_OPCODE_IQ, 11u, 100, 0u));
    before = ack_count();
    push(command(CAN_OPCODE_STOP, 1u, 0, 0u));
    push(command(CAN_OPCODE_OPEN, 2u, 100, 800u));
    CanProtocol_Poll();
    assert(last_ack()->data[2] == 11u && last_ack()->data[3] == APP_RESULT_CANCELLED);
    assert(ack_count() == before + 1u);
    assert(submissions[submission_count - 1u].kind == APP_CMD_STOP);
    assert(rx_tail == rx_head); complete(APP_RESULT_OK);
    before = submission_count; CanProtocol_Poll(); assert(submission_count == before);

    push(command(CAN_OPCODE_STOP, 2u, 0, 0u));
    push(command(CAN_OPCODE_OPEN, 3u, 100, 800u));
    ++app_status.can_session_epoch; /* Local stop happened while old frames accumulated. */
    CanProtocol_Poll();
    assert(submission_count == before && rx_tail == rx_head && stats().require_stop);
    reject_frame(command(CAN_OPCODE_OPEN, 4u, 100, 800u), CAN_RESULT_REQUIRE_STOP);
    handshake(50u);

    /* STOP completed, then local STOP before protocol observes its ACK. */
    poll_frame(command(CAN_OPCODE_STOP, 51u, 0, 0u));
    app_status.can_completed_tag = submissions[submission_count - 1u].tag;
    app_status.can_result = APP_RESULT_OK;
    ++app_status.can_session_epoch;
    CanProtocol_Poll(); assert(stats().require_stop);

    handshake(60u);
    epoch_on_critical = true;
    reject_frame(command(CAN_OPCODE_OPEN, 61u, 100, 800u), CAN_RESULT_REQUIRE_STOP);
    assert(stats().require_stop);

    broadcast = command(CAN_OPCODE_STOP, 0u, 0, 0u); broadcast.id = CAN_ESTOP_ID;
    reject_frame(broadcast, CAN_RESULT_BAD_FRAME);
    broadcast = command(CAN_OPCODE_ESTOP, 0u, 0, 0u); broadcast.id = CAN_ESTOP_ID;
    poll_frame(broadcast);
    assert(submissions[submission_count - 1u].kind == APP_CMD_ESTOP);
    assert(stats().request_pending); /* Synchronous ESTOP epoch must not cancel its own ACK. */
    complete(APP_RESULT_OK);
    assert(last_ack()->data[1] == CAN_OPCODE_ESTOP && last_ack()->data[3] == APP_RESULT_OK);
    assert(stats().require_stop && app_status.faults == APP_FAULT_ESTOP);
    assert(get_u32(&last_ack()->data[4]) == APP_FAULT_ESTOP);
}

static void test_time_and_transport(void)
{
    BoardCanFrame frame;
    unsigned before;
    reset(); handshake(10u);
    now_ms = 1000u;
    frame = command(CAN_OPCODE_IQ, 11u, 100, 0u);
    frame.received_ms = now_ms - CAN_COMMAND_MAX_AGE_MS - 1u;
    reject_frame(frame, CAN_RESULT_STALE);
    frame.received_ms = now_ms + 1u; reject_frame(frame, CAN_RESULT_STALE);
    frame.received_ms = now_ms - CAN_COMMAND_MAX_AGE_MS;
    poll_frame(frame);
    assert(submissions[submission_count - 1u].received_ms == frame.received_ms);
    complete(APP_RESULT_OK);
    /* Timestamp arithmetic is defined across the 32-bit millisecond wrap. */
    now_ms = 5u;
    frame = command(CAN_OPCODE_IQ, 12u, 100, 0u);
    frame.received_ms = UINT32_MAX - 4u;
    poll_frame(frame); complete(APP_RESULT_OK);
    app_status.control_source = APP_SOURCE_CAN;
    push(command(CAN_OPCODE_STOP, 13u, 0, 0u));
    push(command(CAN_OPCODE_IQ, 14u, 100, 0u));
    before = submission_count;
    ++transport.rx_dropped;
    CanProtocol_Poll();
    assert(notify_count == 1u && !app_status.driver_enabled && stats().require_stop);
    assert(submission_count == before && rx_tail == rx_head);
    /* Recovered bus-off still revokes the session through the event counter. */
    ++transport.bus_off_count; transport.bus_off = false;
    CanProtocol_Poll(); assert(notify_count == 2u && stats().require_stop);
    handshake(15u);
    transport.bus_off = true;
    CanProtocol_Poll(); assert(notify_count == 3u && stats().require_stop);
    before = submission_count;
    poll_frame(command(CAN_OPCODE_STOP, 20u, 0, 0u));
    assert(submission_count == before);
    transport.bus_off = false; CanProtocol_Poll();
    reject_frame(command(CAN_OPCODE_IQ, 21u, 100, 0u), CAN_RESULT_REQUIRE_STOP);

    reset();
    app_status.control_source = APP_SOURCE_LOCAL; app_status.driver_enabled = true;
    ++transport.rx_dropped; CanProtocol_Poll();
    assert(app_status.driver_enabled && app_status.faults == 0u); /* Transport does not stop local ownership. */
    transport.initialized = false;
    before = submission_count;
    poll_frame(command(CAN_OPCODE_STOP, 1u, 0, 0u));
    assert(submission_count == before && stats().require_stop);
}

static void test_telemetry_and_bounds(void)
{
    const BoardCanFrame *frame;
    unsigned i;
    reset();
    app_status.foc.mode = FOC_MODE_CURRENT; app_status.driver_enabled = true;
    app_status.sense_confirmed = app_status.offset_ready = app_status.sensor_valid = true;
    app_status.direction_confirmed = app_status.zero_valid = app_status.command_timed_out = true;
    app_status.control_source = APP_SOURCE_CAN;
    app_status.foc.iq_a = -100.0f; app_status.velocity_rad_s = 100.0f;
    app_status.faults = 0x12345678u; app_status.board_faults = 0x1abu;
    transport.error_passive = true;
    now_ms = CAN_FEEDBACK_PERIOD_MS - 1u; CanProtocol_Poll(); assert(tx_count == 0u);
    now_ms = CAN_FEEDBACK_PERIOD_MS; CanProtocol_Poll();
    frame = last_frame(CAN_STATUS_ID);
    assert(frame->dlc == 8u && frame->data[0] == CAN_PROTOCOL_MAGIC);
    assert(frame->data[1] == FOC_MODE_CURRENT && frame->data[2] == 0xffu);
    assert(get_u16(&frame->data[4]) == 0x8000u && get_u16(&frame->data[6]) == 0x7fffu);
    frame = last_frame(CAN_DIAGNOSTIC_ID);
    assert(get_u32(&frame->data[2]) == 0x12345678u && frame->data[6] == 0xabu);
    assert(frame->data[7] & CAN_DIAG_ERROR_PASSIVE);
    assert(frame->data[7] & CAN_DIAG_REQUIRE_STOP);
    app_status.sense_confirmed = false; app_status.foc.iq_a = NAN;
    now_ms += CAN_FEEDBACK_PERIOD_MS; CanProtocol_Poll();
    assert(get_u16(&last_frame(CAN_STATUS_ID)->data[4]) == 0u);
    tx_accept = false;
    now_ms += CAN_FEEDBACK_PERIOD_MS; CanProtocol_Poll(); assert(stats().tx_dropped == 2u);
    assert(last_frame(CAN_DIAGNOSTIC_ID)->data[7] & CAN_DIAG_TX_LOSS);

    reset();
    for (i = 0u; i < CAN_MAX_RX_PER_POLL + 3u; ++i) {
        BoardCanFrame ignored = command(CAN_OPCODE_STOP, 0u, 0, 0u);
        ignored.id = 0x123u; push(ignored);
    }
    CanProtocol_Poll(); assert(rx_tail == CAN_MAX_RX_PER_POLL);
    assert(stats().ignored_frames == CAN_MAX_RX_PER_POLL);
    CanProtocol_Poll(); assert(rx_tail == rx_head);

    reset();
    /* CLEAR/DIR/CAL are allowed before a STOP handshake, ALIGN is not. */
    poll_frame(command(CAN_OPCODE_CLEAR, 1u, 0, 0u)); complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_DIRECTION, 2u, 1, 0u)); complete(APP_RESULT_OK);
    poll_frame(command(CAN_OPCODE_CALIBRATE, 3u, 0, 0u)); complete(APP_RESULT_OK);
    reject_frame(command(CAN_OPCODE_ALIGN, 4u, 0, 0u), CAN_RESULT_REQUIRE_STOP);
    handshake(4u);
    poll_frame(command(CAN_OPCODE_ALIGN, 5u, 0, 0u)); complete(APP_RESULT_OK);
    assert(submissions[submission_count - 1u].kind == APP_CMD_ALIGN);
}

static void test_revoked_pending_results(void)
{
    static const uint8_t nonmotion[] = {
        CAN_OPCODE_DIRECTION, CAN_OPCODE_CALIBRATE, CAN_OPCODE_CLEAR
    };
    unsigned before, i;
    uint32_t tag;
    /* A STOP taking over LOCAL motion must remain queued through a transport
     * failure. It may complete, but cannot unlock the failed handshake. */
    reset();
    app_status.control_source = APP_SOURCE_LOCAL;
    app_status.driver_enabled = true;
    app_status.foc.mode = FOC_MODE_OPEN_LOOP;
    poll_frame(command(CAN_OPCODE_STOP, 20u, 0, 0u));
    before = ack_count();
    ++transport.rx_dropped; CanProtocol_Poll();
    assert(stats().request_pending && app_pending && ack_count() == before);
    assert(!app_status.driver_enabled && app_status.faults == 0u);
    complete(APP_RESULT_OK);
    assert(last_ack()->data[1] == CAN_OPCODE_STOP && last_ack()->data[3] == APP_RESULT_OK);
    assert(stats().require_stop && !stats().request_pending);
    assert(app_status.foc.mode == FOC_MODE_STOP);
    handshake(21u);

    for (i = 0u; i < sizeof(nonmotion) / sizeof(nonmotion[0]); ++i) {
        reset();
        poll_frame(command(nonmotion[i], 10u, nonmotion[i] == CAN_OPCODE_DIRECTION ? 1 : 0, 0u));
        before = ack_count();
        ++transport.rx_dropped; CanProtocol_Poll();
        assert(stats().request_pending && app_pending && ack_count() == before);
        assert(app_status.faults == 0u);
        complete(APP_RESULT_OK);
        assert(last_ack()->data[1] == nonmotion[i] && last_ack()->data[3] == APP_RESULT_OK);
        assert(stats().require_stop);
    }

    /* Actual cancellation is reported through the application's matching tag. */
    reset(); handshake(30u);
    poll_frame(command(CAN_OPCODE_IQ, 31u, 100, 0u));
    tag = submissions[submission_count - 1u].tag;
    before = ack_count();
    ++app_status.can_session_epoch;
    CanProtocol_Poll();
    assert(stats().request_pending && ack_count() == before); /* No invented ACK. */
    app_status.can_completed_tag = tag;
    app_status.can_result = APP_RESULT_CANCELLED;
    app_pending = false;
    CanProtocol_Poll();
    assert(!stats().request_pending && stats().require_stop);
    assert(last_ack()->data[2] == 31u && last_ack()->data[3] == APP_RESULT_CANCELLED);
}

static BoardCanFrame parameter_command(uint8_t opcode, uint8_t sequence_number, uint8_t id, float value)
{
    BoardCanFrame frame = command(opcode,sequence_number,0,0u);
    frame.data[3] = id;
    memcpy(&frame.data[4],&value,sizeof(value));
    return frame;
}

static float frame_float(const BoardCanFrame *frame)
{
    float value;
    memcpy(&value,&frame->data[4],sizeof(value));
    return value;
}

static void test_parameter_framing_results_and_trace(void)
{
    const BoardCanFrame *frame;
    reset();
    app_status.parameters[0] = 1.0f;
    poll_frame(parameter_command(CAN_OPCODE_PARAM_READ,1u,1u,0.0f));
    assert(ack_count() == 0u && stats().request_pending);
    assert(submissions[0].kind == APP_CMD_PARAM_READ && submissions[0].auxiliary == 1.0f);
    complete(APP_RESULT_OK);
    frame = last_frame(CAN_PARAMETER_ID);
    assert(frame->data[1] == 1u && frame->data[2] == 1u && frame->data[3] == APP_RESULT_OK);
    assert(frame_float(frame) == 1.0f && last_ack()->data[3] == APP_RESULT_OK);
    assert(stats().require_stop); /* Queries do not perform a STOP handshake. */
    poll_frame(parameter_command(CAN_OPCODE_PARAM_WRITE,2u,1u,2.5f));
    assert(submissions[1].kind == APP_CMD_PARAM_WRITE && submissions[1].value == 2.5f);
    complete(APP_RESULT_OK);
    assert(frame_float(last_frame(CAN_PARAMETER_ID)) == 2.5f);
    reject_frame(parameter_command(CAN_OPCODE_PARAM_READ,3u,1u,1.0f),CAN_RESULT_BAD_FRAME);
    assert(last_frame(CAN_PARAMETER_ID)->data[3] == CAN_RESULT_BAD_FRAME);
    assert(frame_float(last_frame(CAN_PARAMETER_ID)) == 2.5f);
    reject_frame(parameter_command(CAN_OPCODE_PARAM_WRITE,3u,10u,1.0f),CAN_RESULT_RANGE);
    reject_frame(parameter_command(CAN_OPCODE_PARAM_WRITE,3u,1u,NAN),CAN_RESULT_RANGE);
    poll_frame(parameter_command(CAN_OPCODE_PARAM_WRITE,3u,1u,2.0f));
    poll_frame(command(CAN_OPCODE_STOP,4u,0,0u));
    assert(last_frame(CAN_PARAMETER_ID)->data[3] == APP_RESULT_CANCELLED);
    complete(APP_RESULT_OK);
    reset();
    app_status.foc.id_a = 0.125f; app_status.foc.iq_reference_a = -0.1f;
    app_status.foc.vd_v = 2.0f; app_status.foc.vq_v = -1.5f;
    app_status.raw_u = 2047u; app_status.raw_w = 2061u;
    app_status.angle_rad = 2.5f; app_status.max_isr_cycles = MOTOR_CPU_HZ/10000u;
    app_status.bus_voltage_v = 14.0f;
    now_ms = 10u; CanProtocol_Poll(); frame = last_frame(CAN_TRACE_ID);
    assert(frame->data[1] == 0u && get_u16(&frame->data[2]) == 125u);
    assert((int16_t)get_u16(&frame->data[4]) == -100 && get_u16(&frame->data[6]) == 2000u);
    now_ms = 20u; CanProtocol_Poll(); frame = last_frame(CAN_TRACE_ID);
    assert(frame->data[1] == 1u && (int16_t)get_u16(&frame->data[2]) == -1500);
    assert(get_u16(&frame->data[4]) == 2047u && get_u16(&frame->data[6]) == 2061u);
    now_ms = 30u; CanProtocol_Poll(); frame = last_frame(CAN_TRACE_ID);
    assert(frame->data[1] == 2u && get_u16(&frame->data[2]) == 2500u);
    assert(get_u16(&frame->data[4]) == 100u && get_u16(&frame->data[6]) == 14000u);
    now_ms = 40u; CanProtocol_Poll(); assert(last_frame(CAN_TRACE_ID)->data[1] == 0u);
}

static void test_reply_priority_with_three_hardware_mailboxes(void)
{
    unsigned first, accepted_count;
    uint32_t accepted_ids[3];
    reset();
    mailbox_budget = 3;
    now_ms = CAN_FEEDBACK_PERIOD_MS; CanProtocol_Poll();
    assert(mailbox_budget == 0); /* status + diagnostic + trace occupied them. */
    poll_frame(parameter_command(CAN_OPCODE_PARAM_READ,1u,1u,0.0f));
    complete(APP_RESULT_OK); /* Execution completed while all mailboxes busy. */
    first = tx_count;
    mailbox_budget = 3; /* Transmission completes; the next poll retries replies. */
    CanProtocol_Poll();
    assert(tx_count == first+2u);
    assert(tx_accepted[first] && tx[first].id == CAN_ACK_ID);
    assert(tx_accepted[first+1u] && tx[first+1u].id == CAN_PARAMETER_ID);
    assert(tx[first].data[2] == 1u && tx[first].data[3] == APP_RESULT_OK);
    reset();
    poll_frame(parameter_command(CAN_OPCODE_PARAM_READ,1u,1u,0.0f));
    mailbox_budget = 3;
    now_ms = CAN_FEEDBACK_PERIOD_MS;
    first = tx_count;
    complete(APP_RESULT_OK); /* Reply and periodic telemetry due in the same poll. */
    accepted_count = 0u;
    for (unsigned n = first; n < tx_count; ++n)
        if (tx_accepted[n]) { assert(accepted_count < 3u); accepted_ids[accepted_count++] = tx[n].id; }
    assert(accepted_count == 3u);
    assert(accepted_ids[0] == CAN_ACK_ID && accepted_ids[1] == CAN_PARAMETER_ID);
    assert(accepted_ids[2] == CAN_STATUS_ID);
    assert(stats().tx_dropped == 1u); /* Diagnostic dropped; latest trace awaits a mailbox. */
    mailbox_budget = 3; now_ms += CAN_FEEDBACK_PERIOD_MS; CanProtocol_Poll();
    assert(tx[tx_count-3u].id == CAN_STATUS_ID && tx_accepted[tx_count-3u]);
    assert(tx[tx_count-2u].id == CAN_DIAGNOSTIC_ID && tx_accepted[tx_count-2u]);
    assert(tx[tx_count-1u].id == CAN_TRACE_ID && tx_accepted[tx_count-1u]);
}

static void test_limits_readback_faults_sequences_and_reply_priority(void)
{
    BoardCanFrame request;
    const BoardCanFrame *limits;
    uint32_t accepted_ids[3];
    unsigned first, accepted;
    const uint32_t limits_id = CAN_PROTOCOL_LIMITS_BASE_ID + CAN_NODE_ID;
    const uint16_t trip_ma = (uint16_t)(MOTOR_OVERCURRENT_TRIP_A * 1000.0f + 0.5f);
    reset();
    app_status.faults = FOC_FAULT_OVERCURRENT;
    app_status.foc.mode = FOC_MODE_FAULT;
    tx_accept = false;
    poll_frame(command(CAN_OPCODE_LIMITS_READ,1u,0,0u));
    assert(submission_count == 0u && stats().require_stop && !stats().request_pending);
    assert(!app_status.driver_enabled && app_status.faults == FOC_FAULT_OVERCURRENT);
    tx_accept = true; mailbox_budget = 3;
    now_ms = CAN_FEEDBACK_PERIOD_MS; first = tx_count;
    CanProtocol_Poll();
    accepted = 0u;
    for (unsigned n = first; n < tx_count; ++n) {
        if (tx_accepted[n]) { assert(accepted < 3u); accepted_ids[accepted++] = tx[n].id; }
    }
    assert(accepted == 3u && accepted_ids[0] == CAN_ACK_ID);
    assert(accepted_ids[1] == limits_id && accepted_ids[2] == CAN_STATUS_ID);
    assert(last_ack()->data[1] == CAN_OPCODE_LIMITS_READ && last_ack()->data[2] == 1u);
    assert(last_ack()->data[3] == APP_RESULT_OK);
    assert(get_u32(&last_ack()->data[4]) == FOC_FAULT_OVERCURRENT);
    limits = last_frame(limits_id);
    assert(limits->dlc == 8u && !limits->extended && !limits->remote);
    assert(limits->data[0] == 0xa1u && limits->data[1] == 0x14u);
    assert(limits->data[2] == 1u && limits->data[3] == APP_RESULT_OK);
    assert(get_u16(&limits->data[4]) == MOTOR_IQ_REQUEST_MAX_MA);
    assert(get_u16(&limits->data[6]) == trip_ma);
    /* Both profiles now report the shared 3.3 A request ceiling (3300 = 0x0CE4). */
    assert(limits->data[4] == 0xe4u && limits->data[5] == 0x0cu);
    assert(get_u16(&limits->data[6]) ==
           (uint16_t)(MOTOR_OVERCURRENT_TRIP_A * 1000.0f + 0.5f));
    mailbox_budget = -1;
    reject_frame(command(CAN_OPCODE_LIMITS_READ,1u,0,0u),CAN_RESULT_SEQUENCE);
    assert(last_frame(limits_id)->data[3] == CAN_RESULT_SEQUENCE);
    for (unsigned byte = 3u; byte < 8u; ++byte) {
        request = command(CAN_OPCODE_LIMITS_READ,2u,0,0u); request.data[byte] = 1u;
        reject_frame(request,CAN_RESULT_BAD_FRAME);
        assert(last_frame(limits_id)->data[3] == CAN_RESULT_BAD_FRAME);
    }
    request = command(CAN_OPCODE_LIMITS_READ,2u,0,0u);
    now_ms += CAN_COMMAND_MAX_AGE_MS + 1u;
    reject_frame(request,CAN_RESULT_STALE);
    assert(last_frame(limits_id)->data[3] == CAN_RESULT_STALE);
    assert(submission_count == 0u && stats().require_stop && !app_status.driver_enabled);
    assert(app_status.faults == FOC_FAULT_OVERCURRENT);
    reset(); configured_node = 7u;
    request = command(CAN_OPCODE_LIMITS_READ,255u,0,0u);
    request.id = CAN_PROTOCOL_COMMAND_BASE_ID + configured_node;
    poll_frame(request);
    limits = last_frame(CAN_PROTOCOL_LIMITS_BASE_ID + 7u);
    assert(limits->data[2] == 255u && limits->data[3] == APP_RESULT_OK);
    assert(last_frame(CAN_PROTOCOL_ACK_BASE_ID + 7u)->data[2] == 255u);
    assert(submission_count == 0u && !app_status.driver_enabled);
}

static BoardCanFrame node_command(uint8_t opcode, uint8_t seq, uint8_t id)
{
    BoardCanFrame frame = command(opcode, seq, 0, 0u);
    frame.id = CAN_PROTOCOL_COMMAND_BASE_ID + configured_node;
    if (opcode == CAN_OPCODE_NODE_SET) {
        frame.data[3] = id;
        frame.data[4] = 0x43u; frame.data[5] = 0x41u;
        frame.data[6] = 0x4eu; frame.data[7] = 0x31u;
    }
    return frame;
}

static void test_node_id_configuration(void)
{
    BoardCanFrame request;
    const BoardCanFrame *info;
    reset();
    poll_frame(node_command(CAN_OPCODE_NODE_READ, 1u, 0u));
    assert(last_ack()->data[3] == APP_RESULT_OK);
    info = last_frame(CAN_PROTOCOL_NODE_INFO_BASE_ID + CAN_NODE_ID);
    assert(info->data[1] == CAN_OPCODE_NODE_READ && info->data[2] == 1u);
    assert(info->data[4] == CAN_NODE_ID && info->data[5] == CAN_NODE_ID && info->data[6] == 1u);
    poll_frame(node_command(CAN_OPCODE_NODE_SET, 2u, 2u));
    assert(last_ack()->data[3] == CAN_RESULT_REQUIRE_STOP && node_commit_count == 0u);
    reset(); handshake(10u);
    request = node_command(CAN_OPCODE_NODE_SET, 11u, 0u); poll_frame(request);
    assert(last_ack()->data[3] == CAN_RESULT_RANGE);
    request = node_command(CAN_OPCODE_NODE_SET, 11u, 64u); poll_frame(request);
    assert(last_ack()->data[3] == CAN_RESULT_RANGE);
    request = node_command(CAN_OPCODE_NODE_SET, 11u, 2u); request.data[7] ^= 1u; poll_frame(request);
    assert(last_ack()->data[3] == CAN_RESULT_BAD_FRAME);
    request = node_command(CAN_OPCODE_NODE_SET, 10u, 2u); poll_frame(request);
    assert(last_ack()->data[3] == CAN_RESULT_SEQUENCE);
    request = node_command(CAN_OPCODE_NODE_SET, 11u, 2u);
    now_ms += CAN_COMMAND_MAX_AGE_MS + 1u; poll_frame(request);
    assert(last_ack()->data[3] == CAN_RESULT_STALE);
    assert(node_commit_count == 0u);
    reset(); handshake(10u);
    app_status.driver_enabled = true; app_status.foc.mode = FOC_MODE_CURRENT;
    poll_frame(node_command(CAN_OPCODE_NODE_SET, 11u, 2u));
    assert(last_ack()->data[3] == APP_RESULT_NEED_STOP && node_commit_count == 0u);
    reset(); handshake(10u);
    app_status.faults = APP_FAULT_ESTOP;
    poll_frame(node_command(CAN_OPCODE_NODE_SET, 11u, 2u));
    assert(last_ack()->data[3] == APP_RESULT_FAULT && node_commit_count == 0u);
    reset(); handshake(10u);
    epoch_on_critical = true;
    poll_frame(node_command(CAN_OPCODE_NODE_SET, 11u, 2u));
    assert(last_ack()->data[3] == CAN_RESULT_REQUIRE_STOP && node_commit_count == 0u);
    reset(); handshake(10u);
    node_flags = BOARD_NODE_STORAGE_ERROR;
    poll_frame(command(CAN_OPCODE_OPEN, 11u, 2000, 800u));
    assert(last_ack()->data[3] == CAN_RESULT_STORAGE && node_commit_count == 0u);
    reset(); handshake(10u);
    poll_frame(node_command(CAN_OPCODE_NODE_SET, 11u, 63u));
    assert(node_commit_count == 1u && configured_node == 63u);
    assert(node_commit_ack.id == CAN_ACK_ID && node_commit_info.id == CAN_PROTOCOL_NODE_INFO_BASE_ID + CAN_NODE_ID);
    assert(node_commit_ack.data[1] == CAN_OPCODE_NODE_SET && node_commit_ack.data[2] == 11u);
    assert(node_commit_info.data[1] == CAN_OPCODE_NODE_SET && node_commit_info.data[4] == CAN_NODE_ID);
    CanProtocol_Init();
    assert(stats().require_stop);
    now_ms += CAN_FEEDBACK_PERIOD_MS; CanProtocol_Poll();
    assert(last_frame(CAN_PROTOCOL_STATE_BASE_ID + 63u)->data[0] == CAN_PROTOCOL_MAGIC);
    request = command(CAN_OPCODE_STOP, 20u, 0, 0u); /* Old ID must no longer accept. */
    unsigned before = submission_count; poll_frame(request); assert(submission_count == before);
    request.id = CAN_PROTOCOL_COMMAND_BASE_ID + 63u; poll_frame(request);
    assert(submission_count == before + 1u);
    complete(APP_RESULT_OK);
    assert(!stats().require_stop && last_frame(CAN_PROTOCOL_ACK_BASE_ID + 63u)->data[3] == APP_RESULT_OK);
}

static BoardCanFrame calibration_command(uint8_t opcode, uint8_t seq)
{
    BoardCanFrame frame = command(opcode, seq, 0, 0u);
    if (opcode == CAN_OPCODE_CALIBRATION_SAVE) {
        frame.data[4] = 0x43u; frame.data[5] = 0x41u;
        frame.data[6] = 0x4cu; frame.data[7] = 0x31u;
    } else if (opcode == CAN_OPCODE_CALIBRATION_CLEAR) {
        frame.data[4] = 0x43u; frame.data[5] = 0x4cu;
        frame.data[6] = 0x52u; frame.data[7] = 0x31u;
    }
    return frame;
}

static void test_persistent_calibration_protocol(void)
{
    BoardCanFrame request;
    const BoardCanFrame *info;
    const uint32_t info_id = CAN_PROTOCOL_CALIBRATION_BASE_ID + CAN_NODE_ID;
    reset();
    calibration_info.valid = true;
    calibration_info.motor_model = MOTOR_MODEL;
    calibration_info.pole_pairs = MOTOR_POLE_PAIRS;
    calibration_info.sensor_direction = -1;
    calibration_info.electrical_zero_rad = 1.234f;
    app_status.zero_valid = true;
    poll_frame(calibration_command(CAN_OPCODE_CALIBRATION_READ, 1u));
    assert(last_ack()->data[3] == APP_RESULT_OK);
    info = last_frame(info_id);
    assert(info->data[1] == CAN_OPCODE_CALIBRATION_READ && info->data[2] == 1u);
    assert((info->data[4] & (CAN_CALIBRATION_FLAG_STORED |
        CAN_CALIBRATION_FLAG_RUNTIME_VALID)) ==
        (CAN_CALIBRATION_FLAG_STORED | CAN_CALIBRATION_FLAG_RUNTIME_VALID));
    assert((int8_t)info->data[5] == -1 && get_u16(&info->data[6]) == 1234u);

    request = calibration_command(CAN_OPCODE_CALIBRATION_READ, 2u);
    request.data[7] = 1u; reject_frame(request, CAN_RESULT_BAD_FRAME);
    assert(last_frame(info_id)->data[3] == CAN_RESULT_BAD_FRAME);
    request = calibration_command(CAN_OPCODE_CALIBRATION_SAVE, 2u);
    request.data[7] ^= 1u; reject_frame(request, CAN_RESULT_BAD_FRAME);
    assert(calibration_commit_count == 0u);

    reset(); handshake(10u);
    app_status.offset_ready = app_status.sensor_valid = true;
    poll_frame(calibration_command(CAN_OPCODE_CALIBRATION_SAVE, 11u));
    assert(last_ack()->data[3] == APP_RESULT_NEED_ZERO);
    assert(calibration_commit_count == 0u);
    app_status.direction_confirmed = app_status.zero_valid = true;
    app_status.sensor_direction = -1;
    app_status.electrical_zero_rad = 2.5f;
    poll_frame(calibration_command(CAN_OPCODE_CALIBRATION_SAVE, 12u));
    assert(calibration_commit_count == 1u && !calibration_commit_clear);
    assert(calibration_commit_model == MOTOR_MODEL && calibration_commit_poles == MOTOR_POLE_PAIRS);
    assert(calibration_commit_direction == -1 && calibration_commit_zero == 2.5f);
    assert(calibration_commit_ack.id == CAN_ACK_ID && calibration_commit_info.id == info_id);
    assert(calibration_commit_ack.data[1] == CAN_OPCODE_CALIBRATION_SAVE);

    reset(); handshake(20u);
    poll_frame(calibration_command(CAN_OPCODE_CALIBRATION_CLEAR, 21u));
    assert(calibration_commit_count == 1u && calibration_commit_clear);
    assert(calibration_commit_ack.data[1] == CAN_OPCODE_CALIBRATION_CLEAR);
}

static unsigned frame_count(uint32_t id)
{
    unsigned total = 0u;
    for (unsigned n = 0u; n < tx_count; ++n)
        if (tx_accepted[n] && tx[n].id == id) ++total;
    return total;
}

static void test_fast_feedback_timing_units_and_validity(void)
{
    const BoardCanFrame *frame;
    reset();
    app_status.driver_enabled = app_status.sense_confirmed = app_status.offset_ready = true;
    app_status.sensor_valid = app_status.direction_confirmed = app_status.zero_valid = true;
    app_status.command_timed_out = true; app_status.control_source = APP_SOURCE_CAN;
    app_status.foc.mode = FOC_MODE_CURRENT;
    app_status.angle_rad = 1.25f; app_status.velocity_rad_s = -2.5f; app_status.foc.iq_a = 0.25f;
    now_us = CAN_FAST_FEEDBACK_PERIOD_US - 1u; CanProtocol_Poll(); assert(tx_count == 0u);
    ++now_us; CanProtocol_Poll();
    frame = last_frame(CAN_FAST_ID);
    assert(frame->dlc == 8u && frame->data[0] == CAN_PROTOCOL_MAGIC && frame->data[1] == 0xffu);
    assert(get_u16(&frame->data[2]) == 1250u);
    assert((int16_t)get_u16(&frame->data[4]) == -2500 && get_u16(&frame->data[6]) == 250u);
    now_us = CAN_FAST_FEEDBACK_PERIOD_US * 2u - 1u; CanProtocol_Poll();
    assert(frame_count(CAN_FAST_ID) == 1u);
    ++now_us; CanProtocol_Poll(); assert(frame_count(CAN_FAST_ID) == 2u);
    app_status.sensor_valid = false; now_us += CAN_FAST_FEEDBACK_PERIOD_US; CanProtocol_Poll();
    frame = last_frame(CAN_FAST_ID);
    assert((frame->data[1] & (1u << 3)) == 0u && get_u16(&frame->data[2]) == 0u);
    app_status.sensor_valid = true; app_status.angle_rad = NAN;
    now_us += CAN_FAST_FEEDBACK_PERIOD_US; CanProtocol_Poll(); frame = last_frame(CAN_FAST_ID);
    assert((frame->data[1] & (1u << 3)) == 0u && get_u16(&frame->data[2]) == 0u);
    app_status.angle_rad = FOC_TWO_PI_F;
    now_us += CAN_FAST_FEEDBACK_PERIOD_US; CanProtocol_Poll();
    assert(get_u16(&last_frame(CAN_FAST_ID)->data[2]) == 6283u);
    reset(); now_us = 0xffffff00u; CanProtocol_Init();
    now_us += CAN_FAST_FEEDBACK_PERIOD_US - 1u; CanProtocol_Poll(); assert(tx_count == 0u);
    ++now_us; CanProtocol_Poll(); assert(frame_count(CAN_FAST_ID) == 1u);
    const uint8_t nodes[] = {1u,2u,3u,5u,6u,7u};
    for (unsigned n = 0u; n < sizeof(nodes); ++n) {
        reset(); configured_node = nodes[n]; CanProtocol_Init();
        now_us = CAN_FAST_FEEDBACK_PERIOD_US; CanProtocol_Poll();
        assert(frame_count(CAN_PROTOCOL_FAST_BASE_ID + nodes[n]) == 1u);
    }
}

static void test_fast_ack_priority_latest_value_and_trace_retry(void)
{
    unsigned first, accepted;
    uint32_t order[3];
    reset();
    poll_frame(parameter_command(CAN_OPCODE_PARAM_READ,1u,1u,0.0f));
    mailbox_budget = 3; now_ms = 10u; now_us = 10000u; first = tx_count;
    complete(APP_RESULT_OK);
    accepted = 0u;
    for (unsigned n=first; n<tx_count; ++n)
        if (tx_accepted[n]) { assert(accepted < 3u); order[accepted++] = tx[n].id; }
    assert(accepted == 3u && order[0] == CAN_ACK_ID && order[1] == CAN_PARAMETER_ID && order[2] == CAN_FAST_ID);
    assert(frame_count(CAN_TRACE_ID) == 0u);
    mailbox_budget = 3; ++now_us; CanProtocol_Poll();
    assert(frame_count(CAN_TRACE_ID) == 1u); /* No permanent trace starvation. */
    reset(); mailbox_budget = 0;
    app_status.sensor_valid = true; app_status.angle_rad = 1.0f;
    now_us = 5000u; CanProtocol_Poll();
    app_status.angle_rad = 2.0f; now_us = 10000u; CanProtocol_Poll();
    assert(frame_count(CAN_FAST_ID) == 0u && stats().tx_dropped == 1u);
    mailbox_budget = 1; ++now_us; CanProtocol_Poll();
    assert(frame_count(CAN_FAST_ID) == 1u && get_u16(&last_frame(CAN_FAST_ID)->data[2]) == 2000u);
    reset(); mailbox_budget = 0; now_us = 5000u; CanProtocol_Poll();
    transport.bus_off = true; ++transport.bus_off_count; CanProtocol_Poll();
    transport.bus_off = false; mailbox_budget = 3; ++now_us; CanProtocol_Poll();
    assert(frame_count(CAN_FAST_ID) == 0u); /* Old pending snapshot discarded. */
    now_us = 10000u; CanProtocol_Poll(); assert(frame_count(CAN_FAST_ID) == 1u);
}

int main(void)
{
    test_framing_and_units();
    test_sequence_and_ack();
    test_safety_barriers_and_epoch();
    test_time_and_transport();
    test_telemetry_and_bounds();
    test_revoked_pending_results();
    test_parameter_framing_results_and_trace();
    test_reply_priority_with_three_hardware_mailboxes();
    test_limits_readback_faults_sequences_and_reply_priority();
    test_node_id_configuration();
    test_persistent_calibration_protocol();
    test_fast_feedback_timing_units_and_validity();
    test_fast_ack_priority_latest_value_and_trace_retry();
    puts("All CAN protocol host tests passed.");
    return 0;
}

