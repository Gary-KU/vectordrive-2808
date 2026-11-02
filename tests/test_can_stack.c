/* Compose real protocol + real App + real FOC over the existing fake board. */
#define main app_suite_main
#include "test_app.c"
#undef main
#include "board_can.h"
#include "can_protocol.h"
#include "can_service.h"

static BoardCanStatus transport;
static BoardCanFrame received[32], sent[4096];
static unsigned received_head, received_tail, sent_count;
static unsigned init_calls;
static bool init_success;

uint8_t BoardCan_GetNodeId(void) { return CAN_NODE_ID; }
BoardCanNodeInfo BoardCan_GetNodeInfo(void)
{
    return (BoardCanNodeInfo){CAN_NODE_ID, CAN_NODE_ID, 0u};
}
void BoardCan_CommitNodeIdAndReset(uint8_t id, BoardCanFrame ack, BoardCanFrame info)
{
    (void)id; (void)ack; (void)info;
    CHECK(false); /* Node save/reset is covered by dedicated protocol/store tests. */
}
void BoardCalibration_CommitAndReset(bool clear, uint16_t motor_model,
    uint8_t pole_pairs, int8_t sensor_direction, float electrical_zero_rad,
    BoardCanFrame ack, BoardCanFrame info)
{
    (void)clear; (void)motor_model; (void)pole_pairs; (void)sensor_direction;
    (void)electrical_zero_rad; (void)ack; (void)info;
    CHECK(false); /* Covered by protocol/store unit tests. */
}

bool BoardCan_Init(void)
{
    ++init_calls;
    memset(&transport, 0, sizeof(transport));
    transport.initialized = init_success;
    received_head = received_tail = 0u;
    return init_success;
}
bool BoardCan_Read(BoardCanFrame *frame)
{
    if (received_tail == received_head) return false;
    *frame = received[received_tail++ % 32u]; return true;
}
bool BoardCan_Send(const BoardCanFrame *frame)
{
    if (!transport.initialized || transport.bus_off) {
        ++transport.tx_dropped;
        return false;
    }
    CHECK(sent_count < 4096u); sent[sent_count++] = *frame; return true;
}
void BoardCan_DiscardRx(void) { received_tail = received_head; }
BoardCanStatus BoardCan_GetStatus(void) { return transport; }

static void stack_boot(void)
{
    ready(true);
    memset(&transport, 0, sizeof(transport));
    transport.initialized = true;
    received_head = received_tail = sent_count = 0u;
    init_calls = 0u; init_success = true;
    CanProtocol_Init();
}

static void push(uint8_t opcode, uint8_t sequence_number, int16_t value, uint16_t auxiliary)
{
    BoardCanFrame f;
    memset(&f, 0, sizeof(f));
    f.id = CAN_COMMAND_ID; f.dlc = 8u; f.received_ms = Board_Millis();
    f.data[0] = CAN_PROTOCOL_MAGIC; f.data[1] = opcode; f.data[2] = sequence_number;
    if (opcode == CAN_OPCODE_IQ || opcode == CAN_OPCODE_SPEED ||
        opcode == CAN_OPCODE_OPEN || opcode == CAN_OPCODE_ALIGN) f.data[3] = 1u;
    f.data[4] = (uint8_t)(uint16_t)value; f.data[5] = (uint8_t)((uint16_t)value >> 8);
    f.data[6] = (uint8_t)auxiliary; f.data[7] = (uint8_t)(auxiliary >> 8);
    CHECK(received_head - received_tail < 32u);
    received[received_head++ % 32u] = f;
}

static void stack_step(unsigned milliseconds)
{
    unsigned count = milliseconds * (MOTOR_ADC_HZ / 1000u);
    while (count-- != 0u) { sample(2048u,2048u); CanProtocol_Poll(); }
}

static int last_ack(uint8_t sequence_number)
{
    for (unsigned n = sent_count; n != 0u; --n)
        if (sent[n-1u].id == CAN_ACK_ID && sent[n-1u].data[2] == sequence_number)
            return sent[n-1u].data[3];
    return -1;
}

static void request(uint8_t opcode, uint8_t sequence_number, int16_t value, uint16_t auxiliary)
{
    push(opcode,sequence_number,value,auxiliary); CanProtocol_Poll(); stack_step(1u);
}

static void handshake(uint8_t seq)
{
    request(CAN_OPCODE_STOP,seq,0,0u);
    CHECK(last_ack(seq) == APP_RESULT_OK);
    { CanProtocolStats s; CanProtocol_GetStats(&s); CHECK(!s.require_stop); }
}

static void test_runtime_parameter_stack(void)
{
    BoardCanFrame *frame;
    float readback, value = 0.75f;
    stack_boot();
    push(CAN_OPCODE_PARAM_WRITE,1u,0,0u);
    frame = &received[(received_head-1u)%32u];
    frame->data[3] = APP_PARAM_CURRENT_KP;
    memcpy(&frame->data[4],&value,sizeof(value));
    CanProtocol_Poll();
    CHECK(last_ack(1u) == -1);
    CHECK(state().parameters[APP_PARAM_CURRENT_KP-1u] == MOTOR_CURRENT_KP);
    stack_step(1u);
    CHECK(last_ack(1u) == APP_RESULT_OK);
    CHECK(state().parameters[APP_PARAM_CURRENT_KP-1u] == value);
    CHECK(!driver && state().control_source == APP_SOURCE_NONE);
    bool found = false;
    for (unsigned n = 0u; n < sent_count; ++n) {
        if (sent[n].id == CAN_PARAMETER_ID && sent[n].data[2] == 1u) {
            memcpy(&readback,&sent[n].data[4],sizeof(readback));
            CHECK(sent[n].data[3] == APP_RESULT_OK && readback == value);
            found = true;
        }
    }
    CHECK(found);
    handshake(2u);
    request(CAN_OPCODE_OPEN,3u,2000,800u); CHECK(driver);
    push(CAN_OPCODE_PARAM_WRITE,4u,0,0u);
    frame = &received[(received_head-1u)%32u]; frame->data[3] = APP_PARAM_CURRENT_KP;
    value = 1.5f; memcpy(&frame->data[4],&value,sizeof(value));
    CanProtocol_Poll(); stack_step(1u);
    CHECK(last_ack(4u) == APP_RESULT_NEED_STOP);
    CHECK(state().parameters[APP_PARAM_CURRENT_KP-1u] == 0.75f);
}

static void test_profile_current_request_and_fixed_measured_trip(void)
{
    float requested = 3.3f;
    uint8_t seq = 2u;
    BoardCanFrame *frame;
    stack_boot();
    handshake(1u); /* Cancels a dedicated startup test without starting motion. */
    CHECK(state().parameters[APP_PARAM_IQ_LIMIT-1u] == MOTOR_CURRENT_LIMIT_A);
    CHECK(!driver && state().foc.mode == FOC_MODE_STOP);
    push(CAN_OPCODE_PARAM_WRITE,seq,0,0u);
    frame = &received[(received_head-1u)%32u];
    frame->data[3] = APP_PARAM_IQ_LIMIT;
    memcpy(&frame->data[4],&requested,sizeof(requested));
    CanProtocol_Poll(); stack_step(1u);
    CHECK(last_ack(seq) == APP_RESULT_OK);
    CHECK(state().parameters[APP_PARAM_IQ_LIMIT-1u] == requested && !driver);
    push(CAN_OPCODE_PARAM_READ,++seq,0,0u);
    frame = &received[(received_head-1u)%32u]; frame->data[3] = APP_PARAM_IQ_LIMIT;
    CanProtocol_Poll(); stack_step(1u);
    CHECK(last_ack(seq) == APP_RESULT_OK && state().can_parameter_value == requested);
    align_motor(1);
    handshake(++seq);
    request(CAN_OPCODE_IQ,++seq,(int16_t)MOTOR_IQ_REQUEST_MAX_MA,0u);
#if MOTOR_CURRENT_SENSE_CONFIRMED
    CHECK(last_ack(seq) == APP_RESULT_OK && driver);
    CHECK(state().foc.iq_reference_a > 0.0f && state().foc.iq_reference_a < 0.01f);
    /* Fake ADC stays at zero current. Renew valid commands so CAN's 100 ms
     * deadline does not hide whether real App/FOC keeps the 3.3 A request. */
    for (unsigned n = 0u; n < 68u; ++n) {
        stack_step(49u);
        request(CAN_OPCODE_IQ,++seq,(int16_t)MOTOR_IQ_REQUEST_MAX_MA,0u);
        CHECK(last_ack(seq) == APP_RESULT_OK && driver && state().faults == 0u);
    }
    CHECK(fabsf(state().foc.iq_reference_a - requested) < 0.00001f);
    request(CAN_OPCODE_IQ,++seq,(int16_t)(MOTOR_IQ_REQUEST_MAX_MA+1u),0u);
    CHECK(last_ack(seq) == CAN_RESULT_RANGE && driver);
    /* A measured U phase just above the independent protection threshold
     * trips despite the larger configured/requested synthetic reference. */
    const float current_per_count = MOTOR_ADC_REFERENCE_V /
        (MOTOR_ADC_FULL_SCALE * MOTOR_SHUNT_U_OHM * MOTOR_SENSE_GAIN_U);
    sample((uint16_t)(2048u + ceilf(1.01f * MOTOR_OVERCURRENT_TRIP_A /
                                   current_per_count)),2048u);
    CanProtocol_Poll();
    CHECK((state().faults & FOC_FAULT_OVERCURRENT) != 0u && !driver);
    request(CAN_OPCODE_LIMITS_READ,++seq,0,0u);
    CHECK(last_ack(seq) == APP_RESULT_OK && !driver);
    bool found = false;
    for (unsigned n = 0u; n < sent_count; ++n) {
        if (sent[n].id == CAN_PROTOCOL_LIMITS_BASE_ID + CAN_NODE_ID && sent[n].data[2] == seq) {
            CHECK(sent[n].data[0] == 0xa1u && sent[n].data[1] == CAN_OPCODE_LIMITS_READ);
            CHECK(sent[n].data[3] == APP_RESULT_OK);
            CHECK(sent[n].data[4] == 0xe4u && sent[n].data[5] == 0x0cu);
            const uint16_t trip_ma = (uint16_t)(MOTOR_OVERCURRENT_TRIP_A * 1000.0f + 0.5f);
            CHECK(sent[n].data[6] == (uint8_t)trip_ma &&
                  sent[n].data[7] == (uint8_t)(trip_ma >> 8));
            found = true;
        }
    }
    CHECK(found && (state().faults & FOC_FAULT_OVERCURRENT) != 0u);
#else
    CHECK(last_ack(seq) == APP_RESULT_SENSE_UNKNOWN && !driver);
#endif
}

static void test_limits_query_cannot_renew_motion_deadline(void)
{
    uint8_t seq = 1u;
    stack_boot(); handshake(seq);
    request(CAN_OPCODE_OPEN,++seq,2000,800u);
    CHECK(last_ack(seq) == APP_RESULT_OK && driver);
    for (unsigned n = 0u; n < 12u; ++n) {
        stack_step(10u);
        request(CAN_OPCODE_LIMITS_READ,++seq,0,0u);
        CHECK(last_ack(seq) == APP_RESULT_OK);
    }
    CHECK(!driver && state().command_timed_out && state().control_source == APP_SOURCE_NONE);
    CHECK(state().faults == 0u);
}

static void recovery_boot(void)
{
    stack_boot();
    App_CancelStartupMotion();
    transport.initialized = false;
    init_success = false;
    CanService_Init();
    CHECK(init_calls == 1u && !transport.initialized && !driver);
}

static void test_failed_init_retries_and_requires_stop(void)
{
    CanProtocolStats stats;
    CHECK(init_calls == 0u);
    CanService_Poll(); /* Before initialization: no side effects. */
    CHECK(init_calls == 0u);
    recovery_boot();
    CanService_Poll();
    stack_step(CAN_INIT_RETRY_PERIOD_MS - 1u); CanService_Poll();
    CHECK(init_calls == 1u);
    stack_step(1u); CanService_Poll();
    CHECK(init_calls == 2u && !transport.initialized);
    CanService_Poll(); CHECK(init_calls == 2u); /* No busy retry loop. */
    init_success = true;
    stack_step(CAN_INIT_RETRY_PERIOD_MS - 1u); CanService_Poll();
    CHECK(init_calls == 2u);
    stack_step(1u);
    push(CAN_OPCODE_OPEN, 99u, 2000, 800u); /* Stale traffic at recovery. */
    CanService_Poll();
    CHECK(init_calls == 3u && transport.initialized && !driver && enable_count == 0u);
    CHECK(received_head == received_tail);
    CanProtocol_GetStats(&stats); CHECK(stats.require_stop && !stats.request_pending);
    request(CAN_OPCODE_OPEN, 1u, 2000, 800u);
    CHECK(last_ack(1u) == CAN_RESULT_REQUIRE_STOP && !driver);
    handshake(2u);
    request(CAN_OPCODE_OPEN, 3u, 2000, 800u);
    CHECK(last_ack(3u) == APP_RESULT_OK && driver);
}

static void test_retry_waits_for_queued_and_running_local_motion(void)
{
    recovery_boot();
    stack_step(CAN_INIT_RETRY_PERIOD_MS);
    init_success = true;
    CHECK(App_Submit(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_PENDING);
    CHECK(!App_CanReinitializeCan());
    CanService_Poll(); CHECK(init_calls == 1u && !driver);
    stack_step(1u); CHECK(driver);
    CanService_Poll(); CHECK(init_calls == 1u);
    CHECK(App_Submit(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_PENDING);
    CanService_Poll(); CHECK(init_calls == 1u); /* STOP still awaits ISR execution. */
    stack_step(1u); CHECK(!driver && App_CanReinitializeCan());
    CanService_Poll(); CHECK(init_calls == 2u && transport.initialized && !driver);
}

static void test_recovery_preserves_fault_and_driver_disable(void)
{
    CanProtocolStats stats;
    recovery_boot();
    CHECK(App_Submit(APP_CMD_ESTOP, 0.0f, 0.0f) == APP_RESULT_PENDING);
    stack_step(CAN_INIT_RETRY_PERIOD_MS);
    CHECK((state().faults & APP_FAULT_ESTOP) != 0u && !driver);
    init_success = true;
    CanService_Poll();
    CHECK(init_calls == 2u && transport.initialized && !driver && enable_count == 0u);
    CHECK((state().faults & APP_FAULT_ESTOP) != 0u);
    CanProtocol_GetStats(&stats); CHECK(stats.require_stop);
}

static void test_retry_interval_survives_millis_wrap(void)
{
    stack_boot(); App_CancelStartupMotion();
    fake_us = ((uint64_t)UINT32_MAX - 250u) * 1000u;
    transport.initialized = false; init_success = false;
    CanService_Init(); CHECK(init_calls == 1u);
    fake_us += (uint64_t)(CAN_INIT_RETRY_PERIOD_MS - 1u) * 1000u;
    CanService_Poll(); CHECK(init_calls == 1u);
    fake_us += 1000u;
    CanService_Poll(); CHECK(init_calls == 2u);
}

static void test_initialized_and_bus_off_peripheral_are_not_reset(void)
{
    stack_boot(); App_CancelStartupMotion(); CanService_Init();
    CHECK(init_calls == 1u && transport.initialized);
    stack_step(CAN_INIT_RETRY_PERIOD_MS * 2u); CanService_Poll();
    CHECK(init_calls == 1u);
    transport.bus_off = true; ++transport.bus_off_count;
    CanService_Poll(); CHECK(init_calls == 1u && !driver);
}

#if MOTOR_OPEN_AUTOTEST
static void test_scheduled_startup_motion_blocks_retry(void)
{
    stack_boot(); /* The power-on test is still scheduled. */
    transport.initialized = false; init_success = false;
    CanService_Init();
    stack_step(CAN_INIT_RETRY_PERIOD_MS);
    init_success = true;
    CanService_Poll(); CHECK(init_calls == 1u && !driver);
    App_CancelStartupMotion();
    CanService_Poll(); CHECK(init_calls == 2u && !driver);
}
#endif

int main(void)
{
    uint8_t seq;
    CanProtocolStats protocol_status;
    test_failed_init_retries_and_requires_stop();
    test_retry_waits_for_queued_and_running_local_motion();
    test_recovery_preserves_fault_and_driver_disable();
    test_retry_interval_survives_millis_wrap();
    test_initialized_and_bus_off_peripheral_are_not_reset();
#if MOTOR_OPEN_AUTOTEST
    test_scheduled_startup_motion_blocks_retry();
#endif
    test_runtime_parameter_stack();
    test_profile_current_request_and_fixed_measured_trip();
    test_limits_query_cannot_renew_motion_deadline();
    stack_boot();
    request(CAN_OPCODE_OPEN,1u,2000,800u);
    CHECK(!driver && last_ack(1u) == CAN_RESULT_REQUIRE_STOP);
    push(CAN_OPCODE_STOP,200u,0,0u); CanProtocol_Poll();
    CHECK(last_ack(200u) == -1); /* Actual ISR execution must precede the ACK. */
    stack_step(1u); CHECK(last_ack(200u) == APP_RESULT_OK);
    request(CAN_OPCODE_DIRECTION,201u,1,0u); CHECK(last_ack(201u) == APP_RESULT_OK);
    stack_step(5u);
    seq = 202u;
    request(CAN_OPCODE_ALIGN,seq++,0,0u);
    for (unsigned n = 0u; n < MOTOR_ALIGN_DURATION_MS / 50u + 1u; ++n) {
        stack_step(50u); request(CAN_OPCODE_ALIGN,seq++,0,0u);
    }
    CHECK(state().zero_valid && !driver && state().faults == 0u);
    request(CAN_OPCODE_IQ,seq,50,0u);
#if MOTOR_CURRENT_SENSE_CONFIRMED
    CHECK(last_ack(seq) == APP_RESULT_OK && driver);
    CHECK(state().foc.mode == FOC_MODE_CURRENT && state().control_source == APP_SOURCE_CAN);
    for (unsigned n = 0u; n < 12u; ++n) {
        stack_step(10u); request(CAN_OPCODE_IQ,++seq,50,0u);
        CHECK(driver);
    }
    CHECK(state().foc.iq_reference_a <= 0.050001f && state().foc.iq_reference_a > 0.049f);
    /* Duplicates arrive continuously, but cannot keep torque alive. */
    for (unsigned n = 0u; n < 11u; ++n) {
        stack_step(10u); request(CAN_OPCODE_IQ,seq,50,0u);
    }
    CHECK(!driver && state().command_timed_out);
    CanProtocol_GetStats(&protocol_status); CHECK(protocol_status.require_stop);
    request(CAN_OPCODE_IQ,++seq,50,0u); CHECK(!driver && last_ack(seq) == CAN_RESULT_REQUIRE_STOP);
    handshake(++seq);
    request(CAN_OPCODE_IQ,++seq,50,0u); CHECK(driver && last_ack(seq) == APP_RESULT_OK);
    transport.bus_off = true; ++transport.bus_off_count;
    CanProtocol_Poll(); CHECK(!driver);
    stack_step(1u); CHECK((state().faults & APP_FAULT_CAN) != 0u && !state().zero_valid);
#else
    CHECK(last_ack(seq) == APP_RESULT_SENSE_UNKNOWN && !driver);
#endif
    /* Old STOP+OPEN already buffered before a local STOP cannot re-arm CAN. */
    stack_boot(); handshake(10u);
    request(CAN_OPCODE_OPEN,11u,2000,800u); CHECK(driver);
    push(CAN_OPCODE_STOP,12u,0,0u); push(CAN_OPCODE_OPEN,13u,2000,800u);
    CHECK(App_Submit(APP_CMD_STOP,0.0f,0.0f) == APP_RESULT_PENDING);
    CanProtocol_Poll(); stack_step(2u);
    CHECK(!driver && received_tail == received_head);
    CanProtocol_GetStats(&protocol_status); CHECK(protocol_status.require_stop);
    handshake(20u); request(CAN_OPCODE_OPEN,21u,2000,800u); CHECK(driver);
#if MOTOR_OPEN_AUTOTEST
    /* A valid CAN operator request takes priority over startup even if the
     * protocol rejects it before App_SubmitCan for lacking STOP handshake. */
    stack_boot();
    request(CAN_OPCODE_IQ,1u,50,0u);
    CHECK(last_ack(1u) == CAN_RESULT_REQUIRE_STOP);
    stack_step(MOTOR_OPEN_AUTOTEST_DELAY_MS + MOTOR_COMMAND_TIMEOUT_MS + 10u);
    CHECK(!driver && enable_count == 0u);
    stack_boot(); handshake(40u);
    push(CAN_OPCODE_NODE_SET,41u,0,0u);
    BoardCanFrame *node_request = &received[(received_head-1u)%32u];
    node_request->data[3] = 2u;
    node_request->data[4] = 0x43u; node_request->data[5] = 0x41u;
    node_request->data[6] = 0x4eu; node_request->data[7] = 0x31u;
    CanProtocol_Poll();
    CHECK(last_ack(41u) == CAN_RESULT_UNSUPPORTED && !driver);
#endif
    puts("Real CAN protocol -> App -> FOC integration passed.");
    return EXIT_SUCCESS;
}
