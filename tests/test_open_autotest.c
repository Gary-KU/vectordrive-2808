/* Reuse the production-app fake BSP without duplicating any hardware stubs.
 * Compile this TU twice, with -DMOTOR_OPEN_AUTOTEST=0 and =1. */
#define main app_suite_main
#include "test_app.c"
#undef main

#ifndef MOTOR_OPEN_AUTOTEST
#error MOTOR_OPEN_AUTOTEST must be defined by motor_config.h
#endif
#ifndef MOTOR_OPEN_AUTOTEST_DELAY_MS
#error MOTOR_OPEN_AUTOTEST_DELAY_MS must be defined by motor_config.h
#endif

static void autotest_ms(uint32_t milliseconds)
{
    steady((unsigned)(milliseconds * (MOTOR_ADC_HZ / 1000u)));
}

static void autotest_never_enabled(void)
{
    CHECK(!driver && enable_count == 0u);
    CHECK(state().foc.mode != FOC_MODE_OPEN_LOOP);
    CHECK(state().foc.mode != FOC_MODE_ALIGNMENT);
    CHECK(state().foc.mode != FOC_MODE_CURRENT);
    CHECK(state().foc.mode != FOC_MODE_SPEED);
}

static void test_autotest_nominal_one_shot(void)
{
    unsigned stopped_enable_count;
    boot(false); /* AS5600 may be entirely absent for the first open-loop test. */
    steady(MOTOR_OPEN_AUTOTEST_DELAY_MS * (MOTOR_ADC_HZ / 1000u) - 1u);
    CHECK(state().offset_ready && !state().sensor_valid);
    autotest_never_enabled(); /* Includes any transient EN pulse via enable_count. */
    steady(MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
#if MOTOR_OPEN_AUTOTEST
    CHECK(driver && state().foc.mode == FOC_MODE_OPEN_LOOP);
    CHECK(state().control_source == APP_SOURCE_LOCAL && state().faults == 0u);
    CHECK(!state().sensor_valid && !state().zero_valid);
    CHECK(state().foc.vd_v == 0.0f);
    CHECK(state().foc.vq_v > 0.0f && state().foc.vq_v < 0.01f);
    autotest_ms(400u);
    CHECK(driver && state().foc.vq_v > 0.39f && state().foc.vq_v < 0.42f);
    CHECK(state().foc.open_velocity_rad_s > 7.9f && state().foc.open_velocity_rad_s < 8.3f);
    autotest_ms(700u);
    CHECK(driver && state().foc.mode == FOC_MODE_OPEN_LOOP);
    CHECK(fabsf(state().foc.vq_v - 0.8f) < 0.0001f);
    CHECK(fabsf(state().foc.open_velocity_rad_s - 20.0f) < 0.0001f);
    autotest_ms(MOTOR_COMMAND_TIMEOUT_MS - 1101u);
    CHECK(driver && !state().command_timed_out); /* Still before the 4 s stop point. */
    autotest_ms(2u);
    CHECK(!driver && state().foc.mode == FOC_MODE_STOP);
    CHECK(state().command_timed_out && state().faults == 0u);
    CHECK(state().control_source == APP_SOURCE_NONE);
#else
    autotest_never_enabled();
    autotest_ms(MOTOR_COMMAND_TIMEOUT_MS + 1u);
    autotest_never_enabled();
    CHECK(!state().command_timed_out);
#endif
    stopped_enable_count = enable_count;
    autotest_ms(10000u);
    CHECK(!driver && enable_count == stopped_enable_count);
    CHECK(state().foc.mode == FOC_MODE_STOP);
}

static void test_autotest_stop_and_commands_cancel(void)
{
    ready(false);
    CHECK(command(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_OK);
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    autotest_never_enabled();

    ready(false);
    CHECK(can_command(APP_CMD_STOP, 0.0f, 0.0f, 100u) == APP_RESULT_OK);
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    autotest_never_enabled();

    /* A queued non-motion command near the deadline also claims control.
     * An additional request returning BUSY must never re-arm the auto test. */
    ready(false);
    CHECK(Board_Millis() < MOTOR_OPEN_AUTOTEST_DELAY_MS - 1u);
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS - Board_Millis() - 1u);
    CHECK(App_Submit(APP_CMD_DIRECTION, -1.0f, 0.0f) == APP_RESULT_PENDING);
    CHECK(App_Submit(APP_CMD_CALIBRATE, 0.0f, 0.0f) == APP_RESULT_BUSY);
    autotest_ms(10000u);
    CHECK(state().direction_confirmed && state().sensor_direction == -1);
    autotest_never_enabled();

    ready(false);
    CHECK(App_SubmitCan(APP_CMD_CALIBRATE, 0.0f, 0.0f, 101u, Board_Millis()) == APP_RESULT_PENDING);
    CHECK(App_SubmitCan(APP_CMD_OPEN, 20.0f, 0.8f, 102u, Board_Millis()) == APP_RESULT_BUSY);
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    CHECK(state().can_completed_tag == 101u && state().can_result == APP_RESULT_OK);
    autotest_never_enabled();
}

static void test_autotest_manual_motion_cancels_future_auto(void)
{
    unsigned stopped_enable_count;
    ready(false);
    CHECK(command(APP_CMD_OPEN, 10.0f, 0.4f) == APP_RESULT_OK);
    CHECK(driver);
    CHECK(command(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_OK);
    stopped_enable_count = enable_count;
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    CHECK(!driver && enable_count == stopped_enable_count && state().foc.mode == FOC_MODE_STOP);

    ready(false);
    CHECK(can_command(APP_CMD_OPEN, 10.0f, 0.4f, 103u) == APP_RESULT_OK);
    autotest_ms(CAN_MOTION_TIMEOUT_MS + 1u);
    CHECK(!driver && state().command_timed_out);
    stopped_enable_count = enable_count;
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    CHECK(!driver && enable_count == stopped_enable_count && state().foc.mode == FOC_MODE_STOP);
}

static void test_autotest_trip_clear_does_not_rearm(void)
{
    ready(false);
    hardware_faults = BOARD_FAULT_PWM;
    sample(2048u, 2048u);
    CHECK((state().faults & APP_FAULT_BOARD) != 0u && !driver);
    hardware_faults = 0u;
    CHECK(command(APP_CMD_CLEAR, 0.0f, 0.0f) == APP_RESULT_OK);
    steady(MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(state().offset_ready && state().faults == 0u);
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    autotest_never_enabled();
}

static void test_autotest_offset_unavailable_or_invalid(void)
{
    /* Deadline arrives with no ADC calibration yet. Becoming ready afterwards
     * must not defer the one-shot attempt into an unexpected later start. */
    boot(false);
    fake_us = (uint64_t)MOTOR_OPEN_AUTOTEST_DELAY_MS * 1000u;
    App_Poll();
    CHECK(!state().offset_ready && !driver);
    steady(MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(state().offset_ready);
    autotest_ms(10000u);
    autotest_never_enabled();

    boot(false);
    samples(MOTOR_OFFSET_SAMPLE_COUNT, 100u, 2048u);
    CHECK((state().faults & APP_FAULT_OFFSET) != 0u && !state().offset_ready);
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    autotest_never_enabled();
}

static void test_autotest_reset_inhibit(void)
{
    static const uint32_t inhibited[] = {
        0u,       /* Unknown/no recorded power-on event. */
        1u << 26, /* PINRST alone is not a complete board power cycle. */
        1u << 28, /* RCC_CSR SFTRSTF */
        1u << 29, /* RCC_CSR IWDGRSTF */
        1u << 30, /* RCC_CSR WWDGRSTF */
        (1u << 27) | (1u << 28),
        (1u << 27) | (1u << 29),
        (1u << 27) | (1u << 30)
    };
    unsigned i;
    for (i = 0u; i < sizeof(inhibited) / sizeof(inhibited[0]); ++i) {
        boot(false);
        fake_reset_cause = inhibited[i];
        CHECK(App_Init());
        autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + MOTOR_COMMAND_TIMEOUT_MS + 10000u);
        CHECK(state().offset_ready && state().faults == 0u);
        autotest_never_enabled();
    }
    /* POR is required; it may accompany a pin-reset flag on real power-up. */
    boot(false);
    fake_reset_cause = (1u << 27) | (1u << 26);
    CHECK(App_Init());
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 1u);
#if MOTOR_OPEN_AUTOTEST
    CHECK(driver && state().foc.mode == FOC_MODE_OPEN_LOOP);
#else
    autotest_never_enabled();
#endif
}

static void test_autotest_external_cancel_api(void)
{
    ready(false);
    App_CancelStartupMotion();
    autotest_ms(MOTOR_OPEN_AUTOTEST_DELAY_MS + 10000u);
    autotest_never_enabled();

    boot(false);
    steady(MOTOR_OPEN_AUTOTEST_DELAY_MS * (MOTOR_ADC_HZ / 1000u) - 1u);
    App_CancelStartupMotion(); /* Valid CAN request may be rejected before App_SubmitCan. */
    autotest_ms(MOTOR_COMMAND_TIMEOUT_MS + 10000u);
    autotest_never_enabled();
}

static void test_autotest_millisecond_wrap(void)
{
    unsigned stopped_enable_count;
    boot(false);
    fake_us = ((uint64_t)UINT32_MAX - 1000u) * 1000u;
    CHECK(App_Init());
    steady(MOTOR_OPEN_AUTOTEST_DELAY_MS * (MOTOR_ADC_HZ / 1000u) - 1u);
    autotest_never_enabled();
    CHECK(Board_Millis() < MOTOR_OPEN_AUTOTEST_DELAY_MS); /* Clock crossed UINT32_MAX. */
    steady(MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
#if MOTOR_OPEN_AUTOTEST
    CHECK(driver && state().foc.mode == FOC_MODE_OPEN_LOOP);
    autotest_ms(MOTOR_COMMAND_TIMEOUT_MS + 1u);
    CHECK(!driver && state().command_timed_out);
#else
    autotest_never_enabled();
    autotest_ms(MOTOR_COMMAND_TIMEOUT_MS + 1u);
    autotest_never_enabled();
#endif
    stopped_enable_count = enable_count;
    autotest_ms(10000u);
    CHECK(!driver && enable_count == stopped_enable_count);
}

int main(void)
{
    test_autotest_nominal_one_shot();
    test_autotest_stop_and_commands_cancel();
    test_autotest_manual_motion_cancels_future_auto();
    test_autotest_trip_clear_does_not_rearm();
    test_autotest_offset_unavailable_or_invalid();
    test_autotest_reset_inhibit();
    test_autotest_external_cancel_api();
    test_autotest_millisecond_wrap();
    printf("Open autotest integration passed (MOTOR_OPEN_AUTOTEST=%d).\n",
           MOTOR_OPEN_AUTOTEST);
    return EXIT_SUCCESS;
}

