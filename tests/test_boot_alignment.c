/* Reuse the production-app fake BSP.  Compile twice with startup alignment
 * disabled/enabled so the release default and its opt-out both stay tested. */
#define main app_suite_main
#include "test_app.c"
#undef main

#ifndef MOTOR_BOOT_AUTO_ALIGN
#error MOTOR_BOOT_AUTO_ALIGN must be defined by motor_config.h
#endif

static void align_ms(uint32_t milliseconds)
{
    steady((unsigned)(milliseconds * (MOTOR_ADC_HZ / 1000u)));
}

static void check_never_drove(void)
{
    CHECK(!driver && enable_count == 0u);
    CHECK(state().foc.mode == FOC_MODE_STOP);
    CHECK(!state().zero_valid);
}

static void test_power_on_alignment_one_shot(void)
{
#if MOTOR_BOOT_AUTO_ALIGN
    unsigned completed_enable_count;
#endif
    boot(true);
    CHECK(state().sensor_direction == MOTOR_SENSOR_DIRECTION_DEFAULT);
    CHECK(state().direction_confirmed == (MOTOR_BOOT_AUTO_ALIGN != 0));
    CHECK(!state().offset_ready && !driver);

    steady(MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(state().offset_ready && !driver);
    steady(MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
#if MOTOR_BOOT_AUTO_ALIGN
    CHECK(driver && state().foc.mode == FOC_MODE_ALIGNMENT);
    CHECK(state().control_source == APP_SOURCE_LOCAL);
    CHECK(!state().zero_valid && state().faults == 0u);
    CHECK(state().foc.vd_v > 0.0f && state().foc.vd_v < 0.01f);
    CHECK(fabsf(state().foc.vq_v) < 0.0001f);
    align_ms(MOTOR_ALIGN_DURATION_MS - 350u);
    CHECK(driver && state().foc.mode == FOC_MODE_ALIGNMENT);
    CHECK(fabsf(state().foc.vd_v - MOTOR_ALIGN_VOLTAGE_V) < 0.0001f);
    align_ms(352u);
    CHECK(!driver && state().foc.mode == FOC_MODE_STOP);
    CHECK(state().zero_valid && state().faults == 0u);
    CHECK(state().control_source == APP_SOURCE_NONE);
    completed_enable_count = enable_count;
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS);
    CHECK(!driver && enable_count == completed_enable_count);
#else
    check_never_drove();
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS);
    check_never_drove();
#endif
}

static void test_stop_cancels_pending_alignment(void)
{
    boot(true);
    CHECK(command(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_OK);
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS + 10u);
    check_never_drove();
}

static void test_missing_encoder_cannot_start_late(void)
{
    boot(false);
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + 1u);
    check_never_drove();
    encoder_available = true;
    align_ms(MOTOR_ALIGN_DURATION_MS + 100u);
    check_never_drove();
}

static void test_non_power_resets_are_inhibited(void)
{
    static const uint32_t inhibited[] = {
        0u,
        1u << 26,
        1u << 28,
        1u << 29,
        1u << 30,
        (1u << 27) | (1u << 28),
        (1u << 27) | (1u << 29),
        (1u << 27) | (1u << 30)
    };
    unsigned i;
    for (i = 0u; i < sizeof(inhibited) / sizeof(inhibited[0]); ++i) {
        boot(true);
        fake_reset_cause = inhibited[i];
        CHECK(App_Init());
        enable_count = 0u;
        align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS + 10u);
        check_never_drove();
    }
}

static void test_external_transport_cancel(void)
{
    boot(true);
    App_CancelStartupMotion();
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS + 10u);
    check_never_drove();
}

static void test_millisecond_wrap(void)
{
    boot(true);
    fake_us = ((uint64_t)UINT32_MAX - 300u) * 1000u;
    CHECK(App_Init());
    steady(MOTOR_OFFSET_SAMPLE_COUNT + MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
#if MOTOR_BOOT_AUTO_ALIGN
    CHECK(driver && state().foc.mode == FOC_MODE_ALIGNMENT);
    align_ms(MOTOR_ALIGN_DURATION_MS + 2u);
    CHECK(!driver && state().zero_valid && state().faults == 0u);
#else
    check_never_drove();
#endif
}

static void test_persisted_calibration_skips_alignment(void)
{
    stored_calibration = (BoardCalibrationInfo){
        true, false, (uint16_t)MOTOR_MODEL, (uint8_t)MOTOR_POLE_PAIRS, -1, 1.25f
    };
    boot(true);
    CHECK(state().calibration_stored && !state().calibration_mismatch);
    CHECK(state().sensor_direction == -1 && state().direction_confirmed);
    steady(MOTOR_OFFSET_SAMPLE_COUNT + MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
    CHECK(state().offset_ready && state().zero_valid && !driver);
    CHECK(fabsf(state().electrical_zero_rad - 1.25f) < 0.0001f);
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS);
    CHECK(!driver && enable_count == 0u && state().foc.mode == FOC_MODE_STOP);

    /* A software reset still restores a valid persisted zero without motion. */
    fake_reset_cause = 1u << 28;
    CHECK(App_Init()); enable_count = 0u;
    steady(MOTOR_OFFSET_SAMPLE_COUNT + MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
    CHECK(state().zero_valid && !driver && enable_count == 0u);
    stored_calibration = (BoardCalibrationInfo){0};
}

static void test_bad_or_wrong_profile_storage_blocks_auto_alignment(void)
{
    stored_calibration = (BoardCalibrationInfo){
        true, false, (uint16_t)(MOTOR_MODEL == MOTOR_MODEL_2808 ?
            MOTOR_MODEL_4015 : MOTOR_MODEL_2808), (uint8_t)MOTOR_POLE_PAIRS, 1, 0.5f
    };
    boot(true);
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS + 10u);
    CHECK(state().calibration_stored && state().calibration_mismatch);
    CHECK(!state().direction_confirmed && !state().zero_valid);
    check_never_drove();

    stored_calibration = (BoardCalibrationInfo){false, true, 0u, 0u, 0, 0.0f};
    boot(true);
    align_ms(MOTOR_BOOT_ALIGN_START_TIMEOUT_MS + MOTOR_ALIGN_DURATION_MS + 10u);
    CHECK(state().calibration_storage_error && !state().zero_valid);
    check_never_drove();
    stored_calibration = (BoardCalibrationInfo){0};
}

int main(void)
{
    test_power_on_alignment_one_shot();
    test_stop_cancels_pending_alignment();
    test_missing_encoder_cannot_start_late();
    test_non_power_resets_are_inhibited();
    test_external_transport_cancel();
    test_millisecond_wrap();
    test_persisted_calibration_skips_alignment();
    test_bad_or_wrong_profile_storage_blocks_auto_alignment();
    printf("Boot alignment integration passed (MOTOR_BOOT_AUTO_ALIGN=%d).\n",
           MOTOR_BOOT_AUTO_ALIGN);
    return EXIT_SUCCESS;
}
