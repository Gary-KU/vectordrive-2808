/* Host integration tests for the production app, console and FOC sources.
 * This fake BSP models time, ADC callbacks, encoder availability and UART data.
 * It does not claim to verify STM32 registers, real-time execution or hardware.
 */
#include "app.h"
#include "board.h"
#include "board_calibration.h"
#include "console.h"
#include "motor_config.h"
#include "can_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef TEST_BAD_CURRENT_CONFIG
#define TEST_BAD_CURRENT_CONFIG 0
#endif

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    exit(EXIT_FAILURE); } } while (0)

static uint64_t fake_us;
static uint32_t fake_cycles, cycle_cost, hardware_faults, fake_reset_cause;
static bool driver, started, led, button, encoder_available;
static float encoder_angle, last_pwm[3];
static unsigned enable_count, feed_count;
static char uart_input[4096], uart_output[32768];
static size_t input_head, input_tail, output_length;
static uint32_t rx_lost;
static size_t lose_after_read_index;
static unsigned discard_count;
static void (*encoder_read_hook)(void);
static BoardCalibrationInfo stored_calibration;

void BoardCalibration_Load(void) { }
BoardCalibrationInfo BoardCalibration_GetInfo(void) { return stored_calibration; }

bool Board_Init(void) { return true; }
bool Board_StartSampling(void) { started = true; return true; }
void Board_SetDriverEnabled(bool on)
{
    if (on) {
        CHECK(started && hardware_faults == 0u);
        ++enable_count;
    }
    driver = on;
}
bool Board_DriverEnabled(void) { return driver; }
uint32_t Board_FaultFlags(void) { return hardware_faults; }
uint32_t Board_ResetCause(void) { return fake_reset_cause; }
uint32_t Board_EnterCritical(void) { return 0u; }
void Board_ExitCritical(uint32_t previous) { CHECK(previous == 0u); }
uint32_t Board_Micros32(void) { return (uint32_t)fake_us; }
uint32_t Board_Millis(void) { return (uint32_t)(fake_us / 1000u); }
uint32_t Board_CycleCount(void)
{
    uint32_t result = fake_cycles;
    fake_cycles += cycle_cost;
    return result;
}
void Board_SetPwm(float u, float v, float w)
{
    CHECK(isfinite(u) && isfinite(v) && isfinite(w));
    CHECK(u >= BOARD_PWM_MIN_DUTY && u <= BOARD_PWM_MAX_DUTY);
    CHECK(v >= BOARD_PWM_MIN_DUTY && v <= BOARD_PWM_MAX_DUTY);
    CHECK(w >= BOARD_PWM_MIN_DUTY && w <= BOARD_PWM_MAX_DUTY);
    last_pwm[0] = u; last_pwm[1] = v; last_pwm[2] = w;
}
bool Board_ButtonPressed(void) { return button; }
void Board_SetLed(bool on) { led = on; }
bool Board_ReadEncoder(float *angle)
{
    if (encoder_read_hook != NULL) {
        void (*hook)(void) = encoder_read_hook;
        encoder_read_hook = NULL;
        hook(); /* Models a current IRQ interrupting the foreground I2C call. */
    }
    if (!encoder_available) return false;
    *angle = encoder_angle;
    return true;
}
BoardEncoderStatus Board_EncoderStatus(void)
{
    return encoder_available ? BOARD_ENCODER_OK : BOARD_ENCODER_BUS_ERROR;
}
void Board_FeedWatchdog(void) { ++feed_count; }
bool Board_UartRead(char *character)
{
    if (input_tail == input_head) return false;
    *character = uart_input[input_tail++];
    if (input_tail == lose_after_read_index) ++rx_lost;
    return true;
}
void Board_UartDiscardInput(void)
{
    input_tail = input_head;
    ++discard_count;
}
void Board_UartWrite(const char *text)
{
    size_t count = strlen(text);
    CHECK(output_length + count < sizeof(uart_output));
    memcpy(uart_output + output_length, text, count + 1u);
    output_length += count;
}
uint32_t Board_UartTxDropped(void) { return 0u; }
uint32_t Board_UartRxDropped(void) { return rx_lost; }

static AppStatus state(void)
{
    AppStatus result;
    App_GetStatus(&result);
    return result;
}

static void sample(uint16_t u, uint16_t w)
{
    fake_us += 1000000u / MOTOR_ADC_HZ;
    App_Poll();
    App_OnAdcSample(u, w);
}

static void samples(unsigned count, uint16_t u, uint16_t w)
{
    while (count-- != 0u) sample(u, w);
}

static void steady(unsigned count) { samples(count, 2048u, 2048u); }

static void boot(bool with_encoder)
{
    fake_us = 0u; fake_cycles = cycle_cost = hardware_faults = 0u;
    fake_reset_cause = BOARD_RESET_POWER_ON_MASK;
    driver = started = led = button = false;
    encoder_available = with_encoder; encoder_angle = 1.0f;
    enable_count = feed_count = 0u;
    input_head = input_tail = output_length = 0u; rx_lost = 0u;
    lose_after_read_index = (size_t)-1; discard_count = 0u;
    encoder_read_hook = NULL;
    uart_output[0] = '\0';
    CHECK(Board_Init());
    CHECK(App_Init());
    CHECK(!driver);
    CHECK(Board_StartSampling());
    Console_Init();
}

static void ready(bool with_encoder)
{
    boot(with_encoder);
    steady(MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(state().offset_ready);
    CHECK(!driver && state().foc.mode == FOC_MODE_STOP);
    CHECK(state().sense_confirmed == (MOTOR_CURRENT_SENSE_CONFIRMED != 0));
}

static AppResult command(AppCommandKind kind, float value, float auxiliary)
{
    CHECK(App_Submit(kind, value, auxiliary) == APP_RESULT_PENDING);
    steady(MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
    return state().last_result;
}

static void rx(const char *text)
{
    size_t count = strlen(text);
    CHECK(input_head + count <= sizeof(uart_input));
    memcpy(uart_input + input_head, text, count);
    input_head += count;
}

static void console_drain(void)
{
    while (input_head != input_tail) Console_Poll();
    Console_Poll();
}

static void test_boot_and_exact_offset_count(void)
{
    boot(false);
    CHECK(!state().offset_ready && !state().zero_valid);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_CALIBRATING);
    CHECK(!driver && enable_count == 0u);
    steady(MOTOR_OFFSET_SAMPLE_COUNT - 1u - (MOTOR_ADC_HZ / MOTOR_CONTROL_HZ));
    CHECK(state().adc_samples == MOTOR_OFFSET_SAMPLE_COUNT - 1u);
    CHECK(!state().offset_ready);
    steady(1u);
    CHECK(state().adc_samples == MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(state().offset_ready && !driver);
    CHECK(fabsf(state().offset_u - 2048.0f) < 0.01f);
    CHECK(fabsf(state().offset_w - 2048.0f) < 0.01f);
    CHECK(feed_count > 0u);
}

static void test_adc_cadence_and_fixed_control_period(void)
{
    uint64_t began_us;
    const float dt = 1.0f / (float)MOTOR_CONTROL_HZ;
    boot(false);
    steady(MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(state().offset_ready);
    CHECK(fake_us == (uint64_t)MOTOR_OFFSET_SAMPLE_COUNT * (1000000u / MOTOR_ADC_HZ));
    began_us = fake_us;
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    CHECK(fake_us - began_us == 1000000u / MOTOR_CONTROL_HZ);
    CHECK(driver && state().foc.mode == FOC_MODE_OPEN_LOOP);
    CHECK(fabsf(state().foc.vq_v - dt) < 1e-8f); /* 1 V/s slew. */
    CHECK(fabsf(state().foc.open_velocity_rad_s - 20.0f * dt) < 1e-7f);
    CHECK(fabsf(state().foc.electrical_angle_rad - 20.0f * dt * dt) < 1e-9f);
    CHECK(command(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_OK);
}

static void test_offset_faults_and_clear(void)
{
    boot(false);
    samples(MOTOR_OFFSET_SAMPLE_COUNT, 100u, 2048u);
    CHECK((state().faults & APP_FAULT_OFFSET) != 0u);
    CHECK(!state().offset_ready && !driver);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_FAULT);
    CHECK(command(APP_CMD_CLEAR, 0.0f, 0.0f) == APP_RESULT_OK);
    CHECK(state().faults == 0u && !state().offset_ready && !state().zero_valid);
    steady(MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(state().offset_ready && !driver);
    boot(false);
    for (unsigned i = 0u; i < MOTOR_OFFSET_SAMPLE_COUNT; ++i)
        sample((i & 1u) != 0u ? 2200u : 1900u, 2048u);
    CHECK((state().faults & APP_FAULT_OFFSET) != 0u);
    CHECK(!state().offset_ready && !driver);
}

static void test_open_without_encoder_and_timeout(void)
{
    uint32_t began_ms;
    uint64_t deadline_us;
    const unsigned control_samples = MOTOR_ADC_HZ / MOTOR_CONTROL_HZ;
    const unsigned control_period_us = 1000000u / MOTOR_CONTROL_HZ;
    ready(false);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    CHECK(driver && state().foc.mode == FOC_MODE_OPEN_LOOP);
    CHECK(!state().sensor_valid && state().faults == 0u);
    began_ms = Board_Millis();
    deadline_us = (uint64_t)(began_ms + MOTOR_COMMAND_TIMEOUT_MS) * 1000u;
    fake_us = deadline_us - 2u * control_period_us;
    steady(control_samples);
    CHECK(fake_us == deadline_us - control_period_us);
    CHECK(driver && !state().command_timed_out);
    steady(control_samples);
    CHECK(fake_us == deadline_us);
    CHECK(!driver && state().foc.mode == FOC_MODE_STOP);
    CHECK(state().command_timed_out && state().faults == 0u);
    steady(20u);
    CHECK(!driver); /* Expiry cannot restart motion by itself. */
}

static void test_stop_priority_and_mode_interlocks(void)
{
    ready(false);
    CHECK(App_Submit(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_PENDING);
    CHECK(App_Submit(APP_CMD_SPEED, 2.0f, 0.0f) == APP_RESULT_BUSY);
    CHECK(App_Submit(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_PENDING);
    CHECK(!driver);
    steady(2u);
    CHECK(!driver && enable_count == 0u && state().foc.mode == FOC_MODE_STOP);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    CHECK(driver);
    CHECK(command(APP_CMD_CURRENT, 0.1f, 0.0f) == APP_RESULT_NEED_STOP);
    CHECK(command(APP_CMD_SPEED, 1.0f, 0.0f) == APP_RESULT_NEED_STOP);
    CHECK(command(APP_CMD_ALIGN, 0.0f, 0.0f) == APP_RESULT_NEED_STOP);
    CHECK(command(APP_CMD_DIRECTION, -1.0f, 0.0f) == APP_RESULT_NEED_STOP);
    CHECK(command(APP_CMD_CALIBRATE, 0.0f, 0.0f) == APP_RESULT_NEED_STOP);
    CHECK(command(APP_CMD_BUS, 14.0f, 0.0f) == APP_RESULT_NEED_STOP);
    CHECK(App_Submit(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_PENDING);
    CHECK(!driver); /* GPIO EN is lowered before the next current ISR. */
    steady(2u);
    CHECK(state().foc.mode == FOC_MODE_STOP);
}

static void test_invalid_commands_and_configuration_gates(void)
{
    ready(true);
    CHECK(App_Submit(APP_CMD_OPEN, NAN, 0.8f) == APP_RESULT_RANGE);
    CHECK(App_Submit(APP_CMD_OPEN, 20.0f, INFINITY) == APP_RESULT_RANGE);
    CHECK(command((AppCommandKind)99, 0.0f, 0.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_OPEN, 101.0f, 0.8f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_OPEN, 20.0f, MOTOR_VOLTAGE_LIMIT_V + 0.1f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_DIRECTION, 0.0f, 0.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_BUS, 7.0f, 0.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_ALIGN, 0.0f, 0.0f) == APP_RESULT_NEED_DIRECTION);
#if MOTOR_CURRENT_SENSE_CONFIRMED
    CHECK(command(APP_CMD_CURRENT, 0.1f, 0.0f) == APP_RESULT_NEED_DIRECTION);
    CHECK(command(APP_CMD_SPEED, 1.0f, 0.0f) == APP_RESULT_NEED_DIRECTION);
#else
    CHECK(command(APP_CMD_CURRENT, 0.1f, 0.0f) == APP_RESULT_SENSE_UNKNOWN);
    CHECK(command(APP_CMD_SPEED, 1.0f, 0.0f) == APP_RESULT_SENSE_UNKNOWN);
#endif
    CHECK(!driver && state().faults == 0u);
}

static void align_motor(int direction)
{
    CHECK(command(APP_CMD_DIRECTION, (float)direction, 0.0f) == APP_RESULT_OK);
    steady(40u); /* Direction change requires a fresh encoder transaction. */
    CHECK(state().direction_confirmed && state().sensor_valid);
    CHECK(!state().zero_valid);
    CHECK(command(APP_CMD_ALIGN, 0.0f, 0.0f) == APP_RESULT_OK);
    CHECK(driver && state().foc.mode == FOC_MODE_ALIGNMENT);
    steady((MOTOR_ALIGN_DURATION_MS + 1u) * (MOTOR_ADC_HZ / 1000u));
    CHECK(state().faults == 0u && state().zero_valid);
    CHECK(!driver && state().foc.mode == FOC_MODE_STOP);
}

static void test_direction_alignment_and_closed_loop(void)
{
    ready(true);
    align_motor(-1);
    CHECK(state().sensor_direction == -1);
    CHECK(fabsf(state().angle_rad - (FOC_TWO_PI_F - encoder_angle)) < 0.0001f);
#if MOTOR_CURRENT_SENSE_CONFIRMED
    const float test_current_a = 0.5f * MOTOR_CURRENT_LIMIT_A;
    CHECK(command(APP_CMD_CURRENT, MOTOR_CURRENT_LIMIT_A + 0.001f, 0.0f) == APP_RESULT_RANGE);
    CHECK(!driver);
    CHECK(command(APP_CMD_CURRENT, test_current_a, 0.0f) == APP_RESULT_OK);
    steady(100u);
    CHECK(driver && state().foc.mode == FOC_MODE_CURRENT);
    CHECK(state().foc.iq_reference_a > 0.0f && state().foc.iq_reference_a <= test_current_a);
    CHECK(command(APP_CMD_SPEED, 2.0f, 0.0f) == APP_RESULT_NEED_STOP);
    CHECK(command(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_OK);
    CHECK(state().zero_valid);
    CHECK(command(APP_CMD_SPEED, 2.0f, 0.0f) == APP_RESULT_OK);
    CHECK(driver && state().foc.mode == FOC_MODE_SPEED);
    CHECK(command(APP_CMD_CURRENT, 0.1f, 0.0f) == APP_RESULT_NEED_STOP);
    encoder_available = false;
    fake_us += MOTOR_ENCODER_PERIOD_US;
    App_Poll();
    steady(2u);
    CHECK(!driver && (state().faults & APP_FAULT_ENCODER) != 0u);
    CHECK(!state().zero_valid);
#else
    CHECK(command(APP_CMD_CURRENT, 0.1f, 0.0f) == APP_RESULT_SENSE_UNKNOWN);
    CHECK(command(APP_CMD_SPEED, 2.0f, 0.0f) == APP_RESULT_SENSE_UNKNOWN);
    CHECK(!driver && state().zero_valid);
#endif
}

static void test_alignment_rejects_motion_and_encoder_loss(void)
{
    ready(true);
    CHECK(command(APP_CMD_DIRECTION, 1.0f, 0.0f) == APP_RESULT_OK);
    steady(40u);
    CHECK(command(APP_CMD_ALIGN, 0.0f, 0.0f) == APP_RESULT_OK);
    steady((MOTOR_ALIGN_DURATION_MS - 250u) * (MOTOR_ADC_HZ / 1000u));
    CHECK(driver && !state().zero_valid);
    encoder_angle += 0.1f;
    steady(40u);
    CHECK(!driver && (state().faults & APP_FAULT_ALIGNMENT) != 0u);
    CHECK(!state().zero_valid);
    ready(true);
    CHECK(command(APP_CMD_DIRECTION, 1.0f, 0.0f) == APP_RESULT_OK);
    steady(40u);
    CHECK(command(APP_CMD_ALIGN, 0.0f, 0.0f) == APP_RESULT_OK);
    encoder_available = false;
    fake_us += MOTOR_ENCODER_PERIOD_US;
    App_Poll();
    steady(2u);
    CHECK(!driver && (state().faults & APP_FAULT_ENCODER) != 0u);
}

static void direction_change_interrupt(void)
{
    unsigned i;
    CHECK(App_Submit(APP_CMD_DIRECTION, -1.0f, 0.0f) == APP_RESULT_PENDING);
    for (i = 0u; i < MOTOR_ADC_HZ / MOTOR_CONTROL_HZ; ++i)
        App_OnAdcSample(2048u, 2048u);
    CHECK(state().sensor_direction == -1);
}

static void test_direction_change_during_encoder_transaction(void)
{
    ready(true);
    CHECK(state().sensor_valid);
    encoder_read_hook = direction_change_interrupt;
    fake_us += MOTOR_ENCODER_PERIOD_US;
    App_Poll();
    CHECK(encoder_read_hook == NULL);
    steady(MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
    CHECK(state().sensor_direction == -1 && !state().sensor_valid);
    CHECK(!driver && !state().zero_valid);
    steady(40u);
    CHECK(state().sensor_valid);
    CHECK(fabsf(state().angle_rad - (FOC_TWO_PI_F - encoder_angle)) < 0.0001f);
}

static void test_every_sample_protection_and_overrun(void)
{
    ready(false);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    hardware_faults = BOARD_FAULT_PWM;
    sample(2048u, 2048u); /* Intermediate ADC sample, before the next FOC step. */
    CHECK((state().faults & APP_FAULT_BOARD) != 0u && !driver);
    CHECK(command(APP_CMD_CLEAR, 0.0f, 0.0f) == APP_RESULT_FAULT);
#if MOTOR_CURRENT_SENSE_CONFIRMED
    const float current_per_count = MOTOR_ADC_REFERENCE_V /
        (MOTOR_ADC_FULL_SCALE * MOTOR_SHUNT_U_OHM * MOTOR_SENSE_GAIN_U);
    const uint16_t phase_counts = (uint16_t)ceilf(0.6f * MOTOR_OVERCURRENT_TRIP_A / current_per_count);
    ready(false);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    sample((uint16_t)(2048u + ceilf(1.01f * MOTOR_OVERCURRENT_TRIP_A /
                                   current_per_count)), 2048u);
    CHECK(!driver && (state().faults & FOC_FAULT_OVERCURRENT) != 0u);
    ready(false);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    CHECK(phase_counts * current_per_count < MOTOR_OVERCURRENT_TRIP_A);
    CHECK(2u * phase_counts * current_per_count > MOTOR_OVERCURRENT_TRIP_A);
    sample((uint16_t)(2048u + phase_counts), (uint16_t)(2048u + phase_counts));
    /* Each sensed phase is below trip; only reconstructed V exceeds it. */
    CHECK(!driver && (state().faults & FOC_FAULT_OVERCURRENT) != 0u);
    ready(false);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    sample(0u, 2048u);
    CHECK(!driver && (state().faults & APP_FAULT_ADC_RAIL) != 0u);
    ready(false);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    sample(2048u, 4095u);
    CHECK(!driver && (state().faults & APP_FAULT_ADC_RAIL) != 0u);
#endif
    ready(false);
    CHECK(command(APP_CMD_OPEN, 20.0f, 0.8f) == APP_RESULT_OK);
    cycle_cost = (MOTOR_CPU_HZ / MOTOR_ADC_HZ) / 2u;
    steady(MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
    CHECK(driver && state().faults == 0u); /* Deadline is ADC cadence, not the PWM carrier. */
    cycle_cost = MOTOR_CPU_HZ / MOTOR_ADC_HZ;
    steady(2u);
    CHECK(!driver && (state().faults & APP_FAULT_OVERRUN) != 0u);
}

static void test_console_rejects_bad_and_damaged_lines(void)
{
    char oversized[160];
    ready(false);
    rx("open nan 0.8\niq inf\nopen 20junk 0.8\nopen 20 0.8 extra\n"
       "open 2e1 0.8\nopen 0x14 0.8\nopen . 0.8\nopen + 0.8\n"
       "open 20 0.1234567\nopen 1000001 0.8\n");
    console_drain();
    steady(2u);
    CHECK(!driver && strstr(uart_output, "SYNTAX") != NULL);
    rx("open 20 "); console_drain();
    ++rx_lost; /* A lost byte while assembling a command must poison the line. */
    rx("0.8\n"); console_drain();
    steady(2u);
    CHECK(!driver && discard_count == 1u);
    rx("\n"); console_drain(); /* Boundary must arrive AFTER the buffered data was flushed. */
    CHECK(strstr(uart_output, "discarded") != NULL);
    memset(oversized, ' ', sizeof(oversized));
    memcpy(oversized, "open 20 0.8", 11u);
    oversized[sizeof(oversized) - 2u] = '\n';
    oversized[sizeof(oversized) - 1u] = '\0';
    rx(oversized); console_drain();
    steady(2u);
    CHECK(!driver);
    rx("open 20 0.8\n"); console_drain(); steady(2u);
    CHECK(driver && state().foc.mode == FOC_MODE_OPEN_LOOP);
    rx("stop\n"); console_drain();
    CHECK(!driver);
    steady(2u);
    CHECK(state().foc.mode == FOC_MODE_STOP);
}

static void test_buffered_rx_loss_and_old_eol(void)
{
    ready(false);
    rx("status\nopen 20 0.8\n");
    ++rx_lost;
    console_drain();
    steady(2u);
    CHECK(!driver && enable_count == 0u && discard_count == 1u);
    CHECK(input_tail == input_head);
    /* Even a newly received command before a fresh boundary is discarded. */
    rx("open 20 0.8\n"); console_drain(); steady(2u);
    CHECK(!driver);
    rx("open 20 0.8\n"); console_drain(); steady(2u);
    CHECK(driver);

    ready(false);
    rx("status\nopen 20 0.8\n");
    lose_after_read_index = strlen("status\n");
    console_drain();
    steady(2u);
    CHECK(!driver && enable_count == 0u && discard_count == 1u);
    /* The old EOL read at the instant of overflow must not end discard mode. */
    rx("open 20 0.8\n"); console_drain(); steady(2u);
    CHECK(!driver);
    rx("open 20 0.8\n"); console_drain(); steady(2u);
    CHECK(driver);
}

static void test_fixed_decimal_console_numbers(void)
{
    ready(false);
    rx("open +.5 +.8\n"); console_drain(); steady(2u);
    CHECK(state().last_result == APP_RESULT_OK && driver);
    CHECK(command(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_OK);
    rx("open -20.000000 0.800000\n"); console_drain(); steady(2u);
    CHECK(state().last_result == APP_RESULT_OK && driver);
    CHECK(command(APP_CMD_STOP, 0.0f, 0.0f) == APP_RESULT_OK);
    rx("bus +14.\n"); console_drain(); steady(2u);
    CHECK(state().last_result == APP_RESULT_OK);
    CHECK(fabsf(state().bus_voltage_v - 14.0f) < 0.0001f);
}

static AppResult can_command(AppCommandKind kind, float value, float auxiliary, uint32_t tag)
{
    CHECK(App_SubmitCan(kind, value, auxiliary, tag, Board_Millis()) == APP_RESULT_PENDING);
    steady(MOTOR_ADC_HZ / MOTOR_CONTROL_HZ);
    CHECK(state().can_completed_tag == tag);
    return state().can_result;
}

static void test_can_deadline_ownership_and_cancellation(void)
{
    uint32_t began, epoch;
    ready(false);
    CHECK(can_command(APP_CMD_OPEN,20.0f,0.8f,1u) == APP_RESULT_OK);
    CHECK(driver && state().control_source == APP_SOURCE_CAN);
    CHECK(command(APP_CMD_OPEN,10.0f,0.5f) == APP_RESULT_CONTROL_OWNED);
    CHECK(command(APP_CMD_BUS,14.0f,0.0f) == APP_RESULT_CONTROL_OWNED);
    CHECK(command(APP_CMD_CLEAR,0.0f,0.0f) == APP_RESULT_CONTROL_OWNED);
    began = Board_Millis(); epoch = state().can_session_epoch;
    fake_us = (uint64_t)(began + CAN_MOTION_TIMEOUT_MS + 1u) * 1000u;
    steady(2u);
    CHECK(!driver && state().command_timed_out && state().control_source == APP_SOURCE_NONE);
    CHECK(state().can_session_epoch != epoch);

    ready(false);
    CHECK(App_SubmitCan(APP_CMD_OPEN,20.0f,0.8f,2u,Board_Millis()) == APP_RESULT_PENDING);
    epoch = state().can_session_epoch;
    CHECK(App_Submit(APP_CMD_STOP,0.0f,0.0f) == APP_RESULT_PENDING);
    CHECK(state().can_completed_tag == 2u && state().can_result == APP_RESULT_CANCELLED);
    CHECK(state().can_session_epoch != epoch);
    steady(4u); CHECK(!driver && enable_count == 0u);
    CHECK(command(APP_CMD_OPEN,20.0f,0.8f) == APP_RESULT_OK);
    CHECK(can_command(APP_CMD_OPEN,20.0f,0.8f,3u) == APP_RESULT_CONTROL_OWNED);
    CHECK(can_command(APP_CMD_STOP,0.0f,0.0f,4u) == APP_RESULT_OK);
    CHECK(!driver && state().control_source == APP_SOURCE_NONE);

    ready(false);
    began = Board_Millis(); fake_us += (CAN_COMMAND_MAX_AGE_MS + 1u) * 1000u;
    CHECK(App_SubmitCan(APP_CMD_OPEN,20.0f,0.8f,5u,began) == APP_RESULT_PENDING);
    steady(2u); CHECK(!driver && state().can_result == APP_RESULT_STALE);
    CHECK(App_SubmitCan(APP_CMD_OPEN,20.0f,0.8f,0u,Board_Millis()) == APP_RESULT_RANGE);

    ready(false);
    fake_us = ((uint64_t)UINT32_MAX - 20u) * 1000u;
    CHECK(can_command(APP_CMD_OPEN,20.0f,0.8f,6u) == APP_RESULT_OK);
    fake_us += (CAN_MOTION_TIMEOUT_MS + 2u) * 1000u;
    steady(2u); CHECK(!driver && state().command_timed_out);
}

static void test_can_transport_and_estop(void)
{
    ready(false);
    CHECK(can_command(APP_CMD_OPEN,20.0f,0.8f,10u) == APP_RESULT_OK);
    App_NotifyCanTransportFault(); CHECK(!driver);
    steady(2u);
    CHECK((state().faults & APP_FAULT_CAN) != 0u && !state().zero_valid);
    CHECK(state().control_source == APP_SOURCE_NONE);
    CHECK(command(APP_CMD_CLEAR,0.0f,0.0f) == APP_RESULT_OK);
    steady(MOTOR_OFFSET_SAMPLE_COUNT);
    CHECK(command(APP_CMD_OPEN,20.0f,0.8f) == APP_RESULT_OK);
    App_NotifyCanTransportFault(); steady(2u);
    CHECK(driver && state().faults == 0u); /* Idle/broken CAN cannot stop local commissioning. */
    CHECK(can_command(APP_CMD_ESTOP,0.0f,0.0f,11u) == APP_RESULT_OK);
    CHECK(!driver && (state().faults & APP_FAULT_ESTOP) != 0u);
    CHECK(can_command(APP_CMD_STOP,0.0f,0.0f,12u) == APP_RESULT_OK);
    CHECK((state().faults & APP_FAULT_ESTOP) != 0u); /* STOP never clears a latched e-stop. */

    ready(false);
    CHECK(App_SubmitCan(APP_CMD_OPEN,20.0f,0.8f,13u,Board_Millis()) == APP_RESULT_PENDING);
    App_NotifyCanTransportFault();
    CHECK(state().can_completed_tag == 13u && state().can_result == APP_RESULT_CANCELLED);
    steady(2u); CHECK(!driver && enable_count == 0u && (state().faults & APP_FAULT_CAN) != 0u);
}

static void test_can_alignment_and_closed_modes(void)
{
    uint32_t tag = 20u;
    ready(true);
    CHECK(can_command(APP_CMD_DIRECTION,1.0f,0.0f,tag++) == APP_RESULT_OK);
    steady(40u);
    CHECK(can_command(APP_CMD_ALIGN,0.0f,0.0f,tag++) == APP_RESULT_OK);
    for (unsigned i = 0u; i < MOTOR_ALIGN_DURATION_MS / 50u + 1u; ++i) {
        steady(50u * (MOTOR_ADC_HZ / 1000u));
        CHECK(can_command(APP_CMD_ALIGN,0.0f,0.0f,tag++) == APP_RESULT_OK);
    }
    CHECK(!driver && state().zero_valid && state().faults == 0u);
    CHECK(state().control_source == APP_SOURCE_NONE);
    {
        const unsigned previous_enable_count = enable_count;
        CHECK(can_command(APP_CMD_ALIGN,0.0f,0.0f,tag++) == APP_RESULT_OK);
        CHECK(!driver && enable_count == previous_enable_count);
    }
#if MOTOR_CURRENT_SENSE_CONFIRMED
    CHECK(can_command(APP_CMD_CURRENT,0.05f,0.0f,tag++) == APP_RESULT_OK);
    CHECK(driver && state().foc.mode == FOC_MODE_CURRENT && state().control_source == APP_SOURCE_CAN);
    CHECK(can_command(APP_CMD_SPEED,1.0f,0.0f,tag++) == APP_RESULT_NEED_STOP);
    CHECK(can_command(APP_CMD_STOP,0.0f,0.0f,tag++) == APP_RESULT_OK);
    CHECK(can_command(APP_CMD_SPEED,1.0f,0.0f,tag++) == APP_RESULT_OK);
    CHECK(driver && state().foc.mode == FOC_MODE_SPEED);
    CHECK(can_command(APP_CMD_SPEED,31.0f,0.0f,tag++) == APP_RESULT_RANGE);
#else
    CHECK(can_command(APP_CMD_CURRENT,0.05f,0.0f,tag++) == APP_RESULT_SENSE_UNKNOWN);
    CHECK(can_command(APP_CMD_SPEED,1.0f,0.0f,tag++) == APP_RESULT_SENSE_UNKNOWN);
#endif
    ready(true);
    CHECK(can_command(APP_CMD_DIRECTION,1.0f,0.0f,80u) == APP_RESULT_OK);
    steady(40u);
    CHECK(can_command(APP_CMD_ALIGN,0.0f,0.0f,81u) == APP_RESULT_OK);
    steady((CAN_MOTION_TIMEOUT_MS + 1u) * (MOTOR_ADC_HZ / 1000u));
    CHECK(!driver && !state().zero_valid && state().command_timed_out);
}

static void test_runtime_parameter_safety_and_calibration(void)
{
    static const float low[APP_PARAM_COUNT] = {0,0,0.01f,0.1f,0.01f,0,0,0.1f,8};
    static const float high[APP_PARAM_COUNT] = {5,2000,MOTOR_IQ_REQUEST_MAX_A,5,10,1,10,30,30};
    ready(true);
    align_motor(1);
    for (unsigned id = 1u; id <= APP_PARAM_COUNT; ++id) {
        CHECK(command(APP_CMD_PARAM_WRITE,low[id-1u],(float)id) == APP_RESULT_OK);
        CHECK(command(APP_CMD_PARAM_READ,0.0f,(float)id) == APP_RESULT_OK);
        CHECK(state().can_parameter_value == low[id-1u]);
        CHECK(command(APP_CMD_PARAM_WRITE,high[id-1u],(float)id) == APP_RESULT_OK);
        CHECK(state().parameters[id-1u] == high[id-1u]);
        CHECK(command(APP_CMD_PARAM_WRITE,high[id-1u]+0.01f,(float)id) == APP_RESULT_RANGE);
        CHECK(command(APP_CMD_PARAM_WRITE,low[id-1u]-0.01f,(float)id) == APP_RESULT_RANGE);
        CHECK(state().parameters[id-1u] == high[id-1u] && state().faults == 0u);
        CHECK(!driver && state().zero_valid == (id != APP_PARAM_BUS_VOLTAGE));
    }
    CHECK(command(APP_CMD_PARAM_READ,1.0f,1.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_PARAM_READ,0.0f,10.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_PARAM_WRITE,1.0f,1.5f) == APP_RESULT_RANGE);
    CHECK(App_Submit(APP_CMD_PARAM_WRITE,NAN,1.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_PARAM_WRITE,3.3001f,(float)APP_PARAM_IQ_LIMIT) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_PARAM_WRITE,-3.3f,(float)APP_PARAM_IQ_LIMIT) == APP_RESULT_RANGE);
    /* Both profiles now share the 3.3 A request ceiling, so the ceiling value
     * itself is accepted; the old 4015-only 1 A RANGE expectation is gone. */
    CHECK(command(APP_CMD_PARAM_WRITE,3.3f,(float)APP_PARAM_IQ_LIMIT) == APP_RESULT_OK);
    ready(true);
    align_motor(1);
    CHECK(command(APP_CMD_PARAM_WRITE,0.1f,(float)APP_PARAM_IQ_LIMIT) == APP_RESULT_OK);
#if MOTOR_CURRENT_SENSE_CONFIRMED
    CHECK(command(APP_CMD_CURRENT,0.101f,0.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_CURRENT,0.05f,0.0f) == APP_RESULT_OK);
    CHECK(command(APP_CMD_PARAM_WRITE,2.0f,(float)APP_PARAM_CURRENT_KP) == APP_RESULT_NEED_STOP);
    CHECK(state().parameters[APP_PARAM_CURRENT_KP-1u] == MOTOR_CURRENT_KP);
#endif
    CHECK(command(APP_CMD_STOP,0.0f,0.0f) == APP_RESULT_OK);
    CHECK(command(APP_CMD_ESTOP,0.0f,0.0f) == APP_RESULT_OK);
    CHECK(command(APP_CMD_PARAM_READ,0.0f,1.0f) == APP_RESULT_OK);
    CHECK(command(APP_CMD_PARAM_WRITE,2.0f,1.0f) == APP_RESULT_FAULT);
    /* UART BUS follows the same catalog bounds and still invalidates zero. */
    ready(true);
    align_motor(1);
    const FocParameterDescriptor *bus_parameter = FocParameters_Find(FOC_PARAMETER_BUS_VOLTAGE);
    CHECK(command(APP_CMD_BUS,bus_parameter->minimum-0.1f,0.0f) == APP_RESULT_RANGE);
    CHECK(command(APP_CMD_BUS,bus_parameter->maximum+0.1f,0.0f) == APP_RESULT_RANGE);
    CHECK(state().zero_valid);
    CHECK(command(APP_CMD_BUS,bus_parameter->minimum,0.0f) == APP_RESULT_OK);
    CHECK(!state().zero_valid && state().bus_voltage_v == bus_parameter->minimum);
    CHECK(command(APP_CMD_BUS,bus_parameter->maximum,0.0f) == APP_RESULT_OK);
    CHECK(!driver && state().bus_voltage_v == bus_parameter->maximum);
    /* Parameter queries cannot renew a running CAN motion deadline. */
    ready(true);
    CHECK(can_command(APP_CMD_OPEN,20.0f,0.8f,200u) == APP_RESULT_OK);
    for (unsigned n = 0u; n < 12u; ++n) {
        steady(10u * (MOTOR_ADC_HZ / 1000u));
        CHECK(can_command(APP_CMD_PARAM_READ,0.0f,1.0f,201u+n) == APP_RESULT_OK);
    }
    CHECK(!driver && state().command_timed_out && state().control_source == APP_SOURCE_NONE);
}

int main(void)
{
    if (TEST_BAD_CURRENT_CONFIG) {
        AppStatus result;
        CHECK(Board_Init());
        CHECK(!App_Init());
        App_GetStatus(&result);
        CHECK((result.faults & APP_FAULT_CONFIG) != 0u);
        CHECK(!driver && !result.offset_ready && !result.zero_valid);
        puts("Nonfinite current-sense configuration rejected with EN disabled.");
        return EXIT_SUCCESS;
    }
    test_boot_and_exact_offset_count();
    test_adc_cadence_and_fixed_control_period();
    test_offset_faults_and_clear();
    test_open_without_encoder_and_timeout();
    test_stop_priority_and_mode_interlocks();
    test_invalid_commands_and_configuration_gates();
    test_direction_alignment_and_closed_loop();
    test_alignment_rejects_motion_and_encoder_loss();
    test_direction_change_during_encoder_transaction();
    test_every_sample_protection_and_overrun();
    test_console_rejects_bad_and_damaged_lines();
    test_buffered_rx_loss_and_old_eol();
    test_fixed_decimal_console_numbers();
    test_can_deadline_ownership_and_cancellation();
    test_can_transport_and_estop();
    test_can_alignment_and_closed_modes();
    test_runtime_parameter_safety_and_calibration();
    printf("App/console integration tests passed (%s current sensing).\n",
           MOTOR_CURRENT_SENSE_CONFIRMED ? "test-confirmed" : "default-unconfigured");
    return EXIT_SUCCESS;
}
