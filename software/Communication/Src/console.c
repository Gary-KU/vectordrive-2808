#include "console.h"
#include "app.h"
#include "board.h"
#include "motor_config.h"
#include <math.h>
#include <string.h>

static char line[96];
static unsigned line_length;
static bool discard_line;
static AppResult previous_result;
static uint32_t previous_faults;
static bool previous_zero;
static uint32_t previous_rx_dropped;

static bool rx_input_healthy(void)
{
    const uint32_t dropped = Board_UartRxDropped();
    if (dropped == previous_rx_dropped) return true;
    previous_rx_dropped = dropped;
    Board_UartDiscardInput();
    discard_line = true; line_length = 0u;
    return false;
}

static void help(void)
{
    Board_UartWrite("\r\n" MOTOR_MODEL_NAME " motor. USART1 115200 8N1, newline terminated.\r\n"
        "status | stop | estop | clear | cal | bus <V> | dir <+1|-1> | align\r\n"
        "open <electrical_rad/s> [Vq_volts] : first test, e.g. open 20 0.8\r\n"
        "iq <A> : current/torque mode; speed <mechanical_rad/s> : cascaded PI\r\n"
        "Motion commands expire after 2 seconds; repeat to keep running.\r\n"
        "Stop before changing modes. Shunt/gain configuration required for iq/speed.\r\n");
}

typedef struct { char data[384]; unsigned used; } TextBuffer;

static void text_append(TextBuffer *b, const char *text)
{
    while (*text != '\0' && b->used + 1u < sizeof(b->data)) b->data[b->used++] = *text++;
    b->data[b->used] = '\0';
}

static void unsigned_append(TextBuffer *b, uint32_t value, unsigned base, unsigned width)
{
    static const char alphabet[] = "0123456789ABCDEF";
    char digits[11], one[2] = {0, 0};
    unsigned count = 0u;
    do { digits[count++] = alphabet[value % base]; value /= base; } while (value != 0u);
    while (count < width) digits[count++] = '0';
    while (count != 0u) { one[0] = digits[--count]; text_append(b, one); }
}

static void signed_append(TextBuffer *b, int32_t value)
{
    uint32_t magnitude = (uint32_t)value;
    if (value < 0) { text_append(b, "-"); magnitude = 0u - magnitude; }
    unsigned_append(b, magnitude, 10u, 0u);
}

static void unsigned_field(TextBuffer *b, const char *name, uint32_t value)
{
    text_append(b, name); unsigned_append(b, value, 10u, 0u);
}

static void signed_field(TextBuffer *b, const char *name, int32_t value)
{
    text_append(b, name); signed_append(b, value);
}

static void text_flush(TextBuffer *b)
{
    Board_UartWrite(b->data); b->used = 0u; b->data[0] = '\0';
}

static int32_t scaled(float value, float factor)
{
    if (!isfinite(value)) return 0;
    if (value > 1000000.0f) value = 1000000.0f;
    if (value < -1000000.0f) value = -1000000.0f;
    return (int32_t)(value * factor);
}

static void print_status(void)
{
    AppStatus s;
    TextBuffer buffer;
    buffer.used = 0u; buffer.data[0] = '\0';
    App_GetStatus(&s);
    text_append(&buffer, "mode="); text_append(&buffer, App_ModeName(s.foc.mode));
    text_append(&buffer, " motor=" MOTOR_MODEL_NAME);
    unsigned_field(&buffer, " pole_pairs=", MOTOR_POLE_PAIRS);
    text_append(&buffer, " fault=0x"); unsigned_append(&buffer, s.faults, 16u, 8u);
    text_append(&buffer, " hw=0x"); unsigned_append(&buffer, s.board_faults, 16u, 8u);
    text_append(&buffer, " result="); text_append(&buffer, App_ResultName(s.last_result));
    unsigned_field(&buffer, " owner=", (uint32_t)s.control_source);
    signed_field(&buffer, "\r\nbus_mV=", scaled(s.bus_voltage_v,1000.0f));
    unsigned_field(&buffer, " raw_U/W=", s.raw_u); unsigned_field(&buffer, "/", s.raw_w);
    signed_field(&buffer, " offset_U/W=", scaled(s.offset_u,1.0f));
    signed_field(&buffer, "/", scaled(s.offset_w,1.0f));
    unsigned_field(&buffer, " offset_ok=", (uint32_t)s.offset_ready);
    unsigned_field(&buffer, " sense_ok=", (uint32_t)s.sense_confirmed);
    text_append(&buffer,"\r\n"); text_flush(&buffer);
    unsigned_field(&buffer, "encoder_ok=", (uint32_t)s.sensor_valid);
    signed_field(&buffer, " dir=", (int32_t)s.sensor_direction);
    unsigned_field(&buffer, " dir_ok=", (uint32_t)s.direction_confirmed);
    unsigned_field(&buffer, " zero_ok=", (uint32_t)s.zero_valid);
    signed_field(&buffer, " angle_mrad=", scaled(s.angle_rad,1000.0f));
    signed_field(&buffer, " speed_mrad/s=", scaled(s.velocity_rad_s,1000.0f));
    signed_field(&buffer, "\r\nid_mA=", scaled(s.foc.id_a,1000.0f));
    signed_field(&buffer, " iq_mA=", scaled(s.foc.iq_a,1000.0f));
    signed_field(&buffer, " iq_ref_mA=", scaled(s.foc.iq_reference_a,1000.0f));
    signed_field(&buffer, " vd_mV=", scaled(s.foc.vd_v,1000.0f));
    signed_field(&buffer, " vq_mV=", scaled(s.foc.vq_v,1000.0f));
    unsigned_field(&buffer, "\r\nadc_samples=", s.adc_samples);
    unsigned_field(&buffer, " max_isr_cycles=", s.max_isr_cycles);
    unsigned_field(&buffer, " timeout=", (uint32_t)s.command_timed_out);
    unsigned_field(&buffer, " uart_rx_lost=", Board_UartRxDropped());
    unsigned_field(&buffer, " tx_lost=", Board_UartTxDropped());
    text_append(&buffer,"\r\n"); text_flush(&buffer);
    if (!s.sense_confirmed) Board_UartWrite("Current values unavailable: actual shunts/gains unconfigured.\r\n");
}

static bool parse_number(const char *token, float *value)
{
    float number = 0.0f, place = 0.1f;
    unsigned digits = 0u, fractional_digits = 0u;
    bool negative;
    if (token == NULL) return false;
    negative = *token == '-';
    if (*token == '+' || *token == '-') ++token;
    while (*token >= '0' && *token <= '9') {
        number = number * 10.0f + (float)(*token++ - '0');
        if (number > 1000000.0f) return false;
        ++digits;
    }
    if (*token == '.') {
        ++token;
        while (*token >= '0' && *token <= '9') {
            if (++fractional_digits > 6u) return false;
            number += (float)(*token++ - '0') * place;
            place *= 0.1f; ++digits;
        }
    }
    if (*token != '\0' || digits == 0u) return false;
    *value = negative ? -number : number;
    return true; /* Fixed decimal parser; no strtof/newlib heap dependency. */
}

static void queue(AppCommandKind kind, float value, float auxiliary)
{
    AppResult result;
    if (!rx_input_healthy()) {
        Board_UartWrite("RX_ERROR: command discarded\r\n"); return;
    }
    result = App_Submit(kind, value, auxiliary);
    Board_UartWrite(App_ResultName(result)); Board_UartWrite("\r\n");
    if (result == APP_RESULT_PENDING) previous_result = APP_RESULT_PENDING;
}

static void execute_line(void)
{
    char *tokens[3], *cursor = line;
    unsigned count = 0u;
    float value = 0.0f, auxiliary = MOTOR_OPEN_TEST_VOLTAGE_V;
    /* Avoid strtok's newlib reentrancy allocation: this console is a single
     * foreground parser and only needs in-place space/tab tokenization. */
    while (*cursor != '\0') {
        while (*cursor == ' ' || *cursor == '\t') ++cursor;
        if (*cursor == '\0') break;
        if (count >= 3u) goto syntax;
        tokens[count++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') ++cursor;
        if (*cursor != '\0') *cursor++ = '\0';
    }
    if (count == 0u) return;
    if (count == 1u) {
        if (strcmp(tokens[0],"status") == 0) { print_status(); return; }
        if (strcmp(tokens[0],"help") == 0) { help(); return; }
        if (strcmp(tokens[0],"stop") == 0) { queue(APP_CMD_STOP,0.0f,0.0f); return; }
        if (strcmp(tokens[0],"estop") == 0) { queue(APP_CMD_ESTOP,0.0f,0.0f); return; }
        if (strcmp(tokens[0],"clear") == 0) { queue(APP_CMD_CLEAR,0.0f,0.0f); return; }
        if (strcmp(tokens[0],"cal") == 0) { queue(APP_CMD_CALIBRATE,0.0f,0.0f); return; }
        if (strcmp(tokens[0],"align") == 0) { queue(APP_CMD_ALIGN,0.0f,0.0f); return; }
        goto syntax;
    }
    if (!parse_number(tokens[1], &value)) goto syntax;
    if (strcmp(tokens[0],"open") == 0) {
        if (count == 3u && !parse_number(tokens[2], &auxiliary)) goto syntax;
        queue(APP_CMD_OPEN,value,auxiliary); return;
    }
    if (count != 2u) goto syntax;
    if (strcmp(tokens[0],"iq") == 0) { queue(APP_CMD_CURRENT,value,0.0f); return; }
    if (strcmp(tokens[0],"speed") == 0) { queue(APP_CMD_SPEED,value,0.0f); return; }
    if (strcmp(tokens[0],"dir") == 0) { queue(APP_CMD_DIRECTION,value,0.0f); return; }
    if (strcmp(tokens[0],"bus") == 0) { queue(APP_CMD_BUS,value,0.0f); return; }
syntax:
    Board_UartWrite("SYNTAX: type help; finite numeric arguments only.\r\n");
}

void Console_Init(void)
{
    line_length = 0u; discard_line = false;
    previous_result = APP_RESULT_OK; previous_faults = 0u; previous_zero = false;
    previous_rx_dropped = Board_UartRxDropped();
    Board_UartWrite("\r\nGimbal FOC boot: STOP. motor=" MOTOR_MODEL_NAME ".\r\n"
                    "Offset calibration runs with EN disabled. Type help or status.\r\n");
#if MOTOR_BOOT_AUTO_ALIGN
    Board_UartWrite("POWER-ON ALIGN: after ADC/encoder readiness, apply the checked ALIGN vector once,\r\n"
                    "then return to STOP. Any explicit command cancels a pending start.\r\n");
#endif
#if MOTOR_OPEN_AUTOTEST
    Board_UartWrite("OPEN LOOP TEST IMAGE: power-on delay 2 s, then 0.8 V / 20 electrical rad/s\r\n"
                    "for 2 s, once. Any control command cancels the pending test.\r\n"
                    "Software/watchdog resets inhibit automatic testing; power-cycle to retry.\r\n");
#endif
#if !MOTOR_CURRENT_SENSE_CONFIRMED
    Board_UartWrite("OPEN LOOP ONLY until shunt/gain/polarity are configured.\r\n");
#endif
}

void Console_Poll(void)
{
    char ch;
    unsigned budget = 64u;
    AppStatus s;
    /* A dropped byte must never change a numeric target. Discard until EOL. */
    (void)rx_input_healthy();
    while (budget-- != 0u && Board_UartRead(&ch)) {
        if (!rx_input_healthy()) continue; /* ch may itself be an old, unsafe EOL. */
        if (ch == '\r' || ch == '\n') {
            const bool rejected = discard_line;
            line[line_length] = '\0';
            line_length = 0u; discard_line = false;
            if (rejected) Board_UartWrite("RX_OR_LINE_ERROR: discarded\r\n");
            else execute_line();
        } else if ((ch == '\b' || ch == 127) && !discard_line) {
            if (line_length != 0u) --line_length;
        } else if ((unsigned char)ch >= 32u && (unsigned char)ch <= 126u && !discard_line) {
            if (line_length + 1u < sizeof(line)) line[line_length++] = ch;
            else discard_line = true;
        }
    }
    App_GetStatus(&s);
    if (s.last_result != previous_result) {
        if (s.last_result != APP_RESULT_PENDING) {
            Board_UartWrite(App_ResultName(s.last_result)); Board_UartWrite("\r\n");
        }
        previous_result = s.last_result;
    }
    if (s.faults != previous_faults) {
        if (s.faults != 0u) Board_UartWrite("FAULT latched: driver disabled; type status.\r\n");
        previous_faults = s.faults;
    }
    if (s.zero_valid && !previous_zero) Board_UartWrite("Alignment complete. zero_ok=1; driver stopped.\r\n");
    previous_zero = s.zero_valid;
}
