#include "calibration_store.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint16_t memory[2][CALIBRATION_STORE_PAGE_BYTES / 2u];
static unsigned programs, erases;
static int fail_program;
static bool fail_erase;

static uint16_t read_word(void *context, unsigned page, unsigned word)
{
    (void)context; assert(page < 2u && word < CALIBRATION_STORE_PAGE_BYTES / 2u);
    return memory[page][word];
}
static bool erase_page(void *context, unsigned page)
{
    (void)context; ++erases;
    if (fail_erase) {
        for (unsigned n = 0u; n < CALIBRATION_STORE_PAGE_BYTES / 4u; ++n)
            memory[page][n] = 0xffffu;
        return false;
    }
    memset(memory[page], 0xff, CALIBRATION_STORE_PAGE_BYTES); return true;
}
static bool program_word(void *context, unsigned page, unsigned word, uint16_t value)
{
    (void)context;
    if (fail_program >= 0 && programs++ == (unsigned)fail_program) return false;
    assert(memory[page][word] == 0xffffu); memory[page][word] = value; return true;
}
static const CalibrationStoreIo io = {0, read_word, erase_page, program_word};

static CalibrationStoreState fresh(void)
{
    CalibrationStoreState state;
    memset(memory, 0xff, sizeof(memory)); programs = erases = 0u;
    fail_program = -1; fail_erase = false;
    CalibrationStore_Load(&io, &state);
    assert(!state.has_record && !state.valid && !state.storage_error);
    return state;
}

static void test_save_load_noop_and_clear(void)
{
    CalibrationStoreState state = fresh(), boot;
    assert(!CalibrationStore_Save(&io, 0u, 7u, 1, 1.0f, &state));
    assert(!CalibrationStore_Save(&io, 2808u, 0u, 1, 1.0f, &state));
    assert(!CalibrationStore_Save(&io, 2808u, 7u, 0, 1.0f, &state));
    assert(!CalibrationStore_Save(&io, 2808u, 7u, 1, -1.0f, &state));
    assert(CalibrationStore_Save(&io, 2808u, 7u, 1, 1.25f, &state));
    assert(state.valid && state.has_record && state.generation == 1u);
    CalibrationStore_Load(&io, &boot);
    assert(boot.valid && boot.motor_model == 2808u && boot.pole_pairs == 7u &&
           boot.sensor_direction == 1 && boot.electrical_zero_rad == 1.25f);
    unsigned before = programs;
    assert(CalibrationStore_Save(&io, 2808u, 7u, 1, 1.25f, &boot));
    assert(programs == before);
    assert(CalibrationStore_Clear(&io, &boot));
    CalibrationStore_Load(&io, &state);
    assert(state.has_record && !state.valid && !state.storage_error && state.generation == 2u);
    before = programs; assert(CalibrationStore_Clear(&io, &state)); assert(programs == before);
}

static void test_power_cut_preserves_previous_record(void)
{
    for (int cut = 0; cut < 12; ++cut) {
        CalibrationStoreState state = fresh(), boot;
        assert(CalibrationStore_Save(&io, 2808u, 7u, 1, 0.75f, &state));
        programs = 0u; fail_program = cut;
        assert(!CalibrationStore_Save(&io, 2808u, 7u, -1, 2.0f, &state));
        CalibrationStore_Load(&io, &boot);
        assert(boot.valid && boot.generation == 1u && boot.sensor_direction == 1 &&
               boot.electrical_zero_rad == 0.75f);
    }
    CalibrationStoreState state = fresh(), boot;
    fail_program = 11;
    assert(!CalibrationStore_Save(&io, 4015u, 11u, 1, 1.0f, &state));
    CalibrationStore_Load(&io, &boot);
    assert(!boot.has_record && !boot.valid && boot.storage_error);
}

static void fill_page(CalibrationStoreState *state)
{
    for (unsigned n = 0u; n < CALIBRATION_STORE_RECORDS; ++n)
        assert(CalibrationStore_Save(&io, 2808u, 7u, n & 1u ? 1 : -1,
                                     0.5f + (float)n * 0.01f, state));
    assert(state->page == 0u && state->generation == CALIBRATION_STORE_RECORDS);
}

static void test_rollover_and_corruption(void)
{
    CalibrationStoreState state = fresh(), boot;
    fill_page(&state);
    fail_erase = true;
    assert(!CalibrationStore_Save(&io, 4015u, 11u, 1, 2.0f, &state));
    CalibrationStore_Load(&io, &boot);
    assert(boot.valid && boot.page == 0u && boot.generation == CALIBRATION_STORE_RECORDS);
    fail_erase = false;
    assert(CalibrationStore_Save(&io, 4015u, 11u, 1, 2.0f, &boot));
    assert(boot.page == 1u && boot.generation == CALIBRATION_STORE_RECORDS + 1u);
    memory[1][4] ^= 0x20u;
    CalibrationStore_Load(&io, &state);
    assert(state.valid && state.page == 0u && state.generation == CALIBRATION_STORE_RECORDS);
}

int main(void)
{
    test_save_load_noop_and_clear();
    test_power_cut_preserves_previous_record();
    test_rollover_and_corruption();
    puts("Motor calibration journal: save/clear, CRC and power-cut tests passed.");
    return 0;
}
