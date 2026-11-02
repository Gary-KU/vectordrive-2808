#include "node_id_store.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint16_t memory[2][NODE_ID_STORE_PAGE_BYTES / 2u];
static unsigned programs, erases;
static int fail_program;
static bool fail_erase;

static uint16_t read_word(void *context, unsigned page, unsigned word)
{
    (void)context; assert(page < 2u && word < NODE_ID_STORE_PAGE_BYTES / 2u);
    return memory[page][word];
}
static bool erase_page(void *context, unsigned page)
{
    (void)context; ++erases;
    if (fail_erase) {
        for (unsigned n = 0u; n < NODE_ID_STORE_PAGE_BYTES / 4u; ++n) memory[page][n] = 0xffffu;
        return false; /* Power loss midway through erasing the old page. */
    }
    memset(memory[page], 0xff, NODE_ID_STORE_PAGE_BYTES);
    return true;
}
static bool program_word(void *context, unsigned page, unsigned word, uint16_t value)
{
    (void)context;
    if (fail_program >= 0 && programs++ == (unsigned)fail_program) return false;
    assert(memory[page][word] == 0xffffu);
    memory[page][word] = value;
    return true;
}
static const NodeIdStoreIo io = {0, read_word, erase_page, program_word};

static NodeIdStoreState fresh(void)
{
    NodeIdStoreState state;
    memset(memory, 0xff, sizeof(memory));
    programs = erases = 0u; fail_program = -1; fail_erase = false;
    NodeIdStore_Load(&io, 1u, &state);
    assert(!state.valid && !state.storage_error && state.node_id == 1u);
    return state;
}

static void test_append_and_noop(void)
{
    NodeIdStoreState state = fresh(), boot;
    assert(!NodeIdStore_Save(&io, 0u, &state));
    assert(!NodeIdStore_Save(&io, 64u, &state));
    assert(NodeIdStore_Save(&io, 2u, &state));
    assert(state.valid && state.node_id == 2u && state.generation == 1u);
    NodeIdStore_Load(&io, 1u, &boot);
    assert(boot.valid && boot.node_id == 2u);
    uint16_t before[2][NODE_ID_STORE_PAGE_BYTES / 2u];
    memcpy(before, memory, sizeof(memory));
    assert(NodeIdStore_Save(&io, 2u, &state));
    assert(memcmp(before, memory, sizeof(memory)) == 0 && erases == 0u);
    assert(NodeIdStore_Save(&io, 63u, &state));
    NodeIdStore_Load(&io, 1u, &boot);
    assert(boot.node_id == 63u && boot.generation == 2u);
}

static void test_power_cut_every_halfword(void)
{
    for (int cut = 0; cut < 8; ++cut) {
        NodeIdStoreState state = fresh(), boot;
        assert(NodeIdStore_Save(&io, 5u, &state));
        programs = 0u; fail_program = cut;
        assert(!NodeIdStore_Save(&io, 9u, &state));
        NodeIdStore_Load(&io, 1u, &boot);
        assert(boot.valid && boot.node_id == 5u && boot.generation == 1u);
        fail_program = -1;
        assert(NodeIdStore_Save(&io, 9u, &boot));
        NodeIdStore_Load(&io, 1u, &state);
        assert(state.node_id == 9u);
    }
    /* Interrupted very first assignment returns the configured default. */
    NodeIdStoreState state = fresh(), boot;
    fail_program = 7;
    assert(!NodeIdStore_Save(&io, 4u, &state));
    NodeIdStore_Load(&io, 7u, &boot);
    assert(!boot.valid && boot.storage_error && boot.node_id == 7u);
}

static void fill_page(NodeIdStoreState *state)
{
    for (unsigned n = 0u; n < NODE_ID_STORE_RECORDS; ++n)
        assert(NodeIdStore_Save(&io, (uint8_t)(n % 2u + 2u), state));
    assert(state->generation == NODE_ID_STORE_RECORDS && state->page == 0u);
}

static void test_page_rollover_power_cut_and_corruption(void)
{
    for (int cut = 0; cut < 8; ++cut) {
        NodeIdStoreState state = fresh(), boot;
        fill_page(&state);
        uint8_t old = state.node_id;
        programs = 0u; fail_program = cut;
        assert(!NodeIdStore_Save(&io, 12u, &state));
        NodeIdStore_Load(&io, 1u, &boot);
        assert(boot.valid && boot.node_id == old && boot.generation == NODE_ID_STORE_RECORDS);
    }
    NodeIdStoreState state = fresh(), boot;
    fill_page(&state);
    fail_erase = true;
    assert(!NodeIdStore_Save(&io, 12u, &state));
    NodeIdStore_Load(&io, 1u, &boot);
    assert(boot.valid && boot.node_id == state.node_id && boot.page == 0u);
    fail_erase = false;
    assert(NodeIdStore_Save(&io, 12u, &state) && state.page == 1u);
    memory[1][2] ^= 0x40u; /* Damage generation; checksum/inverse reject it. */
    NodeIdStore_Load(&io, 1u, &boot);
    assert(boot.node_id != 12u && boot.page == 0u && boot.generation == NODE_ID_STORE_RECORDS);
    for (unsigned n = 0u; n < 300u; ++n)
        assert(NodeIdStore_Save(&io, (uint8_t)(n % 62u + 1u), &boot));
    NodeIdStore_Load(&io, 1u, &state);
    assert(state.node_id == boot.node_id && state.generation == boot.generation);
    /* Reclaim a page that really contains older committed records, and lose
     * power halfway through that erase. The newest page must remain valid. */
    state = fresh();
    for (unsigned n = 0u; n < NODE_ID_STORE_RECORDS * 2u; ++n)
        assert(NodeIdStore_Save(&io, (uint8_t)(n % 2u + 2u), &state));
    assert(state.page == 1u && state.generation == NODE_ID_STORE_RECORDS * 2u);
    fail_erase = true;
    assert(!NodeIdStore_Save(&io, 12u, &state));
    NodeIdStore_Load(&io,1u,&boot);
    assert(boot.valid && boot.page == 1u && boot.generation == NODE_ID_STORE_RECORDS * 2u && boot.node_id == state.node_id);
}

static uint16_t test_crc(const uint16_t *words)
{
    unsigned crc = 0xffffu;
    for (unsigned byte = 0u; byte < 12u; ++byte) {
        crc ^= ((words[byte/2u] >> (8u*(byte%2u))) & 255u) << 8;
        for (unsigned bit = 0u; bit < 8u; ++bit)
            crc = ((crc & 0x8000u) != 0u ? (crc << 1) ^ 0x1021u : crc << 1) & 0xffffu;
    }
    return (uint16_t)crc;
}

static void test_generation_wrap(void)
{
    NodeIdStoreState state = fresh(), boot;
    uint16_t *words = memory[0];
    words[0]=0x4e43u; words[1]=0x0102u;
    words[2]=0xfffeu; words[3]=0xffffu;
    words[4]=1u; words[5]=0u;
    words[6]=test_crc(words); words[7]=0xa55au;
    NodeIdStore_Load(&io,1u,&state);
    assert(state.valid && state.generation == 0xfffffffeu);
    assert(NodeIdStore_Save(&io,3u,&state) && state.generation == 0xffffffffu);
    assert(NodeIdStore_Save(&io,4u,&state) && state.generation == 0u);
    NodeIdStore_Load(&io,1u,&boot);
    assert(boot.valid && boot.node_id == 4u && boot.generation == 0u);
}

int main(void)
{
    test_append_and_noop();
    test_power_cut_every_halfword();
    test_page_rollover_power_cut_and_corruption();
    test_generation_wrap();
    puts("CAN node ID journal: power cuts, corruption, rollover and wear tests passed.");
    return 0;
}
