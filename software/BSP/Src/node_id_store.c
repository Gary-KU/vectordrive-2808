#include "node_id_store.h"

#define RECORD_MAGIC 0x4e43u
#define RECORD_COMMIT 0xa55au
#define RECORD_VERSION 1u
#define RECORD_HALFWORDS (NODE_ID_STORE_RECORD_BYTES / 2u)

static uint16_t crc16(const uint16_t *words)
{
    uint16_t crc = 0xffffu;
    for (unsigned byte = 0u; byte < 12u; ++byte) {
        crc ^= (uint16_t)(((words[byte / 2u] >> ((byte % 2u) * 8u)) & 0xffu) << 8);
        for (unsigned bit = 0u; bit < 8u; ++bit)
            crc = (uint16_t)((crc & 0x8000u) != 0u ? ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1);
    }
    return crc;
}

static bool read_record(const NodeIdStoreIo *io, unsigned page, unsigned slot,
                        uint16_t *words)
{
    bool erased = true;
    for (unsigned n = 0u; n < RECORD_HALFWORDS; ++n) {
        words[n] = io->read(io->context, page, slot * RECORD_HALFWORDS + n);
        if (words[n] != 0xffffu) erased = false;
    }
    return erased;
}

void NodeIdStore_Load(const NodeIdStoreIo *io, uint8_t default_id, NodeIdStoreState *state)
{
    uint16_t words[RECORD_HALFWORDS];
    bool dirty = false;
    *state = (NodeIdStoreState){default_id, false, false, 0u, 0u};
    for (unsigned page = 0u; page < 2u; ++page) {
        for (unsigned slot = 0u; slot < NODE_ID_STORE_RECORDS; ++slot) {
            uint32_t generation, inverse;
            uint8_t id;
            if (read_record(io, page, slot, words)) continue;
            dirty = true;
            id = (uint8_t)words[1];
            generation = (uint32_t)words[2] | ((uint32_t)words[3] << 16);
            inverse = (uint32_t)words[4] | ((uint32_t)words[5] << 16);
            if (words[0] != RECORD_MAGIC || (words[1] >> 8) != RECORD_VERSION ||
                id < 1u || id > 63u || inverse != ~generation ||
                words[6] != crc16(words) || words[7] != RECORD_COMMIT) continue;
            if (!state->valid || (int32_t)(generation - state->generation) > 0) {
                state->node_id = id;
                state->valid = true;
                state->generation = generation;
                state->page = page;
            }
        }
    }
    state->storage_error = dirty && !state->valid;
}

bool NodeIdStore_Save(const NodeIdStoreIo *io, uint8_t node_id, NodeIdStoreState *state)
{
    uint16_t words[RECORD_HALFWORDS], scratch[RECORD_HALFWORDS];
    unsigned page, slot;
    uint32_t generation;
    NodeIdStoreState loaded;
    if (node_id < 1u || node_id > 63u) return false;
    if (state->valid && state->node_id == node_id) return true; /* No wear for retries. */
    page = state->valid ? state->page : 0u;
    for (slot = 0u; slot < NODE_ID_STORE_RECORDS; ++slot)
        if (read_record(io, page, slot, scratch)) break;
    if (slot == NODE_ID_STORE_RECORDS) {
        /* The latest committed record stays intact throughout the other page's
         * erase/program operation. No post-commit erase is necessary. */
        page = state->valid ? 1u - state->page : 1u;
        if (!io->erase(io->context, page)) return false;
        for (unsigned n = 0u; n < NODE_ID_STORE_PAGE_BYTES / 2u; ++n)
            if (io->read(io->context, page, n) != 0xffffu) return false;
        slot = 0u;
    }
    generation = state->valid ? state->generation + 1u : 1u;
    words[0] = RECORD_MAGIC;
    words[1] = (uint16_t)((RECORD_VERSION << 8) | node_id);
    words[2] = (uint16_t)generation; words[3] = (uint16_t)(generation >> 16);
    words[4] = (uint16_t)~generation; words[5] = (uint16_t)(~generation >> 16);
    words[6] = crc16(words); words[7] = RECORD_COMMIT;
    for (unsigned n = 0u; n < RECORD_HALFWORDS; ++n)
        if (!io->program(io->context, page, slot * RECORD_HALFWORDS + n, words[n])) break;
    NodeIdStore_Load(io, 1u, &loaded);
    if (!loaded.valid || loaded.node_id != node_id || loaded.generation != generation) {
        state->storage_error = true;
        return false;
    }
    *state = loaded;
    return true;
}
