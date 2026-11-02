#ifndef GIMBAL_NODE_ID_STORE_H
#define GIMBAL_NODE_ID_STORE_H

#include <stdbool.h>
#include <stdint.h>

/* Two 1 KiB pages, append-only 16-byte records. The last halfword is the
 * commit marker; CRC covers version, ID, generation and its inverse. */
#define NODE_ID_STORE_PAGE_BYTES 1024u
#define NODE_ID_STORE_RECORD_BYTES 16u
#define NODE_ID_STORE_RECORDS (NODE_ID_STORE_PAGE_BYTES / NODE_ID_STORE_RECORD_BYTES)

typedef struct {
    void *context;
    uint16_t (*read)(void *context, unsigned page, unsigned halfword);
    bool (*erase)(void *context, unsigned page);
    bool (*program)(void *context, unsigned page, unsigned halfword, uint16_t value);
} NodeIdStoreIo;

typedef struct {
    uint8_t node_id;
    bool valid;
    bool storage_error;
    uint32_t generation;
    unsigned page;
} NodeIdStoreState;

void NodeIdStore_Load(const NodeIdStoreIo *io, uint8_t default_id, NodeIdStoreState *state);
bool NodeIdStore_Save(const NodeIdStoreIo *io, uint8_t node_id, NodeIdStoreState *state);

#endif
