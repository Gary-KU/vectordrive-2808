#ifndef GIMBAL_BOARD_CAN_H
#define GIMBAL_BOARD_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;
    uint32_t received_ms;
    uint8_t data[8];
    uint8_t dlc;
    bool extended;
    bool remote;
} BoardCanFrame;

typedef struct {
    bool initialized;
    bool bus_off;
    bool error_passive;
    uint32_t rx_dropped; /* RX ring drops + hardware FIFO overrun events. */
    uint32_t tx_dropped; /* Invalid/full/bus-off requests + failed/aborted TX. */
    uint32_t rx_count;   /* Frames removed from hardware FIFO. */
    uint32_t tx_count;   /* TXOK completions, not merely mailbox requests. */
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint8_t last_error_code;
    uint32_t rx_fifo_full_count;
    uint32_t rx_fifo_overrun_count;
    uint32_t bus_off_count; /* Cumulative event counter survives ABOM recovery. */
    uint32_t tx_timeout_count;
} BoardCanStatus;

/* Call after Board_Init (APB1=36 MHz); CanService may retry a failed init only
 * under the application's stopped maintenance gate. An initialized peripheral
 * is left untouched. Init waits are bounded and do not disable ADC interrupts.
 * A CAN init error does not latch the motor BSP fault: disconnected CAN must
 * not disable local UART operation.
 * PA11 = CAN_RX, PA12 = CAN_TX, normal mode, ABOM recovery, hardware retry.
 */
bool BoardCan_Init(void);
bool BoardCan_Read(BoardCanFrame *frame);
/* Nonblocking, foreground only. Three hardware mailboxes; no software TX queue.
 * Return true means accepted by a mailbox. Delivery still requires a CAN ACK.
 */
bool BoardCan_Send(const BoardCanFrame *frame);
void BoardCan_DiscardRx(void);
/* Foreground only. Also polls current ESR and completed/failed transmissions.
 * A bus-off event aborts all pending TX; motor recovery is application policy.
 * Call at least every 5 ms to expire FAST TX (other mailboxes at most 20 ms).
 * This also bounds retries on a stuck dominant/unacknowledged bus.
 * Software must also watch bus_off_count: ABOM can recover between polls.
 */
BoardCanStatus BoardCan_GetStatus(void);

enum {
    BOARD_NODE_STORED = 1u << 0,
    BOARD_NODE_STORAGE_ERROR = 1u << 1,
    BOARD_NODE_RESET_PENDING = 1u << 2
};
typedef struct {
    uint8_t active_id;
    uint8_t stored_id;
    uint8_t flags;
} BoardCanNodeInfo;

uint8_t BoardCan_GetNodeId(void);
BoardCanNodeInfo BoardCan_GetNodeInfo(void);
/* Load once at boot before the hardware filter is initialized. */
void BoardCan_LoadNodeId(void);
/* Foreground stopped-maintenance only; production never returns. Both frames
 * are sent at the OLD IDs after actual flash verification, then reset follows
 * even if a peer does not ACK. Caller supplies opcode/sequence and result is
 * filled here (OK or CAN_RESULT_STORAGE). */
void BoardCan_CommitNodeIdAndReset(uint8_t new_id, BoardCanFrame ack, BoardCanFrame info);
/* Terminal-maintenance helper; IRQs already disabled, DWT still running. */
bool BoardCan_SendConfigurationReply(const BoardCanFrame *frame, uint32_t timeout_cycles);

#ifdef __cplusplus
}
#endif
#endif
