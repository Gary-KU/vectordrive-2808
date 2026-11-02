#include "can_service.h"
#include "app.h"
#include "board.h"
#include "board_can.h"
#include "can_config.h"
#include "can_protocol.h"

#include <stdbool.h>
#include <stdint.h>

static bool initialized;
static uint32_t last_init_attempt_ms;
/* Retained for SWD diagnostics, including failed startup attempts. */
static volatile uint32_t init_attempts, init_successes;

static bool try_initialize(void)
{
    bool ready;
    ++init_attempts;
    ready = BoardCan_Init(); /* Bounded hardware wait; ADC interrupts stay enabled. */
    last_init_attempt_ms = Board_Millis();
    if (ready) ++init_successes;
    return ready;
}

void CanService_Init(void)
{
    initialized = false;
    init_attempts = init_successes = 0u;
    (void)try_initialize();
    CanProtocol_Init();
    initialized = true;
}

void CanService_Poll(void)
{
    BoardCanStatus transport;
    if (!initialized) return;
    transport = BoardCan_GetStatus();
    if (!transport.initialized &&
        (uint32_t)(Board_Millis() - last_init_attempt_ms) >= CAN_INIT_RETRY_PERIOD_MS &&
        App_CanReinitializeCan()) {
        if (try_initialize()) {
            /* Discard stale RX/replies and forget the old handshake. Nothing
             * here submits CLEAR, ALIGN or motion, or enables the driver.
             */
            CanProtocol_Init();
        }
    }
    CanProtocol_Poll();
}
