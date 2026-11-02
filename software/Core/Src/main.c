#include "board.h"
#include "app.h"
#include "console.h"
#include "can_service.h"

int main(void)
{
    if (!Board_Init() || !App_Init() || !Board_StartSampling()) {
        Board_SetDriverEnabled(false);
        for (;;) { /* Board's init timeout handler keeps the driver disabled. */ }
    }
    /* Optional communication hardware must not prevent local first-run tests. */
    CanService_Init();
    Console_Init();
    for (;;) {
        CanService_Poll();
        App_Poll();
        CanService_Poll(); /* Inspect transport and targets after the I2C read. */
        Console_Poll();
    }
}
