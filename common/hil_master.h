/*
 * hil_master.h - the controller application: one 50 ms cycle = poll the plant,
 * run the controller, write the commands back.  Hardware-independent: the
 * serial port and a microsecond clock are supplied through HilPort, so the very
 * same file runs on the STM32 (HAL), on Linux (ctrl_node) and in tests.
 *
 *   1. read input registers 0..11   (sensors, discretes, plant heartbeat)
 *   2. read holding registers 0..2  (operator: start, reset, mode)
 *   3. ctrl_update()
 *   4. write holding registers 3..8 (fuel, DO word, heartbeat, state, first-out, alarms)
 *
 * If step 1 or 2 fails (timeout, bad CRC) the cycle counts as a failed poll:
 * the controller is told link_ok = 0 and holds its outputs; step 4 is skipped.
 */
#ifndef HIL_MASTER_H
#define HIL_MASTER_H

#include <stdint.h>
#include "../controller/standin_governor.h"
#include "../include/hil_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void *ctx;
    int      (*write)(void *ctx, const uint8_t *buf, int len);
    /* Receive one frame: return when `expect` bytes have arrived, or the line has
     * gone idle after at least one byte, or timeout_ms passed. Returns bytes read. */
    int      (*read_frame)(void *ctx, uint8_t *buf, int max, int expect, uint32_t timeout_ms);
    void     (*flush_rx)(void *ctx);          /* drop stale bytes before a request */
    uint32_t (*now_us)(void *ctx);            /* free-running, wraps at 2^32 */
} HilPort;

typedef struct {
    HilPort    port;
    uint8_t    slave;
    uint32_t   timeout_ms;
    StandinCtrl ctrl;
    CtrlInput   in;
    uint16_t   ir[HIL_IR_COUNT];
    uint16_t   hr_op[3];
    uint16_t   ctrl_hb;

    /* statistics */
    uint32_t cycles, failed_cycles, crc_errors, timeouts, write_failures, exceptions;
    uint32_t last_cycle_us, max_cycle_us, last_ctrl_us, max_ctrl_us;
    uint16_t last_plant_hb;
} HilMaster;

void hil_master_init(HilMaster *m, HilPort port, uint8_t slave);
/* Run one cycle. Returns 1 when the poll succeeded, 0 when it failed. */
int  hil_master_cycle(HilMaster *m);

#ifdef __cplusplus
}
#endif
#endif
