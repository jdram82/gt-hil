#include "hil_master.h"
#include "modbus_rtu.h"
#include "../include/hil_constants.h"
#include <string.h>

void hil_master_init(HilMaster *m, HilPort port, uint8_t slave)
{
    memset(m, 0, sizeof *m);
    m->port = port;
    m->slave = slave;
    m->timeout_ms = 15;               /* a healthy answer takes about 3 ms at 115200 baud */
    ctrl_init(&m->ctrl);
}

/* One request/response exchange. Returns bytes received. */
static int xfer(HilMaster *m, const uint8_t *req, int rl, uint8_t *resp, int expect)
{
    if (m->port.flush_rx) m->port.flush_rx(m->port.ctx);
    if (m->port.write(m->port.ctx, req, rl) != rl) return 0;
    return m->port.read_frame(m->port.ctx, resp, MB_MAX_FRAME, expect, m->timeout_ms);
}

static void note_error(HilMaster *m, int rc, int got)
{
    if (got == 0)                 m->timeouts++;
    else if (rc == MB_ERR_EXCEPTION) m->exceptions++;
    else                          m->crc_errors++;
}

static int read_regs(HilMaster *m, uint8_t fn, uint16_t start, uint16_t qty, uint16_t *out)
{
    uint8_t rq[MB_MAX_FRAME], rs[MB_MAX_FRAME];
    int rl = mb_make_read(rq, m->slave, fn, start, qty);
    int got = xfer(m, rq, rl, rs, 5 + 2 * qty);
    int rc = got ? mb_check_read(rs, got, m->slave, fn, qty, out) : MB_ERR_SHORT;
    if (rc != MB_OK) note_error(m, rc, got);
    return rc == MB_OK;
}

static void decode_inputs(HilMaster *m)
{
    CtrlInput *in = &m->in;
    for (int i = 0; i < 3; i++) in->speed_rpm[i] = hil_dec(m->ir[HIL_IR_SPEED1_RPM + i], HIL_SCALE_SPEED);
    for (int i = 0; i < 4; i++) in->tc_c[i]      = hil_dec(m->ir[HIL_IR_TC1 + i], HIL_SCALE_TEMP);
    uint16_t d = m->ir[HIL_IR_DISCRETE];
    in->breaker_closed = (d & HIL_DI_BREAKER_CLOSED) != 0;
    in->estop_ok       = (d & HIL_DI_ESTOP_HEALTHY) != 0;
    in->flame          = (d & HIL_DI_FLAME) != 0;
    in->lube_ok        = (d & HIL_DI_LUBE_OK) != 0;
    in->fuel_ok        = (d & HIL_DI_FUEL_OK) != 0;
    in->starter_ok     = (d & HIL_DI_STARTER_OK) != 0;
    in->start_cmd      = m->hr_op[HIL_HR_START_CMD] != 0;
    in->reset_cmd      = m->hr_op[HIL_HR_RESET_CMD] != 0;
    in->droop_mode     = m->hr_op[HIL_HR_MODE] != 0;
    m->last_plant_hb   = m->ir[HIL_IR_PLANT_HB];
}

int hil_master_cycle(HilMaster *m)
{
    uint32_t t0 = m->port.now_us(m->port.ctx);
    int ok = read_regs(m, MB_FN_READ_INPUT, 0, HIL_IR_COUNT, m->ir);
    if (ok) ok = read_regs(m, MB_FN_READ_HOLDING, HIL_HR_START_CMD, 3, m->hr_op);

    if (ok) decode_inputs(m);
    m->in.link_ok = ok;

    uint32_t t1 = m->port.now_us(m->port.ctx);
    ctrl_update(&m->ctrl, HIL_CONTROL_PERIOD_S, &m->in);
    uint32_t t2 = m->port.now_us(m->port.ctx);
    m->last_ctrl_us = t2 - t1;
    if (m->last_ctrl_us > m->max_ctrl_us) m->max_ctrl_us = m->last_ctrl_us;
    (void)t1;

    if (ok) {
        const StandinCtrl *c = &m->ctrl;
        uint16_t out[6], dw = 0;
        if (c->starter_on)   dw |= HIL_DO_STARTER;
        if (c->ignition_on)  dw |= HIL_DO_IGNITION;
        if (c->sync_permit)  dw |= HIL_DO_SYNC_PERMIT;
        if (!c->trip_out)    dw |= HIL_DO_TRIP_OK;             /* de-energise to trip */
        out[0] = hil_enc(c->fsr_out, HIL_SCALE_FSR);
        out[1] = dw;
        out[2] = ++m->ctrl_hb;
        out[3] = (uint16_t)c->state;
        out[4] = (uint16_t)c->first_out;
        out[5] = (uint16_t)(c->alarm_word & 0xFFFFu);

        uint8_t rq[MB_MAX_FRAME], rs[MB_MAX_FRAME];
        int rl = mb_make_write_multiple(rq, m->slave, HIL_HR_FUEL_CMD, 6, out);
        int got = xfer(m, rq, rl, rs, 8);
        int rc = got ? mb_check_write(rs, got, m->slave, HIL_HR_FUEL_CMD, 6) : MB_ERR_SHORT;
        if (rc != MB_OK) { m->write_failures++; note_error(m, rc, got); }
    }

    m->cycles++;
    if (!ok) m->failed_cycles++;
    m->last_cycle_us = m->port.now_us(m->port.ctx) - t0;
    if (m->last_cycle_us > m->max_cycle_us) m->max_cycle_us = m->last_cycle_us;
    return ok;
}
