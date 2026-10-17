#include "modbus_rtu.h"
#include <string.h>

uint16_t mb_crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

static int finish(uint8_t *buf, int n)
{
    uint16_t c = mb_crc16(buf, n);
    buf[n++] = (uint8_t)(c & 0xFF);          /* CRC is sent low byte first */
    buf[n++] = (uint8_t)(c >> 8);
    return n;
}

/* ------------------------------ master ------------------------------ */
int mb_make_read(uint8_t *b, uint8_t slave, uint8_t fn, uint16_t start, uint16_t qty)
{
    b[0] = slave; b[1] = fn;
    b[2] = (uint8_t)(start >> 8); b[3] = (uint8_t)start;
    b[4] = (uint8_t)(qty >> 8);   b[5] = (uint8_t)qty;
    return finish(b, 6);
}

int mb_make_write_multiple(uint8_t *b, uint8_t slave, uint16_t start, uint16_t qty, const uint16_t *v)
{
    int n = 0;
    b[n++] = slave; b[n++] = MB_FN_WRITE_MULTIPLE;
    b[n++] = (uint8_t)(start >> 8); b[n++] = (uint8_t)start;
    b[n++] = (uint8_t)(qty >> 8);   b[n++] = (uint8_t)qty;
    b[n++] = (uint8_t)(2 * qty);
    for (int i = 0; i < qty; i++) { b[n++] = (uint8_t)(v[i] >> 8); b[n++] = (uint8_t)v[i]; }
    return finish(b, n);
}

static int check_common(const uint8_t *r, int len, uint8_t slave, uint8_t fn)
{
    if (len < 5) return MB_ERR_SHORT;
    uint16_t c = mb_crc16(r, len - 2);
    if (r[len - 2] != (uint8_t)(c & 0xFF) || r[len - 1] != (uint8_t)(c >> 8)) return MB_ERR_CRC;
    if (r[0] != slave) return MB_ERR_ADDR;
    if (r[1] == (uint8_t)(fn | 0x80)) return MB_ERR_EXCEPTION;
    if (r[1] != fn) return MB_ERR_FUNC;
    return MB_OK;
}

int mb_check_read(const uint8_t *r, int len, uint8_t slave, uint8_t fn, uint16_t qty, uint16_t *out)
{
    int e = check_common(r, len, slave, fn);
    if (e) return e;
    if (r[2] != 2 * qty || len != 5 + 2 * qty) return MB_ERR_LEN;
    for (int i = 0; i < qty; i++) out[i] = (uint16_t)((r[3 + 2 * i] << 8) | r[4 + 2 * i]);
    return MB_OK;
}

int mb_check_write(const uint8_t *r, int len, uint8_t slave, uint16_t start, uint16_t qty)
{
    int e = (len < 8) ? MB_ERR_SHORT : check_common(r, len, slave, MB_FN_WRITE_MULTIPLE);
    if (e) return e;
    if (len != 8) return MB_ERR_LEN;
    if (r[2] != (uint8_t)(start >> 8) || r[3] != (uint8_t)start ||
        r[4] != (uint8_t)(qty >> 8)   || r[5] != (uint8_t)qty) return MB_ERR_LEN;
    return MB_OK;
}

/* ------------------------------ slave ------------------------------- */
static int exception(uint8_t *resp, uint8_t slave, uint8_t fn, uint8_t code)
{
    resp[0] = slave; resp[1] = (uint8_t)(fn | 0x80); resp[2] = code;
    return finish(resp, 3);
}

int mb_slave_handle(MbSlave *s, const uint8_t *req, int len, uint8_t *resp)
{
    if (len < 8) return 0;
    uint16_t c = mb_crc16(req, len - 2);
    if (req[len - 2] != (uint8_t)(c & 0xFF) || req[len - 1] != (uint8_t)(c >> 8)) return 0;
    if (req[0] != s->slave) return 0;

    uint8_t  fn    = req[1];
    uint16_t start = (uint16_t)((req[2] << 8) | req[3]);
    uint16_t qty   = (uint16_t)((req[4] << 8) | req[5]);

    if (fn == MB_FN_READ_HOLDING || fn == MB_FN_READ_INPUT) {
        uint16_t *reg = (fn == MB_FN_READ_INPUT) ? s->ir : s->hr;
        int       n   = (fn == MB_FN_READ_INPUT) ? s->n_ir : s->n_hr;
        if (len != 8) return 0;
        if (qty < 1 || qty > 125) return exception(resp, s->slave, fn, MB_EX_ILLEGAL_VALUE);
        if ((int)start + qty > n) return exception(resp, s->slave, fn, MB_EX_ILLEGAL_ADDRESS);
        if (s->before_read) s->before_read(s->ctx, fn, start, qty);
        int k = 0;
        resp[k++] = s->slave; resp[k++] = fn; resp[k++] = (uint8_t)(2 * qty);
        for (int i = 0; i < qty; i++) { resp[k++] = (uint8_t)(reg[start + i] >> 8); resp[k++] = (uint8_t)reg[start + i]; }
        return finish(resp, k);
    }
    if (fn == MB_FN_WRITE_MULTIPLE) {
        if (len < 11 || req[6] != 2 * qty || len != 9 + 2 * qty) return exception(resp, s->slave, fn, MB_EX_ILLEGAL_VALUE);
        if (qty < 1 || qty > 123) return exception(resp, s->slave, fn, MB_EX_ILLEGAL_VALUE);
        if ((int)start + qty > s->n_hr) return exception(resp, s->slave, fn, MB_EX_ILLEGAL_ADDRESS);
        for (int i = 0; i < qty; i++) s->hr[start + i] = (uint16_t)((req[7 + 2 * i] << 8) | req[8 + 2 * i]);
        if (s->after_write) s->after_write(s->ctx, start, qty);
        memcpy(resp, req, 6);
        return finish(resp, 6);
    }
    return exception(resp, s->slave, fn, MB_EX_ILLEGAL_FUNCTION);
}
