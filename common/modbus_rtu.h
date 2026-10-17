/*
 * modbus_rtu.h - minimal Modbus RTU, master and slave sides, no hardware calls.
 *
 * Functions: 03 read holding, 04 read input, 16 write multiple holding.
 * Frames are plain byte buffers, so the same code runs on the BeagleBone
 * (Linux), the STM32 (HAL) and in host unit tests.
 */
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MB_MAX_FRAME   256
#define MB_FN_READ_HOLDING   0x03
#define MB_FN_READ_INPUT     0x04
#define MB_FN_WRITE_MULTIPLE 0x10

#define MB_EX_ILLEGAL_FUNCTION 0x01
#define MB_EX_ILLEGAL_ADDRESS  0x02
#define MB_EX_ILLEGAL_VALUE    0x03

/* result codes of the master-side checkers */
enum { MB_OK = 0, MB_ERR_SHORT = -1, MB_ERR_CRC = -2, MB_ERR_ADDR = -3,
       MB_ERR_FUNC = -4, MB_ERR_EXCEPTION = -5, MB_ERR_LEN = -6 };

uint16_t mb_crc16(const uint8_t *buf, int len);

/* ---------------- master ---------------- */
int mb_make_read(uint8_t *buf, uint8_t slave, uint8_t fn, uint16_t start, uint16_t qty);
int mb_make_write_multiple(uint8_t *buf, uint8_t slave, uint16_t start, uint16_t qty, const uint16_t *vals);
/* Validate a read response and copy qty registers to out. Returns MB_OK or error. */
int mb_check_read(const uint8_t *resp, int len, uint8_t slave, uint8_t fn, uint16_t qty, uint16_t *out);
int mb_check_write(const uint8_t *resp, int len, uint8_t slave, uint16_t start, uint16_t qty);

/* ---------------- slave ---------------- */
typedef struct {
    uint8_t   slave;
    uint16_t *ir;   int n_ir;      /* input registers   (read-only for the master) */
    uint16_t *hr;   int n_hr;      /* holding registers */
    void *ctx;
    void (*before_read)(void *ctx, uint8_t fn, uint16_t start, uint16_t qty);   /* refresh values */
    void (*after_write)(void *ctx, uint16_t start, uint16_t qty);               /* act on written values */
} MbSlave;

/* Handle one received frame. Returns response length, or 0 when no response is
 * to be sent (bad CRC, wrong slave address). */
int mb_slave_handle(MbSlave *s, const uint8_t *req, int len, uint8_t *resp);

#ifdef __cplusplus
}
#endif
#endif
