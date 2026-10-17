/* test_modbus.c - Modbus RTU framing: known vectors, round trips, error handling. */
#include <stdio.h>
#include <string.h>
#include "../common/modbus_rtu.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; printf("  [ ok ] "); } else { g_fail++; printf("  [FAIL] "); } \
    printf(__VA_ARGS__); printf("\n"); } while (0)

static int g_written_start = -1, g_written_qty = 0, g_refresh = 0;
static void on_write(void *c, uint16_t s, uint16_t q) { (void)c; g_written_start = s; g_written_qty = q; }
static void on_read(void *c, uint8_t fn, uint16_t s, uint16_t q) { (void)c; (void)fn; (void)s; (void)q; g_refresh++; }

int main(void)
{
    printf("=== gt-hil Modbus RTU tests ===\n\n");

    printf("CRC and framing\n");
    const uint8_t v1[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};     /* textbook example: CRC = C5 CD */
    uint16_t c = mb_crc16(v1, 6);
    CHECK(c == 0xCDC5, "CRC of 01 03 00 00 00 0A is C5 CD (got %02X %02X)", c & 0xFF, c >> 8);
    uint8_t f[MB_MAX_FRAME];
    int n = mb_make_read(f, 1, MB_FN_READ_INPUT, 0, 12);
    CHECK(n == 8 && f[1] == 4 && f[5] == 12, "read request is 8 bytes");
    uint16_t vals[6] = {686, 5, 7, 1, 0, 0};
    n = mb_make_write_multiple(f, 1, 3, 6, vals);
    CHECK(n == 21 && f[6] == 12, "write-multiple of 6 registers is 21 bytes");

    printf("\nMaster against slave (in memory)\n");
    uint16_t ir[12], hr[9]; uint8_t rq[MB_MAX_FRAME], rs[MB_MAX_FRAME];
    for (int i = 0; i < 12; i++) ir[i] = (uint16_t)(1000 + i);
    memset(hr, 0, sizeof hr);
    MbSlave s = { .slave = 1, .ir = ir, .n_ir = 12, .hr = hr, .n_hr = 9, .before_read = on_read, .after_write = on_write };

    int rl = mb_make_read(rq, 1, MB_FN_READ_INPUT, 0, 12);
    int sl = mb_slave_handle(&s, rq, rl, rs);
    uint16_t got[12];
    CHECK(sl == 29 && mb_check_read(rs, sl, 1, MB_FN_READ_INPUT, 12, got) == MB_OK && got[0] == 1000 && got[11] == 1011,
          "read of 12 input registers returns the right values (29-byte response)");
    CHECK(g_refresh == 1, "slave refreshes its values before answering a read");

    n = mb_make_write_multiple(rq, 1, 3, 6, vals);
    sl = mb_slave_handle(&s, rq, n, rs);
    CHECK(sl == 8 && mb_check_write(rs, sl, 1, 3, 6) == MB_OK, "write-multiple acknowledged");
    CHECK(hr[3] == 686 && hr[4] == 5 && hr[8] == 0 && g_written_start == 3 && g_written_qty == 6, "written values land in the holding registers and the hook fires");

    rl = mb_make_read(rq, 1, MB_FN_READ_HOLDING, 3, 2);
    sl = mb_slave_handle(&s, rq, rl, rs);
    CHECK(mb_check_read(rs, sl, 1, MB_FN_READ_HOLDING, 2, got) == MB_OK && got[0] == 686 && got[1] == 5, "holding registers read back");

    printf("\nError handling\n");
    rl = mb_make_read(rq, 1, MB_FN_READ_INPUT, 10, 5);                 /* runs past register 11 */
    sl = mb_slave_handle(&s, rq, rl, rs);
    CHECK(sl == 5 && rs[1] == 0x84 && rs[2] == MB_EX_ILLEGAL_ADDRESS && mb_check_read(rs, sl, 1, 4, 5, got) == MB_ERR_EXCEPTION,
          "read past the end gives exception 02 (illegal address)");
    rl = mb_make_read(rq, 1, 0x06, 0, 1);
    sl = mb_slave_handle(&s, rq, rl, rs);
    CHECK(sl == 5 && rs[2] == MB_EX_ILLEGAL_FUNCTION, "unsupported function gives exception 01");
    rl = mb_make_read(rq, 1, MB_FN_READ_INPUT, 0, 12);
    rq[3] ^= 0x01;
    CHECK(mb_slave_handle(&s, rq, rl, rs) == 0, "frame with a bad CRC is ignored (no response)");
    rl = mb_make_read(rq, 2, MB_FN_READ_INPUT, 0, 12);
    CHECK(mb_slave_handle(&s, rq, rl, rs) == 0, "frame for another slave address is ignored");
    rl = mb_make_read(rq, 1, MB_FN_READ_INPUT, 0, 12);
    sl = mb_slave_handle(&s, rq, rl, rs);
    rs[5] ^= 0x40;
    CHECK(mb_check_read(rs, sl, 1, 4, 12, got) == MB_ERR_CRC, "master detects a corrupted response");
    CHECK(mb_check_read(rs, 3, 1, 4, 12, got) == MB_ERR_SHORT, "master detects a truncated response");

    printf("\n=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
