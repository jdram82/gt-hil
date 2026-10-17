/*
 * plant_node.c - the BeagleBone Black side of the HIL rig.
 *
 *   - runs the plant model in real time, one step every 10 ms;
 *   - is the Modbus RTU SLAVE on a serial port (UART1 = /dev/ttyS1 on the BBB);
 *   - has a local TCP command port (127.0.0.1) used by tools/hil_runner.py to
 *     play the role of the operator and the test supervisor: start, reset,
 *     load, breaker request, sensor faults, link drop, and to read back the
 *     state for logging;
 *   - measures its own timing (wake-up lateness of the 10 ms loop, poll
 *     period, response latency) so the report can quote real numbers.
 *
 * The controller is NOT in this program: it is on the STM32 (or, for software
 * tests, in ctrl_node on another serial port).
 *
 * Build: make build/plant_node        Run: see docs/hil_bringup.md
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

#include "../plant/sim.h"
#include "../common/modbus_rtu.h"
#include "../include/hil_regs.h"
#include "../include/hil_constants.h"

#define MAX_SAMPLES 400000

typedef struct { double *v; int n; } Samples;

static Sim *g_sim;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile sig_atomic_t g_quit = 0;
static int g_link_up = 1;
static int g_drop_steps = 0;             /* timed link drop, in plant steps */

static uint16_t g_ir[HIL_IR_COUNT], g_hr[HIL_HR_COUNT];
static uint16_t g_plant_hb = 0;

static Samples s_wake_us, s_poll_ms, s_lat_us, s_exec_us;
static long n_req = 0, n_bad = 0, n_dropped = 0, n_exc = 0, n_ir = 0;

/* ------------------------------------------------------------------ time */
static double now_s(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static void add_sample(Samples *s, double x)
{
    if (!s->v) s->v = (double *)malloc(sizeof(double) * MAX_SAMPLES);
    if (s->v && s->n < MAX_SAMPLES) s->v[s->n++] = x;
}
static int cmp_d(const void *a, const void *b) { double x = *(const double *)a, y = *(const double *)b; return (x > y) - (x < y); }
static void stats_json(const Samples *s, const char *name, char *out, size_t cap, int first)
{
    if (s->n < 2) { snprintf(out + strlen(out), cap - strlen(out), "%s\"%s\":null", first ? "" : ",", name); return; }
    double *c = (double *)malloc(sizeof(double) * s->n);
    memcpy(c, s->v, sizeof(double) * s->n);
    qsort(c, s->n, sizeof(double), cmp_d);
    double sum = 0, sq = 0;
    for (int i = 0; i < s->n; i++) { sum += c[i]; sq += c[i] * c[i]; }
    double mean = sum / s->n, sd = sqrt(fmax(0.0, sq / s->n - mean * mean));
    snprintf(out + strlen(out), cap - strlen(out),
             "%s\"%s\":{\"n\":%d,\"mean\":%.4f,\"std\":%.4f,\"p50\":%.4f,\"p99\":%.4f,\"p999\":%.4f,\"max\":%.4f}",
             first ? "" : ",", name, s->n, mean, sd, c[s->n / 2], c[(int)(0.99 * (s->n - 1))],
             c[(int)(0.999 * (s->n - 1))], c[s->n - 1]);
    free(c);
}
static void timing_json(char *out, size_t cap)
{
    out[0] = 0;
    snprintf(out, cap, "{");
    stats_json(&s_wake_us, "plant_wake_lateness_us", out, cap, 1);
    stats_json(&s_exec_us, "plant_step_exec_us", out, cap, 0);
    stats_json(&s_poll_ms, "poll_period_ms", out, cap, 0);
    stats_json(&s_lat_us,  "response_latency_us", out, cap, 0);
    snprintf(out + strlen(out), cap - strlen(out),
             ",\"modbus\":{\"requests\":%ld,\"input_reads\":%ld,\"bad_frames\":%ld,\"exceptions\":%ld,\"dropped_link_down\":%ld}}",
             n_req, n_ir, n_bad, n_exc, n_dropped);
}

/* ------------------------------------------------------------ registers */
static void encode_inputs(const PlantMeasurements *m, int wd)
{
    for (int i = 0; i < 3; i++) g_ir[HIL_IR_SPEED1_RPM + i] = hil_enc(m->speed_rpm[i], HIL_SCALE_SPEED);
    for (int i = 0; i < 4; i++) g_ir[HIL_IR_TC1 + i]        = hil_enc(m->tc_c[i], HIL_SCALE_TEMP);
    g_ir[HIL_IR_PRESSURE]  = hil_enc(m->press_bar_g, HIL_SCALE_PRESS);
    g_ir[HIL_IR_POWER]     = hil_enc(m->power_mw, HIL_SCALE_POWER);
    g_ir[HIL_IR_FUEL_FLOW] = hil_enc(m->fuel_kg_s, HIL_SCALE_FUEL);
    uint16_t d = 0;
    if (m->breaker_closed) d |= HIL_DI_BREAKER_CLOSED;
    if (m->estop_ok)       d |= HIL_DI_ESTOP_HEALTHY;
    if (m->flame)          d |= HIL_DI_FLAME;
    if (m->lube_ok)        d |= HIL_DI_LUBE_OK;
    if (m->fuel_ok)        d |= HIL_DI_FUEL_OK;
    if (m->starter_ok)     d |= HIL_DI_STARTER_OK;
    if (wd)                d |= HIL_DI_PLANT_WD;
    g_ir[HIL_IR_DISCRETE] = d;
}

static void before_read(void *ctx, uint8_t fn, uint16_t start, uint16_t qty)
{
    (void)ctx; (void)start; (void)qty;
    if (fn != MB_FN_READ_INPUT) return;
    PlantMeasurements m;
    pthread_mutex_lock(&g_lock);
    sim_ext_sample(g_sim, &m);
    encode_inputs(&m, sim_ext_watchdog_active(g_sim));
    g_plant_hb++;
    g_ir[HIL_IR_PLANT_HB] = g_plant_hb;
    pthread_mutex_unlock(&g_lock);
}

static void after_write(void *ctx, uint16_t start, uint16_t qty)
{
    (void)ctx;
    if (start + qty <= HIL_HR_FUEL_CMD) return;           /* operator block only: ignore */
    pthread_mutex_lock(&g_lock);
    sim_ext_write(g_sim, hil_dec(g_hr[HIL_HR_FUEL_CMD], HIL_SCALE_FSR), g_hr[HIL_HR_DO_WORD],
                  g_hr[HIL_HR_CTRL_STATE], g_hr[HIL_HR_FIRST_OUT], g_hr[HIL_HR_ALARM_WORD]);
    pthread_mutex_unlock(&g_lock);
}

/* --------------------------------------------------------------- serial */
static int open_serial(const char *path, int baud)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(path); return -1; }
    struct termios t;
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        speed_t sp = baud == 9600 ? B9600 : baud == 19200 ? B19200 : baud == 38400 ? B38400 :
                     baud == 57600 ? B57600 : B115200;
        cfsetispeed(&t, sp); cfsetospeed(&t, sp);
        t.c_cflag &= ~(CSIZE | PARODD | CSTOPB | CRTSCTS);
        t.c_cflag |= CS8 | PARENB | CLOCAL | CREAD;          /* 8E1 */
        t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
        tcsetattr(fd, TCSANOW, &t);
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* expected request length once enough bytes have arrived; 0 = not known yet */
static int expected_len(const uint8_t *b, int n)
{
    if (n < 2) return 0;
    if (b[1] == MB_FN_READ_HOLDING || b[1] == MB_FN_READ_INPUT) return 8;
    if (b[1] == MB_FN_WRITE_MULTIPLE) return n >= 7 ? 9 + b[6] : 0;
    return 8;
}

static void *slave_thread(void *arg)
{
    int fd = *(int *)arg;
    MbSlave slave = { .slave = 1, .ir = g_ir, .n_ir = HIL_IR_COUNT, .hr = g_hr, .n_hr = HIL_HR_COUNT,
                      .before_read = before_read, .after_write = after_write };
    uint8_t rq[MB_MAX_FRAME], rs[MB_MAX_FRAME];
    double last_poll = 0;
    while (!g_quit) {
        int n = 0;
        struct pollfd p = { fd, POLLIN, 0 };
        if (poll(&p, 1, 100) <= 0) continue;
        double t_first = now_s();
        while (n < MB_MAX_FRAME) {
            int r = read(fd, rq + n, MB_MAX_FRAME - n);
            if (r > 0) n += r;
            int want = expected_len(rq, n);
            if (want && n >= want) break;
            if (poll(&p, 1, 4) <= 0) break;                  /* 4 ms of silence ends a frame */
        }
        if (n == 0) continue;
        double t_done = now_s();
        n_req++;
        if (!g_link_up) { n_dropped++; continue; }           /* supervisor cut the link: stay silent */
        int rl = mb_slave_handle(&slave, rq, n, rs);
        if (rl == 0) { n_bad++; continue; }
        if (rs[1] & 0x80) n_exc++;
        (void)!write(fd, rs, rl);
        double t_resp = now_s();
        add_sample(&s_lat_us, (t_resp - t_done) * 1e6);
        if (rq[1] == MB_FN_READ_INPUT) {
            n_ir++;
            if (last_poll > 0) add_sample(&s_poll_ms, (t_first - last_poll) * 1e3);
            last_poll = t_first;
        }
    }
    return NULL;
}

/* -------------------------------------------------------- command port */
static void out_line(FILE *f, const char *s) { fputs(s, f); fputc('\n', f); fflush(f); }

static void handle_command(char *line, FILE *out)
{
    char a[32] = "", b[32] = "";
    double x = 0, y = 0, z = 0;
    int i;
    if (sscanf(line, "%31s", a) != 1) return;

    pthread_mutex_lock(&g_lock);
    if (!strcmp(a, "start"))        { sscanf(line, "%*s %d", &i); g_hr[HIL_HR_START_CMD] = (uint16_t)i; out_line(out, "ok"); }
    else if (!strcmp(a, "reset"))   { sscanf(line, "%*s %d", &i); g_hr[HIL_HR_RESET_CMD] = (uint16_t)i; out_line(out, "ok"); }
    else if (!strcmp(a, "droop"))   { sscanf(line, "%*s %d", &i); g_hr[HIL_HR_MODE] = (uint16_t)i; out_line(out, "ok"); }
    else if (!strcmp(a, "load"))    { sscanf(line, "%*s %lf", &x); sim_set_load(g_sim, x); out_line(out, "ok"); }
    else if (!strcmp(a, "breaker")) { sscanf(line, "%*s %d", &i); sim_set_breaker(g_sim, i); out_line(out, "ok"); }
    else if (!strcmp(a, "link"))    { sscanf(line, "%*s %d", &i); g_link_up = i; sim_set_link(g_sim, i); out_line(out, "ok"); }
    else if (!strcmp(a, "linkdrop")) {                    /* link down for exactly N ms of plant time */
        sscanf(line, "%*s %lf", &x);
        g_drop_steps = (int)(x / (HIL_STEP_S * 1e3) + 0.5); g_link_up = 0; sim_set_link(g_sim, 0); out_line(out, "ok");
    }
    else if (!strcmp(a, "reset_sim")) {                   /* fresh plant, rotor at rest */
        unsigned long long seed = 12345ULL; int noise = 1;
        sscanf(line, "%*s %llu %d", &seed, &noise);
        sim_reset(g_sim, seed, noise); sim_ext_enable(g_sim);
        memset(g_hr, 0, sizeof g_hr); g_plant_hb = 0; g_link_up = 1; g_drop_steps = 0;
        out_line(out, "ok");
    }
    else if (!strcmp(a, "perm")) {
        int w = -1;
        if (sscanf(line, "%*s %31s %d", b, &i) == 2) {
            if (!strcmp(b, "estop")) w = 0; else if (!strcmp(b, "lube")) w = 1;
            else if (!strcmp(b, "fuel")) w = 2; else if (!strcmp(b, "starter")) w = 3;
        }
        if (w >= 0) { sim_set_permissive(g_sim, w, i); out_line(out, "ok"); } else out_line(out, "err perm");
    }
    else if (!strcmp(a, "sfault") || !strcmp(a, "tfault")) {
        int ch, mode;
        if (sscanf(line, "%*s %d %d %lf %lf", &ch, &mode, &x, &y) >= 3) {
            if (a[0] == 's') sim_inject_speed_fault(g_sim, ch, mode, x, y);
            else             sim_inject_tc_fault(g_sim, ch, mode, x, y);
            out_line(out, "ok");
        } else out_line(out, "err fault");
    }
    else if (!strcmp(a, "clear"))   { sim_clear_faults(g_sim); out_line(out, "ok"); }
    else if (!strcmp(a, "ambient")) { sscanf(line, "%*s %lf", &x); sim_set_ambient_k(g_sim, x); out_line(out, "ok"); }
    else if (!strcmp(a, "info")) {
        char buf[128]; snprintf(buf, sizeof buf, "info %d %.6f", sim_out_doubles(), sim_rated_power_mw(g_sim)); out_line(out, buf);
    }
    else if (!strcmp(a, "get")) {
        SimOut o; sim_get(g_sim, &o);
        const double *d = (const double *)&o; int n = sim_out_doubles();
        fputs("D", out);
        for (i = 0; i < n; i++) fprintf(out, " %.9g", d[i]);
        fputc('\n', out); fflush(out);
    }
    else if (!strcmp(a, "timing")) {
        static char buf[8192]; timing_json(buf, sizeof buf); out_line(out, buf);
    }
    else if (!strcmp(a, "clear_timing")) {
        s_wake_us.n = s_poll_ms.n = s_lat_us.n = s_exec_us.n = 0; n_req = n_bad = n_dropped = n_exc = n_ir = 0; out_line(out, "ok");
    }
    else out_line(out, "err unknown");
    (void)z;
    pthread_mutex_unlock(&g_lock);
}

static void *cmd_thread(void *arg)
{
    int port = *(int *)arg;
    int ls = socket(AF_INET, SOCK_STREAM, 0), one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons((uint16_t)port) };
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(ls, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(ls, 1) < 0) { perror("command port"); g_quit = 1; return NULL; }
    while (!g_quit) {
        int c = accept(ls, NULL, NULL);
        if (c < 0) continue;
        setsockopt(c, IPPROTO_TCP, 1 /* TCP_NODELAY */, &one, sizeof one);
        FILE *f = fdopen(c, "r+");
        char line[256];
        while (!g_quit && fgets(line, sizeof line, f)) {
            if (!strncmp(line, "quit", 4)) break;
            handle_command(line, f);
        }
        fclose(f);
    }
    return NULL;
}

static void on_sig(int s) { (void)s; g_quit = 1; }

int main(int argc, char **argv)
{
    const char *port = "/dev/ttyS1";
    int baud = 115200, cmd_port = 5001, rt = 0, noise = 1;
    unsigned long long seed = 12345ULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
        else if (!strcmp(argv[i], "--baud") && i + 1 < argc) baud = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cmd-port") && i + 1 < argc) cmd_port = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed") && i + 1 < argc) seed = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(argv[i], "--no-noise")) noise = 0;
        else if (!strcmp(argv[i], "--rt")) rt = 1;
        else { fprintf(stderr, "usage: %s [--port dev] [--baud n] [--cmd-port n] [--seed n] [--no-noise] [--rt]\n", argv[0]); return 2; }
    }
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig); signal(SIGPIPE, SIG_IGN);

    g_sim = sim_create();
    sim_reset(g_sim, seed, noise);
    sim_ext_enable(g_sim);
    g_hr[HIL_HR_START_CMD] = g_hr[HIL_HR_RESET_CMD] = g_hr[HIL_HR_MODE] = 0;

    int fd = open_serial(port, baud);
    if (fd < 0) return 1;

    if (rt) {
        struct sched_param sp = { .sched_priority = 50 };
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("SCHED_FIFO (continuing without)");
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) perror("mlockall (continuing)");
    }

    pthread_t ts, tc;
    pthread_create(&ts, NULL, slave_thread, &fd);
    pthread_create(&tc, NULL, cmd_thread, &cmd_port);
    fprintf(stderr, "plant_node: Modbus slave on %s (%d 8E1), command port 127.0.0.1:%d, step %.0f ms%s\n",
            port, baud, cmd_port, HIL_STEP_S * 1e3, rt ? ", real-time priority requested" : "");

    /* ---- the 10 ms plant loop ---- */
    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    const long step_ns = (long)(HIL_STEP_S * 1e9);
    while (!g_quit) {
        next.tv_nsec += step_ns;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        struct timespec w; clock_gettime(CLOCK_MONOTONIC, &w);
        double late_us = (w.tv_sec - next.tv_sec) * 1e6 + (w.tv_nsec - next.tv_nsec) * 1e-3;
        double t0 = now_s();
        pthread_mutex_lock(&g_lock);
        sim_step(g_sim);
        if (g_drop_steps > 0 && --g_drop_steps == 0) { g_link_up = 1; sim_set_link(g_sim, 1); }
        pthread_mutex_unlock(&g_lock);
        add_sample(&s_wake_us, late_us);
        add_sample(&s_exec_us, (now_s() - t0) * 1e6);
    }
    static char buf[8192]; timing_json(buf, sizeof buf);
    printf("%s\n", buf);
    return 0;
}
