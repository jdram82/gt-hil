/*
 * ctrl_node.c - the controller application running on Linux.
 *
 * This is the same hil_master.c that runs on the STM32, with a Linux serial
 * port underneath.  Its jobs: (1) test the whole link in software, over a
 * virtual serial pair, before any wiring; (2) act as a stand-in controller on a
 * second serial port of the BeagleBone if the STM32 is ever unavailable.
 *
 *   ctrl_node --port /dev/pts/N [--baud 115200] [--rt]
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "../common/hil_master.h"
#include "../include/hil_constants.h"

static volatile sig_atomic_t g_quit = 0;
static void on_sig(int s) { (void)s; g_quit = 1; }

typedef struct { int fd; } LinuxPort;

static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

static int lp_write(void *c, const uint8_t *b, int n)
{
    LinuxPort *p = (LinuxPort *)c; int done = 0;
    while (done < n) {
        int r = write(p->fd, b + done, n - done);
        if (r < 0) { if (errno == EAGAIN) { usleep(200); continue; } return done; }
        done += r;
    }
    return done;
}
static int lp_read(void *c, uint8_t *b, int max, int expect, uint32_t timeout_ms)
{
    LinuxPort *p = (LinuxPort *)c; int n = 0;
    double t_end = now_s() + timeout_ms * 1e-3;
    struct pollfd pf = { p->fd, POLLIN, 0 };
    for (;;) {
        double left = t_end - now_s();
        int wait_ms = (n == 0) ? (int)ceil(left * 1e3) : 4;      /* once started, 4 ms of silence = end */
        if (wait_ms < 0) wait_ms = 0;
        if (poll(&pf, 1, wait_ms) <= 0) break;
        int r = read(p->fd, b + n, max - n);
        if (r > 0) n += r;
        if (expect && n >= expect) break;
        if (n >= max || now_s() > t_end) break;
    }
    return n;
}
static void lp_flush(void *c) { tcflush(((LinuxPort *)c)->fd, TCIFLUSH); }
static uint32_t lp_now(void *c) { (void)c; return (uint32_t)(now_s() * 1e6); }

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
        t.c_cflag |= CS8 | PARENB | CLOCAL | CREAD;
        t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
        tcsetattr(fd, TCSANOW, &t);
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

int main(int argc, char **argv)
{
    const char *port = NULL; int baud = 115200, rt = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
        else if (!strcmp(argv[i], "--baud") && i + 1 < argc) baud = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--rt")) rt = 1;
        else port = NULL;
    }
    if (!port) { fprintf(stderr, "usage: %s --port /dev/xxx [--baud n] [--rt]\n", argv[0]); return 2; }
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);

    LinuxPort lp = { open_serial(port, baud) };
    if (lp.fd < 0) return 1;
    if (rt) {
        struct sched_param sp = { .sched_priority = 50 };
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) perror("SCHED_FIFO (continuing without)");
        mlockall(MCL_CURRENT | MCL_FUTURE);
    }

    HilPort hp = { &lp, lp_write, lp_read, lp_flush, lp_now };
    static HilMaster m;
    hil_master_init(&m, hp, 1);

    fprintf(stderr, "ctrl_node: master on %s (%d 8E1), cycle %.0f ms\n", port, baud, HIL_CONTROL_PERIOD_S * 1e3);
    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    const long ns = (long)(HIL_CONTROL_PERIOD_S * 1e9);
    double sum_late = 0, max_late = 0; long nl = 0;
    double t_report = now_s() + 10.0;
    while (!g_quit) {
        next.tv_nsec += ns;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        struct timespec w; clock_gettime(CLOCK_MONOTONIC, &w);
        double late = (w.tv_sec - next.tv_sec) * 1e6 + (w.tv_nsec - next.tv_nsec) * 1e-3;
        sum_late += late; if (late > max_late) max_late = late; nl++;
        hil_master_cycle(&m);
        if (now_s() >= t_report) {
            t_report += 10.0;
            fprintf(stderr, "ctrl_node: cycles %u failed %u timeouts %u crc %u | cycle %u us (max %u) | state %d first-out %d\n",
                    m.cycles, m.failed_cycles, m.timeouts, m.crc_errors, m.last_cycle_us, m.max_cycle_us,
                    (int)m.ctrl.state, m.ctrl.first_out);
        }
    }
    printf("{\"cycles\":%u,\"failed_cycles\":%u,\"timeouts\":%u,\"crc_errors\":%u,\"write_failures\":%u,"
           "\"cycle_exec_us_max\":%u,\"ctrl_update_us_max\":%u,\"wake_lateness_us_mean\":%.1f,\"wake_lateness_us_max\":%.1f}\n",
           m.cycles, m.failed_cycles, m.timeouts, m.crc_errors, m.write_failures, m.max_cycle_us, m.max_ctrl_us,
           nl ? sum_late / nl : 0.0, max_late);
    return 0;
}
