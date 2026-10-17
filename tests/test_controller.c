/*
 * test_controller.c - unit tests for the control philosophy.
 *
 * Each test states a rule from the Control Philosophy document and checks it
 * directly: latched trips, first-out, explicit reset, start permissives,
 * sensor voting, comms watchdog, warm-up / sync permit, droop, register map.
 * Synthetic-input tests drive the controller alone; the others use the
 * closed-loop simulation.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../plant/sim.h"
#include "../controller/standin_governor.h"
#include "../controller/sensor_vote.h"
#include "../include/hil_constants.h"
#include "../include/hil_regs.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; printf("  [ ok ] "); } \
    else      { g_fail++; printf("  [FAIL] "); } \
    printf(__VA_ARGS__); printf("\n"); } while (0)

#define DT HIL_CONTROL_PERIOD_S

static void healthy(CtrlInput *in, double rpm)
{
    static const double off[4] = {3, -2, 1, -2};
    memset(in, 0, sizeof *in);
    for (int i = 0; i < 3; i++) in->speed_rpm[i] = rpm;
    for (int i = 0; i < 4; i++) in->tc_c[i] = 100.0 + off[i];
    in->estop_ok = in->lube_ok = in->fuel_ok = in->starter_ok = 1;
    in->link_ok = 1;
}
static void cycles(StandinCtrl *c, const CtrlInput *in, int n)
{
    for (int i = 0; i < n; i++) ctrl_update(c, DT, in);
}

/* ---- start permissives ---- */
static void test_permissives(void)
{
    printf("Start permissives (each one blocks a start on its own)\n");
    const char *name[] = {"e-stop", "lube oil", "fuel supply", "starter", "comms link"};
    for (int k = 0; k < 5; k++) {
        StandinCtrl c; CtrlInput in; ctrl_init(&c);
        healthy(&in, 0.0); in.start_cmd = 1;
        switch (k) { case 0: in.estop_ok = 0; break; case 1: in.lube_ok = 0; break;
                     case 2: in.fuel_ok = 0; break; case 3: in.starter_ok = 0; break;
                     default: in.link_ok = 0; }
        cycles(&c, &in, 20);
        CHECK(c.state == CS_IDLE, "start blocked with %s lost", name[k]);
    }
    StandinCtrl c; CtrlInput in; ctrl_init(&c);
    healthy(&in, 0.0); in.start_cmd = 1;
    cycles(&c, &in, 2);
    CHECK(c.state == CS_CRANK, "start accepted with every permissive healthy");

    ctrl_init(&c); healthy(&in, 1000.0); in.start_cmd = 1; cycles(&c, &in, 5);
    CHECK(c.state == CS_IDLE, "start blocked while the rotor is turning (not at standstill)");

    ctrl_init(&c); healthy(&in, 0.0); in.start_cmd = 1; in.lube_ok = 0; cycles(&c, &in, 3);
    CHECK((c.alarm_word & HIL_AL_START_BLOCK) != 0, "a blocked start is announced (START_BLOCK alarm)");
    CHECK((c.perm_word & PM_LUBE) == 0 && (c.perm_word & PM_ESTOP) != 0,
          "permissive word shows exactly which condition is missing");
}

/* ---- latch / first-out / reset ---- */
static void test_latch(void)
{
    printf("Latched trip, first-out, explicit reset\n");
    StandinCtrl c; CtrlInput in; ctrl_init(&c);
    healthy(&in, 0.0); in.start_cmd = 1;
    cycles(&c, &in, 3);                                   /* CRANK */
    in.lube_ok = 0; cycles(&c, &in, 2);
    CHECK(c.state == CS_TRIP && c.first_out == HIL_FC_LOW_OIL, "lube-oil loss trips, first-out = low oil");
    CHECK(c.fsr_out == 0.0 && c.starter_on == 0 && c.trip_out == 1, "trip closes fuel, stops starter, opens breaker");

    in.estop_ok = 0; cycles(&c, &in, 5);
    CHECK(c.first_out == HIL_FC_LOW_OIL, "a later e-stop does not overwrite the first-out cause");

    in.lube_ok = 1; in.estop_ok = 1; in.start_cmd = 0;
    cycles(&c, &in, 400);                                 /* 20 s with everything healthy */
    CHECK(c.state == CS_TRIP, "trip does NOT clear by itself after the cause has gone (20 s)");

    in.reset_cmd = 1; in.estop_ok = 0; cycles(&c, &in, 3);
    CHECK(c.state == CS_TRIP, "reset refused while the e-stop is still pressed");
    in.reset_cmd = 0; in.estop_ok = 1; cycles(&c, &in, 2);
    in.reset_cmd = 1; cycles(&c, &in, 1);
    CHECK(c.state != CS_TRIP && c.first_out == HIL_FC_NONE, "reset accepted once the cause is gone; first-out cleared");

    /* level-held reset must not clear a NEW trip */
    cycles(&c, &in, 40);                                  /* back to IDLE */
    in.start_cmd = 1; cycles(&c, &in, 3);
    in.fuel_ok = 0; cycles(&c, &in, 3);
    CHECK(c.state == CS_TRIP && c.first_out == HIL_FC_FUEL_SUPPLY, "second trip: fuel supply");
    in.fuel_ok = 1; cycles(&c, &in, 20);                  /* reset_cmd still held high */
    CHECK(c.state == CS_TRIP, "a reset held high (no new edge) does not clear a later trip");
}

static void test_start_lockout(void)
{
    printf("Start request must be re-issued after a trip\n");
    StandinCtrl c; CtrlInput in; ctrl_init(&c);
    healthy(&in, 0.0); in.start_cmd = 1; cycles(&c, &in, 3);
    in.lube_ok = 0; cycles(&c, &in, 2);
    in.lube_ok = 1; in.reset_cmd = 1; cycles(&c, &in, 2);
    in.reset_cmd = 0; cycles(&c, &in, 60);                /* start still held */
    CHECK(c.state == CS_IDLE, "start held through the reset: unit stays idle (state %d)", c.state);
    in.start_cmd = 0; cycles(&c, &in, 2);
    in.start_cmd = 1; cycles(&c, &in, 3);
    CHECK(c.state == CS_CRANK, "start released and re-issued: unit starts");
}

/* ---- voting ---- */
static void test_voting(void)
{
    printf("Speed voting and diagnostics\n");
    SpeedVote v; speedvote_init(&v);
    double x[3]; int i;
    for (i = 0; i < 100; i++) {
        x[0] = 3500 + (i % 5) * 0.4; x[1] = 3500 - (i % 3) * 0.5; x[2] = 3500 + (i % 7) * 0.2;
        speedvote_update(&v, x, DT);
    }
    CHECK(v.n_good == 3 && v.bad[0] + v.bad[1] + v.bad[2] == 0, "three healthy channels: no flags");

    double worst = 0.0;
    for (i = 0; i < 400; i++) {                           /* channel 0 drifts +10 rpm/s */
        x[0] = 3500 + 10.0 * i * DT + (i % 5) * 0.4;
        x[1] = 3500 - (i % 3) * 0.5; x[2] = 3500 + (i % 7) * 0.2;
        speedvote_update(&v, x, DT);
        if (fabs(v.voted_rpm - 3500.0) > worst) worst = fabs(v.voted_rpm - 3500.0);
    }
    CHECK(v.bad[0] && !v.bad[1] && !v.bad[2], "drifting channel is flagged, the others are not");
    CHECK(worst < 5.0, "voted speed stays within %.1f RPM of the truth during the drift", worst);
    CHECK(!v.fault && v.n_good == 2, "one bad channel leaves a valid 2-channel vote");

    speedvote_init(&v);
    for (i = 0; i < 100; i++) {                           /* channel 1 stuck on a plausible value */
        x[0] = 3500 + (i % 5) * 0.4; x[1] = 3500.0; x[2] = 3500 + (i % 7) * 0.2;
        speedvote_update(&v, x, DT);
    }
    CHECK(v.bad[1] && !v.bad[0] && !v.bad[2], "flat-lined channel is flagged although its value is plausible");

    speedvote_init(&v);
    for (i = 0; i < 100; i++) {                           /* two channels disagree, one is good */
        x[0] = 3500 + (i % 5) * 0.4; x[1] = 3500 + 80.0 + (i % 3); x[2] = 3500.0 - 120.0 + (i % 7);
        speedvote_update(&v, x, DT);
    }
    CHECK(v.n_good < 3, "two bad channels leave fewer than three good ones");
    for (i = 0; i < 100; i++) {
        x[0] = 3500 + (i % 5) * 0.4; x[1] = 3580.0 + (i % 3); x[2] = 3380.0 + (i % 7);
        speedvote_update(&v, x, DT);
    }
    CHECK(v.fault, "cannot vote with two channels lost: fault raised");

    double t[4] = {503.0, 498.0, 501.0, 498.0};
    CHECK(fabs(tc_median4(t) - 499.5) < 1e-9, "median of four = mean of the middle two (%.1f)", tc_median4(t));
    CHECK(fabs(tc_spread4(t) - 5.0) < 1e-9, "spread = max - min (%.1f)", tc_spread4(t));
    t[3] = 120.0;
    CHECK(fabs(tc_median4(t) - 500.0) < 1.5, "one failed thermocouple moves the control value by only %.1f degC",
          fabs(tc_median4(t) - 499.5));
}

static void test_sensor_ack(void)
{
    printf("Latched sensor flags are cleared by a reset in IDLE\n");
    StandinCtrl c; CtrlInput in; ctrl_init(&c);
    healthy(&in, 0.0);
    for (int i = 0; i < 100; i++) { in.speed_rpm[0] = 200.0; cycles(&c, &in, 1); }   /* channel 1 reads 200 at standstill */
    CHECK(c.sv.bad[0], "a channel far from the others is flagged even at standstill");
    in.speed_rpm[0] = 0.0; cycles(&c, &in, 20);
    CHECK(c.sv.bad[0], "the flag stays latched after the channel recovers");
    in.reset_cmd = 1; cycles(&c, &in, 2);
    CHECK(!c.sv.bad[0] && c.state == CS_IDLE, "reset edge in IDLE clears it");
}

/* ---- comms watchdog ---- */
static void test_comms(void)
{
    printf("Controller-side communication watchdog\n");
    StandinCtrl c; CtrlInput in; ctrl_init(&c);
    healthy(&in, 0.0); in.start_cmd = 1; cycles(&c, &in, 3);        /* CRANK */
    in.link_ok = 0;
    cycles(&c, &in, 2);
    CHECK(!(c.alarm_word & HIL_AL_COMMS) && c.state == CS_CRANK, "2 failed polls: nothing");
    cycles(&c, &in, 1);
    CHECK((c.alarm_word & HIL_AL_COMMS) && c.state == CS_CRANK, "3 failed polls: alarm only");
    cycles(&c, &in, 6);
    CHECK(c.state == CS_CRANK, "9 failed polls: still no trip");
    cycles(&c, &in, 1);
    CHECK(c.state == CS_TRIP && c.first_out == HIL_FC_COMMS_LOSS, "10 failed polls: trip, first-out = comms loss");
    in.link_ok = 1; cycles(&c, &in, 20);
    CHECK(c.state == CS_TRIP, "link restored: trip stays latched");
}

/* ---- closed-loop helpers ---- */
static double to_run(Sim *s, double *t_warm)
{
    SimOut o; double t_acc_end = -1.0;
    sim_set_start(s, 1);
    for (int i = 0; i < 30000; i++) {
        sim_step(s); sim_get(s, &o);
        if (t_acc_end < 0 && (int)o.ctrl_state == CS_WARMUP) t_acc_end = o.time_s;
        if ((int)o.ctrl_state == CS_RUN) { if (t_warm) *t_warm = o.time_s - t_acc_end; return o.time_s; }
    }
    return -1.0;
}
static void run_s(Sim *s, double sec) { for (int i = 0; i < (int)(sec / HIL_STEP_S); i++) sim_step(s); }

static void test_sync(void)
{
    printf("Warm-up hold and sync permit\n");
    Sim *s = sim_create(); SimOut o;
    sim_reset(s, 5, 1);
    sim_set_start(s, 1);
    int permit_early = 0, breaker_early = 0;
    double t_warm = 0.0, t_run = -1.0, t_wu0 = -1.0;
    for (int i = 0; i < 30000 && t_run < 0; i++) {
        sim_step(s); sim_get(s, &o);
        sim_set_breaker(s, 1);                               /* operator asks for the breaker all the time */
        if ((int)o.ctrl_state == CS_WARMUP && t_wu0 < 0) t_wu0 = o.time_s;
        if ((int)o.ctrl_state != CS_RUN) { if (o.sync_permit > 0.5) permit_early = 1; if (o.breaker > 0.5) breaker_early = 1; }
        if ((int)o.ctrl_state == CS_RUN) { t_run = o.time_s; t_warm = t_run - t_wu0; }
    }
    CHECK(t_run > 0, "unit reaches RUN at t = %.1f s", t_run);
    CHECK(t_warm >= HIL_WARMUP_S - 0.1, "warm-up hold lasted %.1f s (>= %.1f s)", t_warm, HIL_WARMUP_S);
    CHECK(!permit_early, "no sync permit before RUN");
    CHECK(!breaker_early, "breaker request ignored until the permit exists");
    run_s(s, 1.0); sim_get(s, &o);
    CHECK(o.sync_permit > 0.5 && o.breaker > 0.5, "in RUN the permit is set and the requested breaker closes");
    sim_set_breaker(s, 0); run_s(s, 0.5); sim_get(s, &o);
    CHECK(o.breaker < 0.5, "breaker opens when the request is withdrawn");
    sim_destroy(s);
}

/* ---- temperature limiter authority ---- */
static void test_temp_limiter(void)
{
    printf("Temperature limiter authority\n");
    Sim *s = sim_create(); SimOut o;
    sim_reset(s, 9, 1);
    to_run(s, NULL);
    sim_set_breaker(s, 1); sim_set_load(s, 0.6 * sim_rated_power_mw(s));
    run_s(s, 40.0);
    sim_get(s, &o);
    CHECK((int)o.selected != SEL_TEMP, "at 60 %% load the limiter is not in control (exhaust %.0f degC)", o.exh_voted_c);
    sim_ctrl(s)->t_ref_c = o.exh_voted_c - 30.0;             /* limit now 30 degC below the running value */
    int seen = 0; double min_fsr_temp = 9.0;
    for (int i = 0; i < 1500; i++) {
        sim_step(s); sim_get(s, &o);
        if ((int)o.selected == SEL_TEMP) seen = 1;
        if (o.fsr_temp < min_fsr_temp) min_fsr_temp = o.fsr_temp;
    }
    CHECK(seen, "with the limit below the running temperature the limiter takes over the fuel");
    sim_destroy(s);
}

/* ---- droop ---- */
static double settled_speed(int droop, double pct)
{
    Sim *s = sim_create(); SimOut o;
    sim_reset(s, 17, 0);                                      /* noise off: clean steady state */
    to_run(s, NULL);
    sim_set_droop(s, droop);
    sim_set_breaker(s, 1);
    sim_set_load(s, pct * sim_rated_power_mw(s) / 100.0);
    run_s(s, 80.0);
    sim_get(s, &o);
    sim_destroy(s);
    return o.speed_rpm;
}
static void test_droop(void)
{
    printf("Isochronous and droop governor\n");
    double i50 = settled_speed(0, 40), i100 = settled_speed(0, 100);
    double d50 = settled_speed(1, 40), d100 = settled_speed(1, 100);
    CHECK(fabs(i100 - i50) < 5.0, "isochronous: speed is the same at 40 %% and 100 %% load (%.1f vs %.1f RPM)", i50, i100);
    double drop = d50 - d100;
    double expect = HIL_DROOP_PU * HIL_RATED_SPEED_RPM * 0.35;   /* ~35 % of the fuel span between the two loads */
    CHECK(drop > 25.0 && drop < 110.0, "droop: speed falls %.0f RPM from 40 %% to 100 %% load (about %.0f expected)", drop, expect);
    CHECK(d100 > HIL_RATED_SPEED_RPM * 0.95, "droop at full load stays above 95 %% speed (%.0f RPM)", d100);
}

/* ---- plant-side watchdog ---- */
static void test_plant_watchdog(void)
{
    printf("Plant-side watchdog\n");
    Sim *s = sim_create(); SimOut o;
    sim_reset(s, 23, 1);
    to_run(s, NULL);
    sim_set_breaker(s, 1); sim_set_load(s, 0.5 * sim_rated_power_mw(s)); run_s(s, 20.0);
    sim_set_link(s, 0); run_s(s, 0.35); sim_set_link(s, 1); run_s(s, 1.0); sim_get(s, &o);
    CHECK(o.plant_wd < 0.5 && o.tripped < 0.5, "0.35 s outage: plant watchdog does not act");
    sim_set_link(s, 0); run_s(s, 1.2); sim_get(s, &o);
    CHECK(o.plant_wd > 0.5 && o.valve_pos < 0.05 && o.breaker < 0.5, "1.2 s outage: plant closes the fuel valve and opens the breaker by itself");
    sim_destroy(s);
}

/* ---- register map ---- */
static void test_regmap(void)
{
    printf("Register map v%d.%d\n", HIL_REGMAP_VERSION_MAJOR, HIL_REGMAP_VERSION_MINOR);
    CHECK(HIL_IR_COUNT == 12 && HIL_HR_COUNT == 9, "12 input registers, 9 holding registers");
    CHECK(hil_enc(3500.0, HIL_SCALE_SPEED) == 3500, "speed 3500 RPM encodes to 3500");
    CHECK(hil_enc(612.3, HIL_SCALE_TEMP) == 6123 && fabs(hil_dec(6123, HIL_SCALE_TEMP) - 612.3) < 1e-9,
          "temperature 612.3 degC round-trips through the 0.1 degC scale");
    CHECK(hil_enc(-5.0, HIL_SCALE_TEMP) == 0 && hil_enc(1e9, HIL_SCALE_TEMP) == 65535, "encoding clamps to 0..65535");
    CHECK(hil_enc(0.686, HIL_SCALE_FSR) == 686, "fuel demand 0.686 encodes to 686 (0.1 %% steps)");
    unsigned all = HIL_DI_BREAKER_CLOSED | HIL_DI_ESTOP_HEALTHY | HIL_DI_FLAME | HIL_DI_LUBE_OK |
                   HIL_DI_FUEL_OK | HIL_DI_STARTER_OK | HIL_DI_PLANT_WD;
    CHECK(all == 0x7Fu, "seven individual discrete bits, no overlap");
    unsigned al = HIL_AL_OVERSPEED | HIL_AL_OVERTEMP | HIL_AL_SPREAD | HIL_AL_SPEED1_BAD |
                  HIL_AL_SPEED2_BAD | HIL_AL_SPEED3_BAD | HIL_AL_COMMS | HIL_AL_START_BLOCK;
    CHECK(al == 0xFFu, "eight alarm bits, no overlap");
    CHECK(HIL_FC_COUNT == 12, "fault codes 0..11");
}

int main(void)
{
    printf("=== gt-hil controller unit tests ===\n\n");
    test_permissives();    printf("\n");
    test_latch();          printf("\n");
    test_start_lockout();  printf("\n");
    test_voting();         printf("\n");
    test_sensor_ack();     printf("\n");
    test_comms();          printf("\n");
    test_sync();           printf("\n");
    test_temp_limiter();   printf("\n");
    test_droop();          printf("\n");
    test_plant_watchdog(); printf("\n");
    test_regmap();         printf("\n");
    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
