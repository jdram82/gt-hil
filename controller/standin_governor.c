/*
 * standin_governor.c - gas-turbine controller, week 2 (full philosophy).
 */
#include "standin_governor.h"
#include "../include/hil_constants.h"
#include <math.h>
#include <string.h>

static double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
static int on(const StandinCtrl *c, unsigned f) { return (c->features & f) != 0; }

void ctrl_init(StandinCtrl *c)
{
    memset(c, 0, sizeof *c);

    c->n_ref_pu        = 1.0;
    c->kp_gov          = 6.0;
    c->ki_gov          = 2.5;
    c->accel_ref_pu_s  = 0.020;
    c->kp_acc          = 5.0;
    c->ki_acc          = 15.0;
    c->t_ref_c         = HIL_EXH_TEMP_CONTROL_C;
    c->kp_t            = 0.0030;
    c->ki_t            = 0.0020;
    c->temp_headroom   = 0.10;
    c->fsr_light       = 0.050;
    c->fsr_start_rate  = 0.020;
    c->fsr_floor_start = 0.040;
    c->fsr_floor_run   = 0.100;
    c->fsr_noload      = 0.140;
    c->purge_s         = 5.0;
    c->light_timeout_s = 8.0;
    c->track_tau_s     = 0.5;
    c->accel_filter_s  = 0.5;
    c->warmup_s        = HIL_WARMUP_S;
    c->droop_pu        = HIL_DROOP_PU;
    c->features        = FEAT_ALL;

    speedvote_init(&c->sv);
    prot_init(&c->prot);
    c->state = CS_IDLE;
    c->trip_out = 0;
}

unsigned ctrl_permissives(const StandinCtrl *c, const CtrlInput *in)
{
    unsigned p = 0;
    if (in->estop_ok)             p |= PM_ESTOP;
    if (in->lube_ok)              p |= PM_LUBE;
    if (in->fuel_ok)              p |= PM_FUEL;
    if (in->starter_ok)           p |= PM_STARTER;
    if (!c->sv.fault)             p |= PM_SENSORS;
    if (in->link_ok)              p |= PM_COMMS;
    if (c->speed_voted_rpm / HIL_RATED_SPEED_RPM < HIL_N_STANDSTILL_PU) p |= PM_STANDSTILL;
    if (c->state != CS_TRIP && !c->start_lockout) p |= PM_NO_TRIP;
    return p;
}

static void enter(StandinCtrl *c, CtrlState s) { c->state = s; c->t_state = 0.0; }

static void trip(StandinCtrl *c, int cause)
{
    if (c->state == CS_TRIP) return;          /* first-out: keep the first */
    c->first_out = cause;
    c->start_lockout = 1;
    enter(c, CS_TRIP);
}

/* priority for first-out when several conditions arrive in the same cycle */
static const int PRIORITY[] = {
    HIL_FC_ESTOP, HIL_FC_OVERSPEED, HIL_FC_OVERTEMP, HIL_FC_SENSOR_FAULT,
    HIL_FC_EXH_SPREAD, HIL_FC_LOW_OIL, HIL_FC_FUEL_SUPPLY
};
#define NPRIO ((int)(sizeof PRIORITY / sizeof PRIORITY[0]))

static void trip_outputs(StandinCtrl *c)
{
    c->fsr_out = 0.0; c->starter_on = 0; c->ignition_on = 0;
    c->sync_permit = 0; c->trip_out = 1;
}

void ctrl_update(StandinCtrl *c, double dt, const CtrlInput *in)
{
    const double RATED = HIL_RATED_SPEED_RPM;
    int active, i;

    c->t_state += dt;

    /* ================= communication watchdog (controller side) ========= */
    if (in->link_ok) c->missed_polls = 0; else c->missed_polls++;

    if (!in->link_ok) {
        /* No fresh data: hold the last outputs, do not run the control law. */
        c->alarm_word = c->alarm_base | (c->missed_polls >= HIL_COMMS_ALARM_POLLS ? HIL_AL_COMMS : 0);
        if (on(c, FEAT_COMMS_WD) && c->missed_polls >= HIL_COMMS_TRIP_POLLS &&
            c->state != CS_IDLE && c->state != CS_TRIP) {
            trip(c, HIL_FC_COMMS_LOSS);
        }
        if (c->state == CS_TRIP) trip_outputs(c);
        c->reset_prev = in->reset_cmd;
        return;
    }

    /* ================= sensors ========================================== */
    c->sv.use_vote = on(c, FEAT_VOTING);
    c->sv.use_diag = on(c, FEAT_SENSOR_DIAG) && on(c, FEAT_VOTING);
    speedvote_update(&c->sv, in->speed_rpm, dt);
    c->speed_voted_rpm = c->sv.voted_rpm;
    c->t_voted_c  = tc_median4(in->tc_c);
    c->t_spread_c = tc_spread4(in->tc_c);

    const double n = c->speed_voted_rpm / RATED;
    const double t_exh = c->t_voted_c;

    double accel_raw = (n - c->n_prev) / dt;
    c->n_prev = n;
    c->accel_f += (accel_raw - c->accel_f) * dt / c->accel_filter_s;

    /* ================= protection (upstream of fuel arbitration) ======== */
    ProtInput pi;
    pi.speed_rpm = c->speed_voted_rpm; pi.t_exh_c = t_exh; pi.spread_c = c->t_spread_c;
    pi.speed_fault = c->sv.fault;
    pi.estop_ok = in->estop_ok; pi.lube_ok = in->lube_ok; pi.fuel_ok = in->fuel_ok;
    unsigned al = 0;
    c->cond_mask = prot_conditions(&c->prot, &pi, c->features, dt, &al);
    for (i = 0; i < 3; i++) if (c->sv.bad[i]) al |= (HIL_AL_SPEED1_BAD << i);
    if (c->sv.disagree) al |= HIL_AL_SPEED1_BAD | HIL_AL_SPEED2_BAD | HIL_AL_SPEED3_BAD;

    active = (c->state == CS_CRANK || c->state == CS_IGNITE || c->state == CS_ACCEL ||
              c->state == CS_WARMUP || c->state == CS_RUN);
    if (active) {
        for (i = 0; i < NPRIO; i++) {
            int fc = PRIORITY[i];
            if (c->cond_mask & (1u << fc)) {
                /* spread is meaningful only once the machine is loaded up */
                if (fc == HIL_FC_EXH_SPREAD && c->state < CS_ACCEL) continue;
                trip(c, fc);
                break;
            }
        }
    }

    /* ================= start / reset bookkeeping ======================= */
    if (!in->start_cmd) c->start_lockout = 0;       /* start must be re-edged */

    int reset_edge = in->reset_cmd && !c->reset_prev;
    c->reset_prev = in->reset_cmd;
    if (reset_edge && c->state == CS_TRIP) {
        /* A reset is accepted only when the cause has gone. Sensor flags are
         * cleared first so a repaired sensor is re-judged from scratch. */
        SpeedVote probe = c->sv;
        speedvote_clear(&probe);
        unsigned blocking = c->cond_mask & ~(1u << HIL_FC_SENSOR_FAULT);
        if (blocking == 0 && in->estop_ok) {
            c->sv = probe;
            c->first_out = HIL_FC_NONE;
            c->start_lockout = in->start_cmd ? 1 : 0;
            enter(c, CS_COOLDOWN);                  /* coast-down then idle  */
        }
    }

    else if (reset_edge && (c->state == CS_IDLE || c->state == CS_COOLDOWN)) {
        speedvote_clear(&c->sv);              /* operator acknowledges latched sensor flags */
    }

    c->perm_word = ctrl_permissives(c, in);
    c->alarm_base = al;
    c->alarm_word = al | (c->missed_polls >= HIL_COMMS_ALARM_POLLS ? HIL_AL_COMMS : 0);

    const double floor = (c->state == CS_RUN || c->state == CS_WARMUP)
                         ? c->fsr_floor_run : c->fsr_floor_start;

    /* ================= sequencing ====================================== */
    switch (c->state) {
    case CS_IDLE:
        c->fsr_out = 0.0; c->starter_on = 0; c->ignition_on = 0;
        c->sync_permit = 0; c->trip_out = 0;
        c->i_gov = c->i_acc = c->i_temp = 0.0;
        c->fsr_start = c->fsr_light;
        if (in->start_cmd && !c->start_lockout) {
            if ((c->perm_word & PM_ALL) == PM_ALL) { enter(c, CS_CRANK); }
            else c->alarm_word |= HIL_AL_START_BLOCK;
        }
        break;

    case CS_CRANK:
        c->fsr_out = 0.0; c->starter_on = 1; c->ignition_on = 0;
        if (!in->start_cmd) { enter(c, CS_COOLDOWN); break; }
        if (c->t_state >= c->purge_s && n >= HIL_N_IGNITION_PU) enter(c, CS_IGNITE);
        break;

    case CS_IGNITE:
        c->starter_on = 1; c->ignition_on = 1;
        c->fsr_out = c->fsr_light;
        if (!in->start_cmd) { enter(c, CS_COOLDOWN); break; }
        if (in->flame && c->t_state >= 2.0) {
            c->i_gov = c->i_acc = c->i_temp = c->fsr_light;
            c->fsr_start = c->fsr_light;
            c->ignition_on = 0;
            enter(c, CS_ACCEL);
        } else if (c->t_state > c->light_timeout_s) {
            trip(c, HIL_FC_FAIL_TO_LIGHT);
        }
        break;

    case CS_ACCEL:
    case CS_WARMUP:
    case CS_RUN: {
        if (!in->start_cmd) { enter(c, CS_COOLDOWN); break; }
        if (!in->flame) { trip(c, HIL_FC_FLAMEOUT); break; }

        c->starter_on = (n < HIL_N_STARTER_CUTOUT_PU);

        /* Load rejection: fuel to the no-load preset the instant the breaker opens */
        if (on(c, FEAT_BREAKER_PRESET) && c->state == CS_RUN &&
            c->breaker_prev && !in->breaker_closed) {
            c->i_gov = c->fsr_noload;
        }

        c->fsr_start += c->fsr_start_rate * dt;
        if (c->fsr_start > 1.0) c->fsr_start = 1.0;

        /* governor, isochronous or droop (droop only while on the grid) */
        double droop_fb = 0.0;
        if (in->droop_mode && in->breaker_closed) {
            double frac = clamp01((c->fsr_gov - c->fsr_noload) / (1.0 - c->fsr_noload));
            droop_fb = c->droop_pu * frac;
        }
        double e_gov = c->n_ref_pu - n - droop_fb;
        c->fsr_gov  = on(c, FEAT_GOVERNOR) ? clamp01(c->kp_gov * e_gov + c->i_gov) : 1.0;

        double e_acc = c->accel_ref_pu_s - c->accel_f;
        c->fsr_acc  = on(c, FEAT_ACCEL_LIM) ? clamp01(c->kp_acc * e_acc + c->i_acc) : 1.0;

        double e_t = c->t_ref_c - t_exh;
        c->fsr_temp = on(c, FEAT_TEMP_LIM) ? clamp01(c->kp_t * e_t + c->i_temp) : 1.0;

        double sel = c->fsr_start; c->selected = SEL_START;
        if (c->fsr_acc  < sel) { sel = c->fsr_acc;  c->selected = SEL_ACCEL; }
        if (c->fsr_temp < sel) { sel = c->fsr_temp; c->selected = SEL_TEMP;  }
        if (c->fsr_gov  < sel) { sel = c->fsr_gov;  c->selected = SEL_GOV;   }
        if (sel < floor)       { sel = floor;       c->selected = SEL_FLOOR; }
        c->fsr_out = sel;

        double a = dt / c->track_tau_s;
        if (c->fsr_gov  > sel) c->i_gov  += (sel - c->fsr_gov)  * a; else c->i_gov  += c->ki_gov * e_gov * dt;
        if (c->fsr_acc  > sel) c->i_acc  += (sel - c->fsr_acc)  * a; else c->i_acc  += c->ki_acc * e_acc * dt;
        if (c->fsr_temp > sel + c->temp_headroom) c->i_temp += (sel + c->temp_headroom - c->fsr_temp) * a; else c->i_temp += c->ki_t   * e_t   * dt;
        c->i_gov  = clamp01(c->i_gov);
        c->i_acc  = clamp01(c->i_acc);
        c->i_temp = clamp01(c->i_temp);

        if (c->state == CS_ACCEL && n >= HIL_N_AT_SPEED_PU) enter(c, CS_WARMUP);
        else if (c->state == CS_WARMUP && c->t_state >= c->warmup_s &&
                 fabs(n - 1.0) <= HIL_SYNC_BAND_PU) enter(c, CS_RUN);

        c->sync_permit = (c->state == CS_RUN) && fabs(n - 1.0) <= HIL_SYNC_BAND_PU;
        break;
    }

    case CS_COOLDOWN:
        c->fsr_out = 0.0; c->starter_on = 0; c->ignition_on = 0;
        c->sync_permit = 0; c->trip_out = 1;
        if (n < HIL_N_STANDSTILL_PU) enter(c, CS_IDLE);
        break;

    case CS_TRIP:
        trip_outputs(c);
        break;
    }

    if (c->state == CS_TRIP) trip_outputs(c);   /* trip inside this cycle acts at once */
    if (c->state != CS_COOLDOWN && c->state != CS_TRIP) c->trip_out = 0;
    if (c->state != CS_RUN) c->sync_permit = 0;

    c->breaker_prev = in->breaker_closed;
}
