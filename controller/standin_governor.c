/*
 * standin_governor.c - min-select fuel controller (stand-in for week 3).
 */
#include "standin_governor.h"
#include "../include/hil_constants.h"
#include <string.h>

static double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

void ctrl_init(StandinCtrl *c)
{
    memset(c, 0, sizeof *c);

    c->n_ref_pu        = 1.0;

    c->kp_gov          = 6.0;
    c->ki_gov          = 2.5;

    c->accel_ref_pu_s  = 0.020;      /* 2 % speed per second */
    c->kp_acc          = 5.0;
    c->ki_acc          = 15.0;

    c->t_ref_c         = HIL_EXH_TEMP_CONTROL_C;
    c->kp_t            = 0.0006;
    c->ki_t            = 0.0004;

    c->fsr_light       = 0.050;
    c->fsr_start_rate  = 0.020;
    c->fsr_floor_start = 0.040;
    c->fsr_floor_run   = 0.100;
    c->fsr_noload      = 0.140;
    c->use_breaker_preset = 1;

    c->purge_s         = 5.0;
    c->light_timeout_s = 8.0;
    c->track_tau_s     = 0.5;
    c->accel_filter_s  = 0.5;

    c->state = CS_IDLE;
}

static void enter(StandinCtrl *c, CtrlState s)
{
    c->state   = s;
    c->t_state = 0.0;
}

static void trip(StandinCtrl *c, TripCause cause)
{
    c->trip_cause = cause;
    enter(c, CS_TRIP);
}

void ctrl_update(StandinCtrl *c, double dt, int start_cmd, const PlantMeasurements *m)
{
    const double n = m->speed_rpm / HIL_RATED_SPEED_RPM;
    const double t_exh = m->exh_temp_c;

    /* Filtered acceleration (pu/s) */
    double accel_raw = (n - c->n_prev) / dt;
    c->n_prev = n;
    c->accel_f += (accel_raw - c->accel_f) * dt / c->accel_filter_s;

    c->t_state += dt;

    /* ---- Protection: independent of the control path ---- */
    if (c->state != CS_IDLE && c->state != CS_TRIP) {
        if (m->speed_rpm >= HIL_OVERSPEED_TRIP_RPM) trip(c, TRIP_OVERSPEED);
        else if (t_exh >= HIL_EXH_TEMP_TRIP_C)      trip(c, TRIP_OVERTEMP);
    }

    double floor = (c->state == CS_RUN) ? c->fsr_floor_run : c->fsr_floor_start;

    /* ---- Sequencing ---- */
    switch (c->state) {
    case CS_IDLE:
        c->fsr_out = 0.0; c->starter_on = 0; c->ignition_on = 0;
        c->i_gov = c->i_acc = c->i_temp = 0.0;
        c->fsr_start = c->fsr_light;
        c->trip_cause = TRIP_NONE;
        if (start_cmd) enter(c, CS_CRANK);
        break;

    case CS_CRANK:
        c->fsr_out = 0.0; c->starter_on = 1; c->ignition_on = 0;
        if (!start_cmd) { enter(c, CS_COOLDOWN); break; }
        if (c->t_state >= c->purge_s && n >= HIL_N_IGNITION_PU) enter(c, CS_IGNITE);
        break;

    case CS_IGNITE:
        c->starter_on = 1; c->ignition_on = 1;
        c->fsr_out = c->fsr_light;
        if (!start_cmd) { enter(c, CS_COOLDOWN); break; }
        if (m->flame && c->t_state >= 2.0) {
            /* hand over to the min-select; seed the integrators so the
             * hand-over is smooth */
            c->i_gov = c->i_acc = c->i_temp = c->fsr_light;
            c->fsr_start = c->fsr_light;
            c->ignition_on = 0;
            enter(c, CS_ACCEL);
        } else if (c->t_state > c->light_timeout_s) {
            trip(c, TRIP_FAIL_TO_LIGHT);
        }
        break;

    case CS_ACCEL:
    case CS_RUN:
        if (!start_cmd) { enter(c, CS_COOLDOWN); break; }
        if (!m->flame) { trip(c, TRIP_FLAMEOUT); break; }

        /* starter assists until cut-out speed */
        c->starter_on = (n < HIL_N_STARTER_CUTOUT_PU);

        /* Load rejection: fuel to the no-load preset the instant the breaker opens */
        if (c->use_breaker_preset && c->state == CS_RUN &&
            c->breaker_prev && !m->breaker_closed) {
            c->i_gov = c->fsr_noload;
        }

        /* --- the four fuel requests --- */
        c->fsr_start += c->fsr_start_rate * dt;
        if (c->fsr_start > 1.0) c->fsr_start = 1.0;

        double e_gov = c->n_ref_pu - n;
        c->fsr_gov  = clamp01(c->kp_gov * e_gov + c->i_gov);

        double e_acc = c->accel_ref_pu_s - c->accel_f;
        c->fsr_acc  = clamp01(c->kp_acc * e_acc + c->i_acc);

        double e_t = c->t_ref_c - t_exh;
        c->fsr_temp = clamp01(c->kp_t * e_t + c->i_temp);

        /* --- minimum select --- */
        double sel = c->fsr_start; c->selected = SEL_START;
        if (c->fsr_acc  < sel) { sel = c->fsr_acc;  c->selected = SEL_ACCEL; }
        if (c->fsr_temp < sel) { sel = c->fsr_temp; c->selected = SEL_TEMP;  }
        if (c->fsr_gov  < sel) { sel = c->fsr_gov;  c->selected = SEL_GOV;   }
        if (sel < floor)       { sel = floor;       c->selected = SEL_FLOOR; }
        c->fsr_out = sel;

        /* --- integrators: run when selected, track when not (anti-windup) --- */
        double a = dt / c->track_tau_s;
        if (c->fsr_gov  > sel) c->i_gov  += (sel - c->fsr_gov)  * a; else c->i_gov  += c->ki_gov * e_gov * dt;
        if (c->fsr_acc  > sel) c->i_acc  += (sel - c->fsr_acc)  * a; else c->i_acc  += c->ki_acc * e_acc * dt;
        if (c->fsr_temp > sel) c->i_temp += (sel - c->fsr_temp) * a; else c->i_temp += c->ki_t   * e_t   * dt;
        c->i_gov  = clamp01(c->i_gov);
        c->i_acc  = clamp01(c->i_acc);
        c->i_temp = clamp01(c->i_temp);

        if (c->state == CS_ACCEL && n >= HIL_N_AT_SPEED_PU) enter(c, CS_RUN);
        break;

    case CS_COOLDOWN:
        c->fsr_out = 0.0; c->starter_on = 0; c->ignition_on = 0;
        if (n < 0.05) enter(c, CS_IDLE);
        break;

    case CS_TRIP:
        c->fsr_out = 0.0; c->starter_on = 0; c->ignition_on = 0;
        if (!start_cmd && n < 0.05) { c->trip_cause = TRIP_NONE; enter(c, CS_IDLE); }
        break;
    }

    c->breaker_prev = m->breaker_closed;
}
