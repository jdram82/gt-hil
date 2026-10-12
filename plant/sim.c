#include "sim.h"
#include "../include/hil_constants.h"
#include <stdlib.h>
#include <string.h>

struct Sim {
    Plant       plant;
    StandinCtrl ctrl;
    PlantMeasurements meas;
    CtrlInput   in;               /* operator inputs + last poll */
    int    link_up;
    int    breaker_req;
    double ctrl_timer;
    unsigned features;
    int    noise;

    /* commands as last written to the plant over the link */
    double w_fuel;
    int    w_starter, w_ignition, w_sync, w_trip;
    double wd_timer;
    int    wd_active;
};

Sim *sim_create(void)
{
    Sim *s = (Sim *)calloc(1, sizeof *s);
    if (s) sim_reset(s, 12345ULL, 1);
    return s;
}

void sim_destroy(Sim *s) { free(s); }

void sim_reset(Sim *s, unsigned long long seed, int noise_enabled)
{
    PlantParams p;
    plant_params_default(&p);
    plant_init(&s->plant, &p);
    plant_seed(&s->plant, seed, noise_enabled);
    ctrl_init(&s->ctrl);
    s->noise = noise_enabled;
    s->ctrl.sv.use_flat = noise_enabled ? 1 : 0;   /* flat-line test needs noise */
    s->features = FEAT_ALL;
    memset(&s->meas, 0, sizeof s->meas);
    memset(&s->in, 0, sizeof s->in);
    s->link_up = 1;
    s->breaker_req = 0;
    s->ctrl_timer = 0.0;
    s->w_fuel = 0.0; s->w_starter = s->w_ignition = s->w_sync = 0; s->w_trip = 0;
    s->wd_timer = 0.0; s->wd_active = 0;
    plant_measure(&s->plant, &s->meas);
}

void sim_set_start(Sim *s, int v)   { s->in.start_cmd  = v ? 1 : 0; }
void sim_set_reset(Sim *s, int v)   { s->in.reset_cmd  = v ? 1 : 0; }
void sim_set_droop(Sim *s, int v)   { s->in.droop_mode = v ? 1 : 0; }
void sim_set_load(Sim *s, double mw){ s->plant.in.load_elec_mw = mw; }
void sim_set_breaker(Sim *s, int c) { s->breaker_req = c ? 1 : 0; }
void sim_set_link(Sim *s, int up)   { s->link_up = up ? 1 : 0; }

void sim_set_permissive(Sim *s, int which, int ok)
{
    ok = ok ? 1 : 0;
    switch (which) {
    case 0: s->plant.in.perm_estop_ok   = ok; break;
    case 1: s->plant.in.perm_lube_ok    = ok; break;
    case 2: s->plant.in.perm_fuel_ok    = ok; break;
    case 3: s->plant.in.perm_starter_ok = ok; break;
    default: break;
    }
}

void sim_inject_speed_fault(Sim *s, int ch, int mode, double a, double b)
{ plant_inject_speed_fault(&s->plant, ch, mode, a, b); }
void sim_inject_tc_fault(Sim *s, int tc, int mode, double a, double b)
{ plant_inject_tc_fault(&s->plant, tc, mode, a, b); }
void sim_clear_faults(Sim *s) { plant_clear_faults(&s->plant); }
void sim_set_ambient_k(Sim *s, double k) { s->plant.par.t_amb_k = k; }

void sim_set_features(Sim *s, unsigned mask)
{
    s->features = mask;
    s->ctrl.features = mask;
}

static void controller_cycle(Sim *s)
{
    CtrlInput *in = &s->in;
    int i;
    in->link_ok = s->link_up;

    if (s->link_up) {
        plant_measure(&s->plant, &s->meas);
        for (i = 0; i < 3; i++) in->speed_rpm[i] = s->meas.speed_rpm[i];
        for (i = 0; i < 4; i++) in->tc_c[i]      = s->meas.tc_c[i];
        in->flame = s->meas.flame;
        in->breaker_closed = s->meas.breaker_closed;
        in->estop_ok = s->meas.estop_ok; in->lube_ok = s->meas.lube_ok;
        in->fuel_ok  = s->meas.fuel_ok;  in->starter_ok = s->meas.starter_ok;
    }

    ctrl_update(&s->ctrl, HIL_CONTROL_PERIOD_S, in);

    if (s->link_up) {                       /* write answered: apply and feed the watchdog */
        s->w_fuel     = s->ctrl.fsr_out;
        s->w_starter  = s->ctrl.starter_on;
        s->w_ignition = s->ctrl.ignition_on;
        s->w_sync     = s->ctrl.sync_permit;
        s->w_trip     = s->ctrl.trip_out;
        s->wd_timer   = 0.0;
    }
}

static void plant_side(Sim *s)
{
    PlantInputs *pi = &s->plant.in;

    /* plant-side watchdog: no write for HIL_PLANT_WD_S -> fail safe */
    s->wd_timer += HIL_STEP_S;
    s->wd_active = ((s->features & FEAT_PLANT_WD) && s->wd_timer >= HIL_PLANT_WD_S);

    if (s->wd_active) {
        pi->fuel_cmd = 0.0; pi->starter_on = 0; pi->ignition_on = 0;
    } else {
        pi->fuel_cmd = s->w_fuel; pi->starter_on = s->w_starter; pi->ignition_on = s->w_ignition;
    }

    /* breaker: opens on trip / watchdog / request withdrawn; closes only with permit */
    if (s->w_trip || s->wd_active) {
        s->breaker_req = 0;
        pi->breaker_closed = 0;
    } else if (!s->breaker_req) {
        pi->breaker_closed = 0;
    } else if (!pi->breaker_closed && s->w_sync) {
        pi->breaker_closed = 1;
    }
}

void sim_step(Sim *s)
{
    s->ctrl_timer += HIL_STEP_S;
    if (s->ctrl_timer >= HIL_CONTROL_PERIOD_S - 1e-9) {
        s->ctrl_timer -= HIL_CONTROL_PERIOD_S;
        controller_cycle(s);
    }
    plant_side(s);
    plant_step(&s->plant, HIL_STEP_S);
}

double sim_rated_power_mw(const Sim *s)
{
    return s->plant.par.p_design_w * s->plant.par.eta_gen / 1.0e6;
}

int sim_out_doubles(void) { return (int)(sizeof(SimOut) / sizeof(double)); }

void sim_get(const Sim *s, SimOut *o)
{
    const PlantOutputs *p = &s->plant.out;
    const StandinCtrl  *c = &s->ctrl;
    o->time_s          = s->plant.st.time;
    o->speed_rpm       = p->speed_rpm;
    o->speed1_rpm      = s->in.speed_rpm[0];
    o->speed2_rpm      = s->in.speed_rpm[1];
    o->speed3_rpm      = s->in.speed_rpm[2];
    o->speed_voted_rpm = c->speed_voted_rpm;
    o->n_pu            = p->n_pu;
    o->exh_gas_c       = p->t_exh_gas_c;
    o->exh_tc_c        = p->t_exh_tc_c;
    o->tc1_c = s->in.tc_c[0]; o->tc2_c = s->in.tc_c[1];
    o->tc3_c = s->in.tc_c[2]; o->tc4_c = s->in.tc_c[3];
    o->exh_voted_c     = c->t_voted_c;
    o->exh_spread_c    = c->t_spread_c;
    o->tit_k           = p->tit_k;
    o->press_bar_g     = p->p_cd_bar_g;
    o->power_mw        = p->p_load_elec_mw;
    o->load_mw         = s->plant.in.breaker_closed ? s->plant.in.load_elec_mw : 0.0;
    o->fuel_kg_s       = p->wf_kg_s;
    o->valve_pos       = p->valve_pos;
    o->fsr_cmd         = c->fsr_out;
    o->fsr_gov         = c->fsr_gov;
    o->fsr_acc         = c->fsr_acc;
    o->fsr_temp        = c->fsr_temp;
    o->fsr_start       = c->fsr_start;
    o->selected        = (double)c->selected;
    o->ctrl_state      = (double)c->state;
    o->tripped         = (c->state == CS_TRIP) ? 1.0 : 0.0;
    o->trip_cause      = (double)c->first_out;
    o->alarm_word      = (double)c->alarm_word;
    o->perm_word       = (double)c->perm_word;
    o->sensor_bad_word = (double)(c->sv.bad[0] | (c->sv.bad[1] << 1) | (c->sv.bad[2] << 2));
    o->flame           = (double)p->flame;
    o->starter_on      = (double)s->plant.in.starter_on;
    o->breaker         = (double)s->plant.in.breaker_closed;
    o->sync_permit     = (double)c->sync_permit;
    o->link_up         = (double)s->link_up;
    o->plant_wd        = (double)s->wd_active;
    o->missed_polls    = (double)c->missed_polls;
    o->ke_mj           = p->ke_j / 1.0e6;
    o->p_gas_net_mw    = p->p_gas_net_w / 1.0e6;
    o->p_comp_mw       = p->p_comp_w / 1.0e6;
    o->p_turb_mw       = p->p_turb_w / 1.0e6;
    o->p_shaft_net_mw  = p->p_shaft_net_w / 1.0e6;
}

StandinCtrl *sim_ctrl(Sim *s)  { return &s->ctrl; }
Plant       *sim_plant(Sim *s) { return &s->plant; }

void sim_set_temp_tuning(Sim *s, double kp, double ki, double hr)
{ s->ctrl.kp_t = kp; s->ctrl.ki_t = ki; s->ctrl.temp_headroom = hr; }
