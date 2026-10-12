#include "sim.h"
#include "../include/hil_constants.h"
#include <stdlib.h>
#include <string.h>

struct Sim {
    Plant       plant;
    StandinCtrl ctrl;
    PlantMeasurements meas;
    int    start_cmd;
    double ctrl_timer;
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
    memset(&s->meas, 0, sizeof s->meas);
    s->start_cmd  = 0;
    s->ctrl_timer = 0.0;
    plant_measure(&s->plant, &s->meas);
}

void sim_set_start(Sim *s, int start)       { s->start_cmd = start ? 1 : 0; }
void sim_set_load(Sim *s, double load_mw)   { s->plant.in.load_elec_mw = load_mw; }
void sim_set_breaker(Sim *s, int closed)    { s->plant.in.breaker_closed = closed ? 1 : 0; }

void sim_step(Sim *s)
{
    /* Controller runs on its own, slower clock */
    s->ctrl_timer += HIL_STEP_S;
    if (s->ctrl_timer >= HIL_CONTROL_PERIOD_S - 1e-9) {
        s->ctrl_timer -= HIL_CONTROL_PERIOD_S;
        plant_measure(&s->plant, &s->meas);
        ctrl_update(&s->ctrl, HIL_CONTROL_PERIOD_S, s->start_cmd, &s->meas);
        s->plant.in.fuel_cmd    = s->ctrl.fsr_out;
        s->plant.in.starter_on  = s->ctrl.starter_on;
        s->plant.in.ignition_on = s->ctrl.ignition_on;
    }
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
    o->time_s         = s->plant.st.time;
    o->speed_rpm      = p->speed_rpm;
    o->speed_meas_rpm = s->meas.speed_rpm;
    o->n_pu           = p->n_pu;
    o->exh_gas_c      = p->t_exh_gas_c;
    o->exh_tc_c       = p->t_exh_tc_c;
    o->exh_meas_c     = s->meas.exh_temp_c;
    o->tit_k          = p->tit_k;
    o->press_bar_g    = p->p_cd_bar_g;
    o->power_mw       = p->p_load_elec_mw;
    o->load_mw        = s->plant.in.breaker_closed ? s->plant.in.load_elec_mw : 0.0;
    o->fuel_kg_s      = p->wf_kg_s;
    o->valve_pos      = p->valve_pos;
    o->fsr_cmd        = c->fsr_out;
    o->fsr_gov        = c->fsr_gov;
    o->fsr_acc        = c->fsr_acc;
    o->fsr_temp       = c->fsr_temp;
    o->fsr_start      = c->fsr_start;
    o->selected       = (double)c->selected;
    o->ctrl_state     = (double)c->state;
    o->tripped        = (c->state == CS_TRIP) ? 1.0 : 0.0;
    o->trip_cause     = (double)c->trip_cause;
    o->flame          = (double)p->flame;
    o->starter_on     = (double)s->plant.in.starter_on;
    o->breaker        = (double)s->plant.in.breaker_closed;
    o->ke_mj          = p->ke_j / 1.0e6;
    o->p_gas_net_mw   = p->p_gas_net_w / 1.0e6;
    o->p_comp_mw      = p->p_comp_w / 1.0e6;
    o->p_turb_mw      = p->p_turb_w / 1.0e6;
    o->p_shaft_net_mw = p->p_shaft_net_w / 1.0e6;
}

StandinCtrl *sim_ctrl(Sim *s)  { return &s->ctrl; }
Plant       *sim_plant(Sim *s) { return &s->plant; }
