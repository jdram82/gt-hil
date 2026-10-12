/*
 * plant.c - single-shaft gas turbine generator set model.
 * See plant.h for the interface and docs/physics.md for the derivation.
 */
#include "plant.h"
#include <math.h>
#include <string.h>

#define TWO_PI           6.28318530717958647692
#define N_FLOOR          0.03      /* minimum speed used for flow scaling      */
#define OMEGA_FLOOR_FRAC 0.02      /* minimum speed used when dividing P by w  */
#define TIT_MAX_K        2000.0    /* numerical guard, far above any real TIT  */

/* ------------------------------------------------------------------ */
/* Thermodynamic state of the gas path at one operating point          */
/* ------------------------------------------------------------------ */
typedef struct {
    double ma;      /* air mass flow, kg/s                  */
    double mg;      /* gas mass flow through turbine, kg/s  */
    double r;       /* compressor pressure ratio            */
    double r_t;     /* turbine expansion ratio              */
    double t_cd;    /* compressor discharge temperature, K  */
    double tit;     /* turbine inlet temperature, K         */
    double t_exh;   /* turbine exhaust gas temperature, K   */
    double w_c;     /* compressor power absorbed, W         */
    double w_t;     /* turbine power produced, W            */
} Thermo;

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static double air_flow(const PlantParams *P, double n_pu)
{
    double n = n_pu < N_FLOOR ? N_FLOOR : n_pu;
    double dens = (P->p_amb_bar / 1.01325) * (288.15 / P->t_amb_k);
    return P->ma_rated * n * dens;
}

static void thermo(const PlantParams *P, double n_pu, double wf, int flame, Thermo *t)
{
    double n = n_pu < N_FLOOR ? N_FLOOR : n_pu;

    /* Compressor: flow ~ speed, pressure ratio ~ speed squared */
    t->ma  = air_flow(P, n_pu);
    t->r   = 1.0 + (P->r_rated - 1.0) * n * n;
    t->t_cd = P->t_amb_k * (1.0 + (pow(t->r, P->kappa_air) - 1.0) / P->eta_comp);
    t->w_c = t->ma * P->cp_air * (t->t_cd - P->t_amb_k);

    /* Combustor: heat release raises gas temperature */
    double q = flame ? wf * P->lhv * P->eta_comb : 0.0;
    t->mg  = t->ma + (flame ? wf : 0.0);
    t->tit = t->t_cd + q / (t->mg * P->cp_gas);
    if (t->tit > TIT_MAX_K) t->tit = TIT_MAX_K;

    /* Turbine: expansion ratio reduced by flow-squared pressure losses */
    t->r_t = t->r * (1.0 - P->dp_frac * n * n);
    if (t->r_t < 1.0) t->r_t = 1.0;
    t->t_exh = t->tit * (1.0 - P->eta_turb * (1.0 - pow(t->r_t, -P->kappa_gas)));
    t->w_t = t->mg * P->cp_gas * (t->tit - t->t_exh);

    /* Below the flow floor the model would otherwise hold a constant residual
     * power (and therefore a constant drag torque) as speed -> 0.  Real gas-path
     * power falls roughly with speed cubed, so fade it out smoothly. */
    if (n_pu < N_FLOOR) {
        double s = n_pu > 0.0 ? n_pu / N_FLOOR : 0.0;
        s = s * s * s;
        t->w_c *= s;
        t->w_t *= s;
    }
}

/* ------------------------------------------------------------------ */
void plant_params_default(PlantParams *p)
{
    memset(p, 0, sizeof *p);
    p->t_amb_k      = 288.15;
    p->p_amb_bar    = 1.01325;

    p->cp_air       = 1005.0;
    p->cp_gas       = 1150.0;
    p->kappa_air    = 0.2857;   /* gamma = 1.40 */
    p->kappa_gas    = 0.2481;   /* gamma = 1.33 */
    p->lhv          = 47.0e6;   /* natural gas, approx. */
    p->eta_comb     = 0.99;

    p->r_rated      = 12.0;
    p->eta_comp     = 0.85;
    p->eta_turb     = 0.88;
    p->dp_frac      = 0.06;
    p->ma_rated     = 80.0;
    p->wf_design    = 1.50;
    p->wf_max       = 1.90;
    p->far_blowout  = 0.0020;
    p->far_light    = 0.0030;
    p->n_light_min_pu = 0.10;

    p->rated_rpm    = 3500.0;
    p->h_inertia_s  = 2.5;
    p->p_fric_frac  = 0.008;
    p->eta_gen      = 0.98;
    p->load_damping = 1.0;

    p->tau_valve_s  = 0.20;
    p->tau_fuel_s   = 0.30;
    p->tau_tc_s     = 3.0;

    p->starter_torque_nm = 12000.0;   /* must beat compressor drag up to light-off speed */
    p->starter_power_w   = 1.0e6;

    p->noise_speed_rpm = 1.0;
    p->noise_temp_c    = 0.25;
    p->tc_offset_c[0]  =  3.0;
    p->tc_offset_c[1]  = -2.0;
    p->tc_offset_c[2]  =  1.0;
    p->tc_offset_c[3]  = -2.0;
}

void plant_seed(Plant *pl, unsigned long long seed, int noise_enabled)
{
    pl->rng = seed ^ 0x9E3779B97F4A7C15ULL;
    if (pl->rng == 0) pl->rng = 0x2545F4914F6CDD1DULL;
    pl->noise_enabled = noise_enabled;
}

static double rnd_pm1(Plant *pl)
{
    unsigned long long x = pl->rng;
    x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
    pl->rng = x;
    unsigned long long y = x * 2685821657736338717ULL;
    return ((double)(y >> 11) / 9007199254740992.0) * 2.0 - 1.0;
}

static void update_outputs(Plant *pl, const Thermo *th, double p_fric, double p_load_mech,
                           double p_starter)
{
    const PlantParams *P = &pl->par;
    PlantOutputs *o = &pl->out;
    double n = pl->st.omega / P->omega_rated;

    o->n_pu         = n;
    o->speed_rpm    = n * P->rated_rpm;
    o->p_cd_bar_g   = P->p_amb_bar * (th->r - 1.0);
    o->t_cd_k       = th->t_cd;
    o->tit_k        = th->tit;
    o->t_exh_gas_k  = th->t_exh;
    o->t_exh_gas_c  = th->t_exh - 273.15;
    o->t_exh_tc_c   = pl->st.t_tc_k - 273.15;
    o->ma_kg_s      = th->ma;
    o->wf_kg_s      = pl->st.wf;
    o->valve_pos    = pl->st.valve;
    o->p_comp_w     = th->w_c;
    o->p_turb_w     = th->w_t;
    o->p_gas_net_w  = th->w_t - th->w_c;
    o->p_fric_w     = p_fric;
    o->p_load_mech_w = p_load_mech;
    o->p_load_elec_mw = p_load_mech * P->eta_gen / 1.0e6;
    o->p_starter_w  = p_starter;
    o->p_shaft_net_w = o->p_gas_net_w - p_fric - p_load_mech + p_starter;
    o->ke_j         = 0.5 * P->j_kgm2 * pl->st.omega * pl->st.omega;
    o->flame        = pl->st.flame;
}

void plant_init(Plant *pl, const PlantParams *params)
{
    memset(pl, 0, sizeof *pl);
    pl->par = *params;
    PlantParams *P = &pl->par;

    P->omega_rated = P->rated_rpm * TWO_PI / 60.0;

    /* Design point: n = 1, wf = wf_design, flame on */
    Thermo th;
    thermo(P, 1.0, P->wf_design, 1, &th);
    P->p_gas_design_w = th.w_t - th.w_c;
    P->p_fric_rated_w = P->p_fric_frac * P->p_gas_design_w;
    P->p_design_w     = P->p_gas_design_w - P->p_fric_rated_w;

    /* J from the inertia constant: H = 0.5 J w^2 / P_design */
    P->j_kgm2 = 2.0 * P->h_inertia_s * P->p_design_w / (P->omega_rated * P->omega_rated);

    pl->in.perm_estop_ok = pl->in.perm_lube_ok = 1;
    pl->in.perm_fuel_ok  = pl->in.perm_starter_ok = 1;

    pl->st.omega  = 0.0;
    pl->st.valve  = 0.0;
    pl->st.wf     = 0.0;
    pl->st.t_tc_k = P->t_amb_k;
    pl->st.flame  = 0;
    pl->st.time   = 0.0;
    plant_seed(pl, 12345ULL, 1);

    thermo(P, 0.0, 0.0, 0, &th);
    update_outputs(pl, &th, 0.0, 0.0, 0.0);
}

void plant_step(Plant *pl, double dt)
{
    const PlantParams *P = &pl->par;
    PlantState *s = &pl->st;
    const PlantInputs *in = &pl->in;

    double n = s->omega / P->omega_rated;
    double omega_f = s->omega > OMEGA_FLOOR_FRAC * P->omega_rated
                   ? s->omega : OMEGA_FLOOR_FRAC * P->omega_rated;

    /* ---- Flame logic: light-off needs ignition + enough fuel/air + speed;
     *      lean blowout extinguishes the flame ---- */
    double far = s->wf / air_flow(P, n);
    if (s->flame) {
        if (far < P->far_blowout) s->flame = 0;
    } else if (in->ignition_on && n >= P->n_light_min_pu && far >= P->far_light) {
        s->flame = 1;
    }

    /* ---- Gas path at the current state ---- */
    Thermo th;
    thermo(P, n, s->wf, s->flame, &th);
    double p_gas_net = th.w_t - th.w_c;

    /* ---- Rotor torques ---- */
    double p_fric = P->p_fric_rated_w * n * n;

    double p_load_mech = 0.0;
    if (in->breaker_closed) {
        double f = 1.0 + P->load_damping * (n - 1.0);
        if (f < 0.0) f = 0.0;
        p_load_mech = in->load_elec_mw * 1.0e6 * f / P->eta_gen;
    }

    double tau_starter = 0.0;
    if (in->starter_on) {
        tau_starter = P->starter_power_w / omega_f;
        if (tau_starter > P->starter_torque_nm) tau_starter = P->starter_torque_nm;
    }
    double p_starter = tau_starter * s->omega;

    double tau_net = (p_gas_net - p_fric - p_load_mech) / omega_f + tau_starter;
    double domega  = tau_net / P->j_kgm2;

    update_outputs(pl, &th, p_fric, p_load_mech, p_starter);

    /* ---- Advance states (explicit Euler; dt << smallest time constant) ---- */
    double cmd = clampd(in->fuel_cmd, 0.0, 1.0);
    s->valve  += (cmd - s->valve) * dt / P->tau_valve_s;
    s->wf     += (s->valve * P->wf_max - s->wf) * dt / P->tau_fuel_s;
    s->t_tc_k += (th.t_exh - s->t_tc_k) * dt / P->tau_tc_s;
    s->omega  += domega * dt;
    if (s->omega < 0.0) s->omega = 0.0;
    s->time   += dt;
}

static double apply_fault(Plant *pl, SensorFault *f, double v)
{
    if (f->mode == SF_NONE) return v;
    double age = pl->st.time - f->t_on;
    if (age < 0.0) return v;
    switch (f->mode) {
    case SF_DRIFT: {
        double d = f->a * age;
        if (f->b > 0.0) {
            if (d >  f->b) d =  f->b;
            if (d < -f->b) d = -f->b;
        }
        return v + d;
    }
    case SF_STUCK:
        if (!f->captured) { f->held = v; f->captured = 1; }
        return f->held;
    case SF_NOISY:  return v + f->a * rnd_pm1(pl);
    case SF_OFFSET: return v + f->a;
    default:        return v;
    }
}

void plant_measure(Plant *pl, PlantMeasurements *m)
{
    const PlantOutputs *o = &pl->out;
    const PlantParams *P = &pl->par;

    for (int i = 0; i < 3; i++) {
        double v = o->speed_rpm + (pl->noise_enabled ? P->noise_speed_rpm * rnd_pm1(pl) : 0.0);
        v = apply_fault(pl, &pl->speed_fault[i], v);
        m->speed_rpm[i] = v < 0.0 ? 0.0 : v;
    }
    for (int i = 0; i < 4; i++) {
        double v = o->t_exh_tc_c + P->tc_offset_c[i]
                 + (pl->noise_enabled ? P->noise_temp_c * rnd_pm1(pl) : 0.0);
        m->tc_c[i] = apply_fault(pl, &pl->tc_fault[i], v);
    }
    m->press_bar_g    = o->p_cd_bar_g;
    m->power_mw       = o->p_load_elec_mw;
    m->fuel_kg_s      = o->wf_kg_s;
    m->flame          = o->flame;
    m->breaker_closed = pl->in.breaker_closed;
    m->estop_ok       = pl->in.perm_estop_ok;
    m->lube_ok        = pl->in.perm_lube_ok;
    m->fuel_ok        = pl->in.perm_fuel_ok;
    m->starter_ok     = pl->in.perm_starter_ok;
}

void plant_inject_speed_fault(Plant *pl, int ch, int mode, double a, double b)
{
    if (ch < 0 || ch > 2) return;
    SensorFault *f = &pl->speed_fault[ch];
    memset(f, 0, sizeof *f);
    f->mode = mode; f->a = a; f->b = b; f->t_on = pl->st.time;
}

void plant_inject_tc_fault(Plant *pl, int tc, int mode, double a, double b)
{
    if (tc < 0 || tc > 3) return;
    SensorFault *f = &pl->tc_fault[tc];
    memset(f, 0, sizeof *f);
    f->mode = mode; f->a = a; f->b = b; f->t_on = pl->st.time;
}

void plant_clear_faults(Plant *pl)
{
    memset(pl->speed_fault, 0, sizeof pl->speed_fault);
    memset(pl->tc_fault, 0, sizeof pl->tc_fault);
}

/* Bisection: fuel flow that balances a requested net shaft power at speed n_pu. */
double plant_fuel_for_shaft_power(const PlantParams *params, double n_pu, double p_shaft_w)
{
    /* Work on a copy so derived quantities exist even if the caller passed raw defaults */
    Plant tmp;
    plant_init(&tmp, params);
    const PlantParams *P = &tmp.par;

    double p_fric = P->p_fric_rated_w * n_pu * n_pu;
    double lo = 0.0, hi = P->wf_max;
    Thermo th;

    thermo(P, n_pu, hi, 1, &th);
    if ((th.w_t - th.w_c - p_fric) < p_shaft_w) return -1.0;

    for (int i = 0; i < 80; i++) {
        double mid = 0.5 * (lo + hi);
        thermo(P, n_pu, mid, 1, &th);
        if ((th.w_t - th.w_c - p_fric) < p_shaft_w) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}
