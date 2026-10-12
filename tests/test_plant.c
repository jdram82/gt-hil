/*
 * test_plant.c - unit tests for the plant model and the closed-loop smoke test.
 *
 * Every test checks a physical law or an engineering expectation, not just
 * "the code runs": energy conservation, rotor equation, flame logic, and so on.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../plant/plant.h"
#include "../plant/sim.h"
#include "../include/hil_constants.h"

static int g_fail = 0, g_pass = 0;

#define CHECK(cond, ...) do { \
    if (cond) { g_pass++; printf("  [ ok ] "); } \
    else      { g_fail++; printf("  [FAIL] "); } \
    printf(__VA_ARGS__); printf("\n"); } while (0)

/* Put a plant at rated speed in steady state carrying load_mw (electrical) */
static void steady_at_rated(Plant *pl, double load_mw)
{
    PlantParams p;
    plant_params_default(&p);
    plant_init(pl, &p);
    plant_seed(pl, 1, 0);                       /* no noise */
    const PlantParams *P = &pl->par;

    pl->in.breaker_closed = 1;
    pl->in.load_elec_mw   = load_mw;
    pl->st.omega = P->omega_rated;
    pl->st.flame = 1;

    double p_load_mech = load_mw * 1.0e6 / P->eta_gen;     /* load factor = 1 at n = 1 */
    double wf = plant_fuel_for_shaft_power(P, 1.0, p_load_mech);
    pl->st.wf    = wf;
    pl->st.valve = wf / P->wf_max;
    pl->in.fuel_cmd = pl->st.valve;
    pl->st.t_tc_k = 273.15 + 500.0;
    for (int i = 0; i < 4000; i++) plant_step(pl, HIL_STEP_S);   /* let the thermocouple settle */
}

static void test_design_point(void)
{
    printf("Design point\n");
    PlantParams p; plant_params_default(&p);
    Plant pl; plant_init(&pl, &p);
    const PlantParams *P = &pl.par;

    double wf = plant_fuel_for_shaft_power(P, 1.0, P->p_design_w);
    CHECK(fabs(wf - P->wf_design) / P->wf_design < 0.005,
          "fuel for design power = %.4f kg/s (design %.4f)", wf, P->wf_design);

    double eta = P->p_design_w / (P->wf_design * P->lhv);
    CHECK(eta > 0.28 && eta < 0.40, "thermal efficiency %.1f %% in 28..40 %%", eta * 100);
    CHECK(P->p_design_w > 18e6 && P->p_design_w < 30e6, "design shaft power %.1f MW", P->p_design_w / 1e6);
    CHECK(P->j_kgm2 > 300 && P->j_kgm2 < 3000, "rotor inertia %.0f kg m^2", P->j_kgm2);

    Plant s; steady_at_rated(&s, P->p_design_w * P->eta_gen / 1e6);
    CHECK(s.out.t_exh_gas_c > 480 && s.out.t_exh_gas_c < 620,
          "exhaust temperature at full load %.0f degC", s.out.t_exh_gas_c);
    CHECK(s.out.tit_k - 273.15 > 1000 && s.out.tit_k - 273.15 < 1300,
          "turbine inlet temperature %.0f degC", s.out.tit_k - 273.15);
    CHECK(s.out.p_cd_bar_g > 10 && s.out.p_cd_bar_g < 12, "discharge pressure %.1f bar(g)", s.out.p_cd_bar_g);
}

static void test_no_load_fuel(void)
{
    printf("No-load fuel\n");
    PlantParams p; plant_params_default(&p);
    Plant pl; plant_init(&pl, &p);
    double wf0 = plant_fuel_for_shaft_power(&pl.par, 1.0, pl.par.p_fric_rated_w);
    double frac = wf0 / pl.par.wf_design;
    CHECK(frac > 0.10 && frac < 0.30, "no-load fuel is %.0f %% of design fuel (expect 10..30 %%)", frac * 100);
    double far = wf0 / pl.par.ma_rated;
    CHECK(far > pl.par.far_blowout, "no-load fuel/air %.4f above blow-out %.4f", far, pl.par.far_blowout);
}

static void test_equilibrium(void)
{
    printf("Steady equilibrium\n");
    Plant pl; steady_at_rated(&pl, 11.0);
    double n0 = pl.out.n_pu;
    for (int i = 0; i < 2000; i++) plant_step(&pl, HIL_STEP_S);   /* 20 s */
    CHECK(fabs(pl.out.n_pu - n0) < 0.001, "speed holds at balance: drift %.5f pu in 20 s", pl.out.n_pu - n0);
}

static void test_rotor_equation(void)
{
    printf("Rotor equation (load step, fuel held)\n");
    Plant pl; steady_at_rated(&pl, 11.0);
    const PlantParams *P = &pl.par;

    /* remove the load: surplus power is the old load (plus nothing else) */
    double surplus_w = 11.0e6 / P->eta_gen;
    pl.in.breaker_closed = 0;
    double w0 = pl.st.omega;
    plant_step(&pl, HIL_STEP_S);
    double domega_dt = (pl.st.omega - w0) / HIL_STEP_S;
    double expected  = surplus_w / (P->j_kgm2 * w0);
    CHECK(fabs(domega_dt - expected) / expected < 0.03,
          "d(omega)/dt = %.3f rad/s^2, expected P/(J w) = %.3f", domega_dt, expected);

    double n_prev = pl.out.n_pu; int monotone = 1;
    for (int i = 0; i < 300; i++) {
        plant_step(&pl, HIL_STEP_S);
        if (pl.out.n_pu < n_prev - 1e-9) monotone = 0;
        n_prev = pl.out.n_pu;
    }
    CHECK(monotone, "speed rises monotonically after load loss with fuel held");
}

static void test_energy_conservation(void)
{
    printf("Energy conservation\n");
    PlantParams p; plant_params_default(&p);
    Plant pl; plant_init(&pl, &p);
    plant_seed(&pl, 1, 0);
    pl.st.omega = 0.5 * pl.par.omega_rated;
    pl.st.flame = 1;
    pl.st.wf = 0.45; pl.st.valve = 0.45 / pl.par.wf_max; pl.in.fuel_cmd = pl.st.valve;
    pl.in.breaker_closed = 0;

    double ke0 = 0.5 * pl.par.j_kgm2 * pl.st.omega * pl.st.omega;
    double work = 0.0;
    for (int i = 0; i < 1500; i++) {                    /* 15 s */
        plant_step(&pl, HIL_STEP_S);
        work += pl.out.p_shaft_net_w * HIL_STEP_S;
    }
    double ke1 = 0.5 * pl.par.j_kgm2 * pl.st.omega * pl.st.omega;
    double err = fabs((ke1 - ke0) - work) / fabs(ke1 - ke0);
    CHECK(err < 0.02, "delta KE = %.2f MJ, integral of net power = %.2f MJ (error %.2f %%)",
          (ke1 - ke0) / 1e6, work / 1e6, err * 100);
}

static void test_coastdown(void)
{
    printf("Coast-down\n");
    Plant pl; steady_at_rated(&pl, 0.0);
    pl.st.flame = 0; pl.st.wf = 0; pl.st.valve = 0; pl.in.fuel_cmd = 0;
    double n_prev = pl.out.n_pu; int monotone = 1, neg = 0;
    for (int i = 0; i < 60000; i++) {                   /* 600 s */
        plant_step(&pl, HIL_STEP_S);
        if (pl.out.n_pu > n_prev + 1e-9) monotone = 0;
        if (pl.st.omega < 0.0) neg = 1;
        n_prev = pl.out.n_pu;
    }
    CHECK(monotone, "speed falls monotonically with no fuel");
    CHECK(!neg, "speed never negative");
    CHECK(pl.out.n_pu < 0.20, "speed after 600 s is %.3f pu", pl.out.n_pu);
    CHECK(pl.out.n_pu > 0.0,  "speed fades toward zero without hitting it in finite time");
}

static void test_starter(void)
{
    printf("Starter\n");
    PlantParams p; plant_params_default(&p);
    Plant pl; plant_init(&pl, &p); plant_seed(&pl, 1, 0);
    pl.in.starter_on = 1;
    for (int i = 0; i < 6000; i++) plant_step(&pl, HIL_STEP_S);    /* 60 s */
    CHECK(pl.out.n_pu > HIL_N_IGNITION_PU && pl.out.n_pu < 0.5,
          "starter alone reaches %.2f pu (must pass light-off speed %.2f, cannot self-sustain)",
          pl.out.n_pu, HIL_N_IGNITION_PU);
}

static void test_flame_logic(void)
{
    printf("Flame logic\n");
    PlantParams p; plant_params_default(&p);

    /* lean blow-out */
    Plant a; plant_init(&a, &p); plant_seed(&a, 1, 0);
    a.st.omega = a.par.omega_rated; a.st.flame = 1;
    a.st.wf = 0.10; a.st.valve = 0.10 / a.par.wf_max; a.in.fuel_cmd = a.st.valve;
    plant_step(&a, HIL_STEP_S);
    CHECK(a.st.flame == 0, "flame blows out at fuel/air %.4f", 0.10 / a.par.ma_rated);

    /* light-off at speed with ignition and fuel */
    Plant b; plant_init(&b, &p); plant_seed(&b, 1, 0);
    b.st.omega = 0.2 * b.par.omega_rated;
    b.st.wf = 0.09; b.st.valve = 0.09 / b.par.wf_max; b.in.fuel_cmd = b.st.valve;
    b.in.ignition_on = 1;
    plant_step(&b, HIL_STEP_S);
    CHECK(b.st.flame == 1, "flame lights at 0.20 pu with ignition and fuel");

    /* no light below minimum speed */
    Plant c; plant_init(&c, &p); plant_seed(&c, 1, 0);
    c.st.omega = 0.05 * c.par.omega_rated;
    c.st.wf = 0.05; c.st.valve = 0.05 / c.par.wf_max; c.in.fuel_cmd = c.st.valve;
    c.in.ignition_on = 1;
    plant_step(&c, HIL_STEP_S);
    CHECK(c.st.flame == 0, "no light-off below %.2f pu", c.par.n_light_min_pu);

    /* no light without ignition */
    Plant d; plant_init(&d, &p); plant_seed(&d, 1, 0);
    d.st.omega = 0.2 * d.par.omega_rated;
    d.st.wf = 0.09; d.st.valve = 0.09 / d.par.wf_max; d.in.fuel_cmd = d.st.valve;
    plant_step(&d, HIL_STEP_S);
    CHECK(d.st.flame == 0, "no light-off without ignition");
}

static void test_determinism(void)
{
    printf("Determinism\n");
    Sim *a = sim_create(), *b = sim_create();
    sim_reset(a, 42, 1); sim_reset(b, 42, 1);
    sim_set_start(a, 1); sim_set_start(b, 1);
    for (int i = 0; i < 3000; i++) { sim_step(a); sim_step(b); }
    SimOut oa, ob; sim_get(a, &oa); sim_get(b, &ob);
    CHECK(oa.speed_rpm == ob.speed_rpm && oa.tc1_c == ob.tc1_c,
          "same seed gives identical results (%.4f RPM)", oa.speed_rpm);
    sim_reset(b, 43, 1); sim_set_start(b, 1);
    for (int i = 0; i < 3000; i++) sim_step(b);
    sim_get(b, &ob);
    CHECK(oa.speed1_rpm != ob.speed1_rpm || oa.tc1_c != ob.tc1_c,
          "different seed gives different sensor noise");
    sim_destroy(a); sim_destroy(b);
}

static void test_closed_loop_smoke(void)
{
    printf("Closed-loop smoke test (start-up)\n");
    Sim *s = sim_create();
    sim_reset(s, 7, 1);
    sim_set_start(s, 1);
    SimOut o; double t_at_speed = -1, max_speed = 0, max_t = 0; int tripped = 0;
    for (int i = 0; i < 12000; i++) {                   /* 120 s */
        sim_step(s); sim_get(s, &o);
        if (o.speed_rpm > max_speed) max_speed = o.speed_rpm;
        if (o.exh_gas_c > max_t) max_t = o.exh_gas_c;
        if (o.tripped) tripped = 1;
        if (t_at_speed < 0 && fabs(o.speed_rpm - HIL_RATED_SPEED_RPM) < 50.0) t_at_speed = o.time_s;
    }
    CHECK(!tripped, "no trip during start-up");
    CHECK(t_at_speed > 0, "reaches rated speed band at t = %.1f s", t_at_speed);
    CHECK(max_speed < HIL_OVERSPEED_ALARM_RPM, "peak speed %.0f RPM below alarm %.0f", max_speed, HIL_OVERSPEED_ALARM_RPM);
    CHECK(max_t < HIL_EXH_TEMP_ALARM_C, "peak exhaust gas temperature %.0f degC below alarm", max_t);
    sim_destroy(s);
}

int main(void)
{
    printf("=== gt-hil plant unit tests ===\n\n");
    test_design_point();       printf("\n");
    test_no_load_fuel();       printf("\n");
    test_equilibrium();        printf("\n");
    test_rotor_equation();     printf("\n");
    test_energy_conservation();printf("\n");
    test_coastdown();          printf("\n");
    test_starter();            printf("\n");
    test_flame_logic();        printf("\n");
    test_determinism();        printf("\n");
    test_closed_loop_smoke();  printf("\n");
    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
