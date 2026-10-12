/*
 * plant.h - single-shaft gas turbine generator set, lumped-parameter model.
 *
 * Physics: Brayton cycle with component efficiencies (compressor, combustor,
 * turbine), a rotor energy balance, first-order fuel actuator and combustor
 * lags, and a first-order thermocouple.  Full derivation in docs/physics.md.
 *
 * Units are SI (K, Pa/bar, W, kg, s) unless a name says otherwise.
 * The model has no hardware calls, so the same file builds on a PC, the
 * BeagleBone Black and, if wanted, the STM32.
 */
#ifndef PLANT_H
#define PLANT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- Parameters ---------------- */
typedef struct {
    /* ambient (ISO standard day) */
    double t_amb_k;          /* 288.15 K                                    */
    double p_amb_bar;        /* 1.01325 bar                                 */

    /* working fluids */
    double cp_air;           /* J/(kg K), compressor side                   */
    double cp_gas;           /* J/(kg K), combustion products               */
    double kappa_air;        /* (gamma-1)/gamma for air  = 0.2857           */
    double kappa_gas;        /* (gamma-1)/gamma for gas  = 0.2481           */
    double lhv;              /* fuel lower heating value, J/kg              */
    double eta_comb;         /* combustion efficiency                       */

    /* thermodynamic design */
    double r_rated;          /* compressor pressure ratio at 100 % speed    */
    double eta_comp;         /* compressor isentropic efficiency            */
    double eta_turb;         /* turbine isentropic efficiency               */
    double dp_frac;          /* total pressure loss fraction (inlet,        */
                             /* combustor, exhaust)                         */
    double ma_rated;         /* air mass flow at 100 % speed, kg/s          */
    double wf_design;        /* fuel flow at design point, kg/s             */
    double wf_max;           /* fuel flow at 100 % valve, kg/s              */
    double far_blowout;      /* lean blowout fuel/air ratio                 */
    double far_light;        /* minimum fuel/air ratio to light off         */
    double n_light_min_pu;   /* minimum speed to light off                  */

    /* mechanical / electrical */
    double rated_rpm;
    double h_inertia_s;      /* inertia constant H = KE_rated / P_design    */
    double p_fric_frac;      /* friction + windage at rated speed / P_design*/
    double eta_gen;          /* generator efficiency                        */
    double load_damping;     /* d(load)/d(speed), per-unit/per-unit         */

    /* dynamics */
    double tau_valve_s;      /* fuel valve actuator lag                     */
    double tau_fuel_s;       /* fuel line / combustor lag                   */
    double tau_tc_s;         /* exhaust thermocouple lag                    */

    /* starter motor */
    double starter_torque_nm;
    double starter_power_w;

    /* sensor noise (uniform, +/- amplitude) */
    double noise_speed_rpm;
    double noise_temp_c;

    /* derived in plant_init() */
    double omega_rated;      /* rad/s                                       */
    double p_gas_design_w;   /* net gas power at design point               */
    double p_design_w;       /* shaft power available to the generator      */
    double p_fric_rated_w;
    double j_kgm2;           /* rotor inertia                               */
} PlantParams;

/* ---------------- Inputs written by the controller / scenario ---------------- */
typedef struct {
    double fuel_cmd;         /* fuel valve demand, 0..1                     */
    int    starter_on;
    int    ignition_on;
    int    breaker_closed;
    double load_elec_mw;     /* electrical load demand at rated speed       */
} PlantInputs;

/* ---------------- State ---------------- */
typedef struct {
    double omega;            /* rotor speed, rad/s                          */
    double valve;            /* actual valve position, 0..1                 */
    double wf;               /* actual fuel flow, kg/s                      */
    double t_tc_k;           /* thermocouple reading, K                     */
    int    flame;            /* 1 when flame is established                 */
    double time;             /* simulated time, s                           */
} PlantState;

/* ---------------- Derived outputs (ground truth) ---------------- */
typedef struct {
    double n_pu;
    double speed_rpm;
    double p_cd_bar_g;       /* compressor discharge pressure, gauge        */
    double t_cd_k;
    double tit_k;            /* turbine inlet temperature                   */
    double t_exh_gas_k;
    double t_exh_gas_c;
    double t_exh_tc_c;
    double ma_kg_s;
    double wf_kg_s;
    double valve_pos;
    double p_comp_w;
    double p_turb_w;
    double p_gas_net_w;
    double p_fric_w;
    double p_load_mech_w;
    double p_load_elec_mw;
    double p_starter_w;
    double p_shaft_net_w;    /* net power into the rotor (drives dKE/dt)    */
    double ke_j;             /* rotor kinetic energy                        */
    int    flame;
} PlantOutputs;

/* ---------------- Measurements (what a sensor would report) ---------------- */
typedef struct {
    double speed_rpm;
    double exh_temp_c;
    double press_bar_g;
    double power_mw;
    double fuel_kg_s;
    int    flame;
    int    breaker_closed;
} PlantMeasurements;

typedef struct {
    PlantParams  par;
    PlantInputs  in;
    PlantState   st;
    PlantOutputs out;
    unsigned long long rng;
    int noise_enabled;
} Plant;

void plant_params_default(PlantParams *p);
void plant_init(Plant *pl, const PlantParams *p);
void plant_step(Plant *pl, double dt);
void plant_measure(Plant *pl, PlantMeasurements *m);
void plant_seed(Plant *pl, unsigned long long seed, int noise_enabled);

/* Steady-state helper (used by unit tests and by initialisation):
 * fuel flow (kg/s) at speed n_pu that gives net shaft power p_shaft_w,
 * assuming flame is established.  Returns negative if unreachable. */
double plant_fuel_for_shaft_power(const PlantParams *p, double n_pu, double p_shaft_w);

#ifdef __cplusplus
}
#endif
#endif /* PLANT_H */
