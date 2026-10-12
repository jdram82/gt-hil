/*
 * sim.h - plant + controller glued together for the software-only tier.
 *
 * The controller runs every HIL_CONTROL_PERIOD_S (50 ms), the plant every
 * HIL_STEP_S (10 ms).  The controller's commands are held between updates,
 * the same way a Modbus poll would hold them in a register.
 *
 * SimOut is a flat array of doubles so Python can read it through ctypes
 * without any alignment surprises.
 */
#ifndef SIM_H
#define SIM_H

#include "plant.h"
#include "../controller/standin_governor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Sim Sim;

typedef struct {
    double time_s;
    double speed_rpm;        /* true */
    double speed_meas_rpm;   /* as the controller saw it */
    double n_pu;
    double exh_gas_c;        /* true gas temperature */
    double exh_tc_c;         /* thermocouple (lagged) */
    double exh_meas_c;       /* thermocouple + noise */
    double tit_k;
    double press_bar_g;
    double power_mw;         /* electrical output */
    double load_mw;          /* demand */
    double fuel_kg_s;
    double valve_pos;
    double fsr_cmd;
    double fsr_gov;
    double fsr_acc;
    double fsr_temp;
    double fsr_start;
    double selected;         /* Selected enum */
    double ctrl_state;       /* CtrlState enum */
    double tripped;          /* 1 when in CS_TRIP */
    double trip_cause;       /* TripCause enum */
    double flame;
    double starter_on;
    double breaker;
    double ke_mj;
    double p_gas_net_mw;
    double p_comp_mw;
    double p_turb_mw;
    double p_shaft_net_mw;
} SimOut;

Sim  *sim_create(void);
void  sim_destroy(Sim *s);
void  sim_reset(Sim *s, unsigned long long seed, int noise_enabled);
void  sim_set_start(Sim *s, int start);
void  sim_set_load(Sim *s, double load_mw);
void  sim_set_breaker(Sim *s, int closed);
void  sim_step(Sim *s);                 /* advance one plant step */
void  sim_get(const Sim *s, SimOut *out);
double sim_rated_power_mw(const Sim *s);
int    sim_out_doubles(void);           /* sizeof(SimOut)/8, lets Python self-check */

/* Tuning access for tests / scenarios */
StandinCtrl *sim_ctrl(Sim *s);
Plant       *sim_plant(Sim *s);

#ifdef __cplusplus
}
#endif
#endif
