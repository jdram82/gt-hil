/*
 * sim.h - plant + controller glued together for the software-only tier.
 *
 * The controller runs every HIL_CONTROL_PERIOD_S (50 ms), the plant every
 * HIL_STEP_S (10 ms).  Each controller cycle is one "Modbus poll":
 *   link up   : plant measurements go to the controller, its commands come
 *               back and are applied (and the plant-side watchdog is fed);
 *   link down : the controller sees a failed poll, nothing is applied, and
 *               the plant-side watchdog runs down (HIL_PLANT_WD_S).
 *
 * The breaker is the plant's, not the controller's: the operator REQUESTS
 * it closed and it closes only while the controller's SYNC permit is set.
 * A trip or a plant watchdog action opens it and cancels the request.
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
    double speed1_rpm, speed2_rpm, speed3_rpm;   /* as measured (with faults) */
    double speed_voted_rpm;  /* what the controller used */
    double n_pu;
    double exh_gas_c;        /* true gas temperature */
    double exh_tc_c;         /* thermocouple (lagged), no offset/noise */
    double tc1_c, tc2_c, tc3_c, tc4_c;           /* as measured */
    double exh_voted_c;      /* median of 4 */
    double exh_spread_c;     /* max - min */
    double tit_k;
    double press_bar_g;
    double power_mw;
    double load_mw;          /* demand while the breaker is closed */
    double fuel_kg_s;
    double valve_pos;
    double fsr_cmd;
    double fsr_gov, fsr_acc, fsr_temp, fsr_start;
    double selected;         /* Selected enum */
    double ctrl_state;       /* CtrlState enum */
    double tripped;          /* 1 when latched in CS_TRIP */
    double trip_cause;       /* first-out, hil_fault_code */
    double alarm_word;
    double perm_word;        /* start permissives PM_* */
    double sensor_bad_word;  /* bit i = speed channel i flagged bad */
    double flame;
    double starter_on;
    double breaker;
    double sync_permit;
    double link_up;
    double plant_wd;         /* 1 while the plant-side watchdog is acting */
    double missed_polls;
    double ke_mj;
    double p_gas_net_mw, p_comp_mw, p_turb_mw, p_shaft_net_mw;
} SimOut;

Sim  *sim_create(void);
void  sim_destroy(Sim *s);
void  sim_reset(Sim *s, unsigned long long seed, int noise_enabled);
void  sim_step(Sim *s);                 /* advance one plant step */
void  sim_get(const Sim *s, SimOut *out);
double sim_rated_power_mw(const Sim *s);
int    sim_out_doubles(void);           /* sizeof(SimOut)/8, lets Python self-check */

/* Operator / supervisor inputs */
void  sim_set_start(Sim *s, int start);
void  sim_set_reset(Sim *s, int reset);          /* level; the controller edge-detects */
void  sim_set_droop(Sim *s, int droop);
void  sim_set_load(Sim *s, double load_mw);
void  sim_set_breaker(Sim *s, int closed);       /* request, gated by SYNC permit */

/* Test-supervisor inputs: these act on the plant side only */
void  sim_set_link(Sim *s, int up);
void  sim_set_permissive(Sim *s, int which, int ok);   /* 0 estop 1 lube 2 fuel 3 starter */
void  sim_inject_speed_fault(Sim *s, int ch, int mode, double a, double b);
void  sim_inject_tc_fault(Sim *s, int tc, int mode, double a, double b);
void  sim_clear_faults(Sim *s);
void  sim_set_temp_tuning(Sim *s, double kp, double ki, double headroom);
void  sim_set_ambient_k(Sim *s, double t_amb_k);   /* inlet temperature (hot-day / heat-wave test) */
void  sim_set_features(Sim *s, unsigned mask);   /* FEAT_* , FEAT_ALL = normal */

/* Tuning access for tests */
StandinCtrl *sim_ctrl(Sim *s);
Plant       *sim_plant(Sim *s);

#ifdef __cplusplus
}
#endif
#endif
