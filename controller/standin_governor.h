/*
 * standin_governor.h - stand-in controller for weeks 1-2.
 *
 * It exists so the plant and the scenarios can be proven before the real
 * controller is written (week 3).  It follows the same architecture a
 * turbine control system uses:
 *
 *      FSR = min( FSR_start, FSR_accel, FSR_temp, FSR_governor )
 *
 * FSR is the "fuel stroke reference": the fuel valve demand, 0..1.
 * The lowest of the four requests always wins, so the temperature and
 * acceleration limiters can override the speed governor.
 *
 * Pure C, no hardware calls: the same file compiles for the STM32 later.
 */
#ifndef STANDIN_GOVERNOR_H
#define STANDIN_GOVERNOR_H

#include "../plant/plant.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CS_IDLE = 0,
    CS_CRANK,
    CS_IGNITE,
    CS_ACCEL,
    CS_RUN,
    CS_COOLDOWN,
    CS_TRIP
} CtrlState;

typedef enum {
    TRIP_NONE = 0,
    TRIP_OVERSPEED,
    TRIP_OVERTEMP,
    TRIP_FLAMEOUT,
    TRIP_FAIL_TO_LIGHT
} TripCause;

typedef enum {
    SEL_START = 0,
    SEL_ACCEL,
    SEL_TEMP,
    SEL_GOV,
    SEL_FLOOR
} Selected;

typedef struct {
    /* ---- tuning ---- */
    double n_ref_pu;          /* speed reference, 1.0 = rated               */
    double kp_gov, ki_gov;    /* governor PI (fsr per pu speed)             */
    double accel_ref_pu_s;    /* acceleration limit, pu speed per second    */
    double kp_acc, ki_acc;    /* acceleration PI                            */
    double t_ref_c;           /* exhaust temperature limit reference        */
    double kp_t, ki_t;        /* temperature PI (fsr per K)                 */
    double fsr_light;         /* fuel demand at light-off                   */
    double fsr_start_rate;    /* start ramp, fsr per second                 */
    double fsr_floor_start;   /* minimum fuel during start                  */
    double fsr_floor_run;     /* minimum fuel when running (flame-out guard)*/
    double fsr_noload;        /* fuel preset on breaker opening             */
    int    use_breaker_preset;/* 1 = step fuel on load rejection            */
    double purge_s;           /* minimum crank time before ignition         */
    double light_timeout_s;
    double track_tau_s;       /* anti-windup back-calculation time constant */
    double accel_filter_s;

    /* ---- state ---- */
    CtrlState state;
    double t_state;
    double i_gov, i_acc, i_temp;
    double fsr_start;
    double n_prev, accel_f;
    int    breaker_prev;
    TripCause trip_cause;

    /* ---- outputs / diagnostics ---- */
    double fsr_out;
    int    starter_on;
    int    ignition_on;
    double fsr_gov, fsr_acc, fsr_temp;
    Selected selected;
} StandinCtrl;

void ctrl_init(StandinCtrl *c);
void ctrl_update(StandinCtrl *c, double dt, int start_cmd, const PlantMeasurements *m);

#ifdef __cplusplus
}
#endif
#endif
