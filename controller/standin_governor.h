/*
 * standin_governor.h - the gas-turbine controller (reference implementation).
 *
 * Week 2 version: full control philosophy.  Week 3 compiles this same code
 * for the STM32; only the I/O (Modbus master) is new.
 *
 *      FSR = min( FSR_start, FSR_accel, FSR_temp, FSR_governor )
 *
 * Rules implemented (see docs in the Control Philosophy document):
 *   - protection is evaluated first, upstream of the fuel arbitration;
 *   - trips are LATCHED, record a first-out cause, and clear only on an
 *     explicit reset (rising edge) when the cause has gone;
 *   - a start needs a rising edge on START and every start permissive;
 *   - after the warm-up hold the controller issues a SYNC permit; the
 *     breaker may be closed only with that permit;
 *   - 3 speed channels (median vote + diagnostics), 4 exhaust TCs (median);
 *   - controller-side comms watchdog: alarm after 3 failed polls, trip after 10;
 *   - isochronous or 4 % droop governor.
 *
 * Pure C, no hardware calls.
 */
#ifndef STANDIN_GOVERNOR_H
#define STANDIN_GOVERNOR_H

#include "sensor_vote.h"
#include "protection.h"
#include "../include/hil_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CS_IDLE = 0,
    CS_CRANK,
    CS_IGNITE,
    CS_ACCEL,
    CS_WARMUP,
    CS_RUN,
    CS_COOLDOWN,
    CS_TRIP
} CtrlState;

typedef enum {
    SEL_START = 0,
    SEL_ACCEL,
    SEL_TEMP,
    SEL_GOV,
    SEL_FLOOR
} Selected;

/* Start-permissive bits (one per condition; shown for diagnosis) */
enum {
    PM_ESTOP     = 1u << 0,
    PM_LUBE      = 1u << 1,
    PM_FUEL      = 1u << 2,
    PM_STARTER   = 1u << 3,
    PM_SENSORS   = 1u << 4,
    PM_COMMS     = 1u << 5,
    PM_STANDSTILL= 1u << 6,
    PM_NO_TRIP   = 1u << 7,
    PM_ALL       = 0xFFu
};

/* One poll's worth of data from the plant plus the operator commands. */
typedef struct {
    double speed_rpm[3];
    double tc_c[4];
    int    flame, breaker_closed;
    int    estop_ok, lube_ok, fuel_ok, starter_ok;
    int    link_ok;            /* 1 = this poll was answered              */
    int    start_cmd, reset_cmd, droop_mode;
} CtrlInput;

typedef struct {
    /* ---- tuning ---- */
    double n_ref_pu;
    double kp_gov, ki_gov;
    double accel_ref_pu_s;
    double kp_acc, ki_acc;
    double t_ref_c;
    double kp_t, ki_t;
    double temp_headroom;     /* limiter integrator parks this far above the selected FSR */
    double fsr_light, fsr_start_rate;
    double fsr_floor_start, fsr_floor_run, fsr_noload;
    double purge_s, light_timeout_s, track_tau_s, accel_filter_s;
    double warmup_s, droop_pu;
    unsigned features;         /* FEAT_* mask */

    /* ---- state ---- */
    CtrlState state;
    double t_state;
    double i_gov, i_acc, i_temp;
    double fsr_start;
    double n_prev, accel_f;
    int    breaker_prev, reset_prev, start_lockout, missed_polls;
    unsigned alarm_base;
    SpeedVote sv;
    ProtState prot;

    /* ---- outputs / diagnostics ---- */
    double fsr_out;
    int    starter_on, ignition_on, sync_permit, trip_out;
    int    first_out;          /* latched first-out cause (hil_fault_code) */
    unsigned alarm_word, perm_word, cond_mask;
    double speed_voted_rpm, t_voted_c, t_spread_c;
    double fsr_gov, fsr_acc, fsr_temp;
    Selected selected;
} StandinCtrl;

#define TRIP_NONE HIL_FC_NONE
#define ctrl_tripped(c) ((c)->state == CS_TRIP)

void ctrl_init(StandinCtrl *c);
void ctrl_update(StandinCtrl *c, double dt, const CtrlInput *in);
/* Start-permissive mask for the current inputs (PM_* bits set = satisfied). */
unsigned ctrl_permissives(const StandinCtrl *c, const CtrlInput *in);

#ifdef __cplusplus
}
#endif
#endif
