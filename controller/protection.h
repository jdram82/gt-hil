/*
 * protection.h - trip-condition evaluator.
 *
 * Runs on the voted values, BEFORE the fuel arbitration, and returns a mask
 * with bit n set when the condition for hil_fault_code n is present.  The
 * controller decides whether a present condition causes a trip (state) and
 * latches it.  Alarms are returned as a separate word.
 */
#ifndef PROTECTION_H
#define PROTECTION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Feature switches. All on in normal operation; the negative tests switch
 * one rule off to prove the scenario really depends on it. */
enum {
    FEAT_VOTING          = 1u << 0,
    FEAT_SENSOR_DIAG     = 1u << 1,
    FEAT_GOVERNOR        = 1u << 2,
    FEAT_ACCEL_LIM       = 1u << 3,
    FEAT_TEMP_LIM        = 1u << 4,
    FEAT_BREAKER_PRESET  = 1u << 5,
    FEAT_OVERSPEED_TRIP  = 1u << 6,
    FEAT_OVERTEMP_TRIP   = 1u << 7,
    FEAT_SPREAD_TRIP     = 1u << 8,
    FEAT_COMMS_WD        = 1u << 9,   /* controller side of the watchdog */
    FEAT_PLANT_WD        = 1u << 10,  /* plant side (used by sim.c)      */
    FEAT_ALL             = (1u << 11) - 1u
};

typedef struct {
    double speed_rpm;         /* voted */
    double t_exh_c;           /* voted (median of 4) */
    double spread_c;
    int    speed_fault;       /* SpeedVote.fault */
    int    estop_ok, lube_ok, fuel_ok;
} ProtInput;

typedef struct {
    double spread_t;          /* debounce timer */
} ProtState;

void     prot_init(ProtState *p);
unsigned prot_conditions(ProtState *p, const ProtInput *in, unsigned features,
                         double dt, unsigned *alarms);

#ifdef __cplusplus
}
#endif
#endif
