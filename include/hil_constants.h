/*
 * hil_constants.h - single source of truth for limits and rated values.
 *
 * Shared by the plant model, the controller and the test tools so that a
 * limit is never defined in two places.  Change a number here and every
 * component sees it.
 */
#ifndef HIL_CONSTANTS_H
#define HIL_CONSTANTS_H

/* ---- Rated operating point (notional machine, see docs/physics.md) ---- */
#define HIL_RATED_SPEED_RPM        3500.0   /* 100 % speed                      */
#define HIL_STEP_S                 0.010    /* plant integration step, 10 ms    */
#define HIL_CONTROL_PERIOD_S       0.050    /* controller / Modbus poll, 50 ms  */

/* ---- Protection and acceptance limits ---- */
#define HIL_OVERSPEED_ALARM_RPM    3780.0   /* 108 % */
#define HIL_OVERSPEED_TRIP_RPM     3850.0   /* 110 % */
#define HIL_EXH_TEMP_CONTROL_C     600.0    /* temperature limiter reference    */
#define HIL_EXH_TEMP_ALARM_C       610.0
#define HIL_EXH_TEMP_TRIP_C        620.0

/* ---- Start-up sequence thresholds (per-unit speed) ---- */
#define HIL_N_IGNITION_PU          0.18     /* light-off speed                  */
#define HIL_N_STARTER_CUTOUT_PU    0.60     /* starter drops out                */
#define HIL_N_AT_SPEED_PU          0.98     /* declared "at speed"              */

#endif /* HIL_CONSTANTS_H */
