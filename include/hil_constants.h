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

/* ---- Exhaust thermocouple spread (4 TCs) ---- */
#define HIL_SPREAD_ALARM_C         40.0     /* max - min, alarm                  */
#define HIL_SPREAD_TRIP_C          80.0     /* max - min, trip (after debounce)  */
#define HIL_SPREAD_TRIP_DELAY_S    0.5

/* ---- Speed sensor diagnostics (3 channels, median voting) ---- */
#define HIL_SPEED_DEV_RPM          40.0     /* channel vs median, flag as bad    */
#define HIL_SPEED_DEV_TIME_S       1.0      /* deviation must persist this long  */
#define HIL_SPEED_FLAT_TIME_S      1.0      /* identical readings => flat-lined  */
#define HIL_SPEED_FLAT_MIN_RPM     1000.0   /* flat-line test only when turning  */

/* ---- Start-up sequence additions ---- */
#define HIL_N_STANDSTILL_PU        0.05     /* start permissive / cool-down end  */
#define HIL_WARMUP_S               5.0      /* hold at speed before sync permit  */
#define HIL_SYNC_BAND_PU           0.005    /* +/-0.5 % speed to allow breaker   */

/* ---- Communication watchdog (two-way) ---- */
#define HIL_COMMS_ALARM_POLLS      3        /* failed polls -> alarm (150 ms)    */
#define HIL_COMMS_TRIP_POLLS       10       /* failed polls -> trip  (500 ms)    */
#define HIL_PLANT_WD_S             0.5      /* plant side: no command for this   */
                                            /* long -> fuel closed, breaker open */

/* ---- Governor ---- */
#define HIL_DROOP_PU               0.04     /* 4 % droop when in droop mode      */

#endif /* HIL_CONSTANTS_H */
