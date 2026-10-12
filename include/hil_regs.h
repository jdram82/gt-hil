/*
 * hil_regs.h - Modbus register map, VERSION 1.0 (frozen at end of week 2).
 *
 * Modbus RTU, 115200 8E1, BeagleBone Black = slave (address 1),
 * STM32 = master, poll every 50 ms.
 *
 * Input registers  (function 04): plant -> controller, read-only for the master.
 * Holding registers (function 03/16): two blocks, see the "written by" notes.
 *
 * Every analog value is a 16-bit unsigned integer = engineering value * scale.
 * Both boards include this one file, so the map cannot drift apart.
 * Any change after this point must bump HIL_REGMAP_VERSION.
 */
#ifndef HIL_REGS_H
#define HIL_REGS_H

#include <stdint.h>

#define HIL_REGMAP_VERSION_MAJOR 1
#define HIL_REGMAP_VERSION_MINOR 0

/* ---------------- Input registers: plant -> controller ---------------- */
enum hil_input_reg {
    HIL_IR_SPEED1_RPM  = 0,   /* scale 1 RPM       three independent channels */
    HIL_IR_SPEED2_RPM  = 1,
    HIL_IR_SPEED3_RPM  = 2,
    HIL_IR_TC1         = 3,   /* scale 0.1 degC    four exhaust thermocouples */
    HIL_IR_TC2         = 4,
    HIL_IR_TC3         = 5,
    HIL_IR_TC4         = 6,
    HIL_IR_PRESSURE    = 7,   /* scale 0.1 bar(g)                              */
    HIL_IR_POWER       = 8,   /* scale 0.1 MW                                  */
    HIL_IR_FUEL_FLOW   = 9,   /* scale 0.01 kg/s                               */
    HIL_IR_DISCRETE    = 10,  /* bit field, hil_discrete_bits                  */
    HIL_IR_PLANT_HB    = 11,  /* plant heartbeat, +1 per answered poll (wraps) */
    HIL_IR_COUNT       = 12
};

/* ---------------- Holding registers ---------------- */
enum hil_holding_reg {
    /* written by the operator side (BeagleBone supervisor), read by the STM32 */
    HIL_HR_START_CMD   = 0,   /* 1 = run, 0 = stop; must go low->high to start */
    HIL_HR_RESET_CMD   = 1,   /* rising edge = reset latched trip              */
    HIL_HR_MODE        = 2,   /* 0 isochronous, 1 droop (4 %)                  */
    /* written by the STM32 master, read by the plant                          */
    HIL_HR_FUEL_CMD    = 3,   /* scale 0.1 %  (0..1000 = 0..100 % valve)       */
    HIL_HR_DO_WORD     = 4,   /* bit field, hil_do_bits                        */
    HIL_HR_CTRL_HB     = 5,   /* controller heartbeat, +1 per cycle (wraps)    */
    HIL_HR_CTRL_STATE  = 6,   /* enum CtrlState (display)                      */
    HIL_HR_FIRST_OUT   = 7,   /* enum hil_fault_code, 0 = none (latched)       */
    HIL_HR_ALARM_WORD  = 8,   /* bit field, hil_alarm_bits                     */
    HIL_HR_COUNT       = 9
};

/* Individual permissive / status inputs. One bit each: never a lumped OK. */
enum hil_discrete_bits {
    HIL_DI_BREAKER_CLOSED = 1u << 0,
    HIL_DI_ESTOP_HEALTHY  = 1u << 1,
    HIL_DI_FLAME          = 1u << 2,
    HIL_DI_LUBE_OK        = 1u << 3,
    HIL_DI_FUEL_OK        = 1u << 4,
    HIL_DI_STARTER_OK     = 1u << 5,
    HIL_DI_PLANT_WD       = 1u << 6   /* plant-side watchdog has acted         */
};

/* Controller outputs (energised = action; de-energise-to-trip philosophy:
 * DO_TRIP_OK is held 1 while healthy and drops to 0 on trip). */
enum hil_do_bits {
    HIL_DO_STARTER     = 1u << 0,
    HIL_DO_IGNITION    = 1u << 1,
    HIL_DO_SYNC_PERMIT = 1u << 2,   /* breaker may be closed                   */
    HIL_DO_TRIP_OK     = 1u << 3    /* 1 = healthy; 0 = tripped, breaker opens */
};

enum hil_alarm_bits {
    HIL_AL_OVERSPEED   = 1u << 0,
    HIL_AL_OVERTEMP    = 1u << 1,
    HIL_AL_SPREAD      = 1u << 2,
    HIL_AL_SPEED1_BAD  = 1u << 3,
    HIL_AL_SPEED2_BAD  = 1u << 4,
    HIL_AL_SPEED3_BAD  = 1u << 5,
    HIL_AL_COMMS       = 1u << 6,
    HIL_AL_START_BLOCK = 1u << 7    /* start requested but a permissive is out */
};

/* First-out trip cause. Values 1..5 are kept from the draft map. */
enum hil_fault_code {
    HIL_FC_NONE         = 0,
    HIL_FC_OVERSPEED    = 1,
    HIL_FC_OVERTEMP     = 2,
    HIL_FC_VIBRATION    = 3,   /* reserved, not implemented in the simulator   */
    HIL_FC_LOW_OIL      = 4,
    HIL_FC_ESTOP        = 5,
    HIL_FC_FLAMEOUT     = 6,
    HIL_FC_FAIL_TO_LIGHT= 7,
    HIL_FC_EXH_SPREAD   = 8,
    HIL_FC_SENSOR_FAULT = 9,
    HIL_FC_COMMS_LOSS   = 10,
    HIL_FC_FUEL_SUPPLY  = 11,
    HIL_FC_COUNT        = 12
};

/* ---------------- Scaling ---------------- */
#define HIL_SCALE_SPEED  1.0
#define HIL_SCALE_TEMP   10.0
#define HIL_SCALE_PRESS  10.0
#define HIL_SCALE_POWER  10.0
#define HIL_SCALE_FUEL   100.0
#define HIL_SCALE_FSR    1000.0   /* fraction 0..1 -> 0..1000 */

static inline uint16_t hil_clamp_u16(double v)
{
    if (v < 0.0)     return 0;
    if (v > 65535.0) return 65535;
    return (uint16_t)(v + 0.5);
}
static inline uint16_t hil_enc(double eng, double scale) { return hil_clamp_u16(eng * scale); }
static inline double   hil_dec(uint16_t raw, double scale) { return (double)raw / scale; }

#endif /* HIL_REGS_H */
