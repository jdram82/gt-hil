/*
 * hil_regs.h - Modbus register map, version 1.0 (frozen at end of week 2).
 *
 * Plant -> controller : Modbus INPUT registers   (read by the STM32 master)
 * Controller -> plant : Modbus HOLDING registers (written by the STM32 master)
 *
 * Every analog value is a 16-bit unsigned integer = engineering value / scale.
 * Both boards include this one file, so the map cannot drift apart.
 */
#ifndef HIL_REGS_H
#define HIL_REGS_H

#include <stdint.h>

/* ---- Input registers: plant to controller ---- */
enum hil_input_reg {
    HIL_IR_SPEED_RPM   = 0,   /* scale 1 RPM        range 0..10000 */
    HIL_IR_EXH_TEMP    = 1,   /* scale 0.1 degC     range 0..600   (mean of TCs) */
    HIL_IR_PRESSURE    = 2,   /* scale 0.1 bar(g)   range 0..20    */
    HIL_IR_VIBRATION   = 3,   /* scale 0.01 mm/s    (week 2)       */
    HIL_IR_POWER       = 4,   /* scale 0.1 MW       range 0..500   */
    HIL_IR_FUEL_FLOW   = 5,   /* scale 0.01 kg/s    */
    HIL_IR_STATUS      = 6,   /* bit field, see hil_status_bits    */
    HIL_IR_FAULT_CODE  = 7,   /* enum, see hil_fault_code          */
    HIL_IR_TC1         = 8,   /* scale 0.1 degC (week 2)           */
    HIL_IR_TC2         = 9,
    HIL_IR_TC3         = 10,
    HIL_IR_TC4         = 11,
    HIL_IR_DISCRETE    = 12,  /* bit field, see hil_discrete_bits  */
    HIL_IR_COUNT       = 13
};

/* ---- Holding registers: controller to plant ---- */
enum hil_holding_reg {
    HIL_HR_FUEL_CMD    = 0,   /* scale 0.1 %  (0..1000 = 0..100 % of valve)  */
    HIL_HR_START_CMD   = 1,   /* 0 stop, 1 start (starter engaged by plant)  */
    HIL_HR_FAULT_RESET = 2,   /* write 1 to reset                            */
    HIL_HR_MODE        = 3,   /* 0 manual, 1 auto                            */
    HIL_HR_LOAD_SP     = 4,   /* scale 0.1 MW                                */
    HIL_HR_STARTER_CMD = 5,   /* 0 off, 1 on (added: starter is commanded    */
                              /* by the controller, not by the plant)        */
    HIL_HR_COUNT       = 6
};

enum hil_status_bits {
    HIL_ST_RUNNING   = 1u << 0,
    HIL_ST_READY     = 1u << 1,
    HIL_ST_FAULT     = 1u << 2,
    HIL_ST_WARNING   = 1u << 3,
    HIL_ST_AT_SPEED  = 1u << 4,
    HIL_ST_OVERSPEED = 1u << 5,
    HIL_ST_OVERTEMP  = 1u << 6,
    HIL_ST_LOW_FUEL  = 1u << 7
};

enum hil_discrete_bits {
    HIL_DI_BREAKER_CLOSED = 1u << 0,
    HIL_DI_ESTOP_HEALTHY  = 1u << 1,
    HIL_DI_FLAME          = 1u << 2,
    HIL_DI_PERMISSIVE_OK  = 1u << 3
};

enum hil_fault_code {
    HIL_FC_NONE        = 0,
    HIL_FC_OVERSPEED   = 1,
    HIL_FC_OVERTEMP    = 2,
    HIL_FC_VIBRATION   = 3,
    HIL_FC_LOW_OIL     = 4,
    HIL_FC_ESTOP       = 5
};

/* ---- Scaling helpers ---- */
#define HIL_SCALE_TEMP   10.0    /* register = degC * 10 */
#define HIL_SCALE_PRESS  10.0
#define HIL_SCALE_POWER  10.0
#define HIL_SCALE_FUEL   100.0
#define HIL_SCALE_FSR    1000.0  /* fraction 0..1 -> 0..1000 */

static inline uint16_t hil_clamp_u16(double v)
{
    if (v < 0.0)     return 0;
    if (v > 65535.0) return 65535;
    return (uint16_t)(v + 0.5);
}

#endif /* HIL_REGS_H */
