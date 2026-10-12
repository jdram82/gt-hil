#include "protection.h"
#include "../include/hil_constants.h"
#include "../include/hil_regs.h"
#include <string.h>

#define BIT(code) (1u << (code))

void prot_init(ProtState *p) { memset(p, 0, sizeof *p); }

unsigned prot_conditions(ProtState *p, const ProtInput *in, unsigned f,
                         double dt, unsigned *alarms)
{
    unsigned cond = 0, al = 0;

    if (in->speed_rpm >= HIL_OVERSPEED_ALARM_RPM) al |= HIL_AL_OVERSPEED;
    if (in->t_exh_c   >= HIL_EXH_TEMP_ALARM_C)    al |= HIL_AL_OVERTEMP;
    if (in->spread_c  >= HIL_SPREAD_ALARM_C)      al |= HIL_AL_SPREAD;

    if (!in->estop_ok) cond |= BIT(HIL_FC_ESTOP);
    if (!in->lube_ok)  cond |= BIT(HIL_FC_LOW_OIL);
    if (!in->fuel_ok)  cond |= BIT(HIL_FC_FUEL_SUPPLY);

    if ((f & FEAT_OVERSPEED_TRIP) && in->speed_rpm >= HIL_OVERSPEED_TRIP_RPM)
        cond |= BIT(HIL_FC_OVERSPEED);
    if ((f & FEAT_OVERTEMP_TRIP) && in->t_exh_c >= HIL_EXH_TEMP_TRIP_C)
        cond |= BIT(HIL_FC_OVERTEMP);

    p->spread_t = (in->spread_c >= HIL_SPREAD_TRIP_C) ? p->spread_t + dt : 0.0;
    if ((f & FEAT_SPREAD_TRIP) && p->spread_t >= HIL_SPREAD_TRIP_DELAY_S)
        cond |= BIT(HIL_FC_EXH_SPREAD);

    if ((f & FEAT_SENSOR_DIAG) && in->speed_fault)
        cond |= BIT(HIL_FC_SENSOR_FAULT);

    if (alarms) *alarms = al;
    return cond;
}
