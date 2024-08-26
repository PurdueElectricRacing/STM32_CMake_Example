#include "em_pp.h"
#include "em.h"
#include "can_parse.h"
#include "common_defs.h"

void em_pp(ExtU_em *rtU_tm, ExtY_tv *rtY_tv)
{
    /* Throttle Map Input */
    rtU_tm->rTV[0] = rtY_tv->rTVS[0];
    rtU_tm->rTV[1] = rtY_tv->rTVS[1];
    rtU_tm->rEQUAL = (can_data.filt_throttle_brake.throttle/4095.0);

    rtU_tm->D_raw[0] = (can_data.orion_currents_volts.pack_voltage*0.1);
    rtU_tm->D_raw[1] = (can_data.rear_wheel_speeds.left_speed_sensor*8.75*0.01);
    rtU_tm->D_raw[2] = (can_data.rear_wheel_speeds.right_speed_sensor*8.75*0.01);

    rtU_tm->F_raw[0] = (can_data.filt_throttle_brake.stale == 0);
    rtU_tm->F_raw[1] = (can_data.rear_wheel_speeds.stale == 0);
    rtU_tm->F_raw[2] = (can_data.orion_currents_volts.stale == 0); 

    /* Driver Tunable Parameters */
    if (!can_data.dashboard_tv_parameters.stale) {
        rtU_tm->F_raw[3] = (can_data.dashboard_tv_parameters.tv_enabled == 1);
    }
}