#include "amk.h"
#include "source/main_module/can/can_parse.h"
#include "common/faults/faults.h"

/* NOTE:
 * Step 1 is turning on LV
 * Step 3 happens when HV is turned on and precharging starts (I think this is actually step 2)
 *  I can check when this is done with precharge complete and then move 
 *  onto other steps
 *  MAYBE we can check when AMK_bSystemReady is on and display a message
 *  on LCD so they know when to turn on the HV to start the precharging.
 *  Not sure if I need that
 */

/* NOTE: As of now this is just setting everything to 0, but it may make sense
 * to have it in case something changes down the line while I learn more, so
 * this may end up being deleted if everything just inits to 0 */
void amkInit(amk_motor_t* motor, bool* pchg_complete, void (*sendSetpoints)())
{
    *motor = (amk_motor_t){
        /* States */
        .states.main_state = AMK_STATE_OFF,
        .states.init_state = AMK_INIT_POWER_ON,
        .states.deinit_state = AMK_DEINIT_DISABLE,
        .states.running_state = AMK_RUNNING_GOOD,
        .states.reset_state = AMK_RESET_INVERTER_OFF,

        /* Values */
        .torque_setpoint = DEFAULT_TORQUE_SETPOINT,
        .torque_limit_positive = DEFAULT_POSITIVE_TORQUE_LIMIT,
        .torque_limit_negative = DEFAULT_NEGATIVE_TORQUE_LIMIT,

        .pchg_complete = pchg_complete,

        .sendSetpoints = sendSetpoints
    };
}

void amkPeriodic(amk_motor_t* motor)
{
    amkGetData(motor);

    switch(motor->states.main_state) {
    case AMK_STATE_OFF:
        if (*motor->pchg_complete) {
            /* FIXME: Also need to check READY2DRIVE */
            motor->states.main_state = AMK_STATE_INIT;
        }
        break;
    case AMK_STATE_INIT:
        turnAmkOn(motor);
        break;
    case AMK_STATE_RUNNING:
        if (motor->status.AMK_bError) {
            setFault(motor->error_fault_id, true);
            resetAmk(motor);
        }
        else
        {
            motor->control.AMK_bInverterOn = true;
            motor->control.AMK_bEnable = true;
            motor->torque_setpoint = 100;
            motor->torque_limit_positive = 100;
            motor->torque_limit_negative = -1;
        }

        break;
    case AMK_STATE_DEINIT:
        turnAmkOff(motor);
        break;
    }

    SEND_INVA_SET(motor->control.AMK_bReserve1, motor->control.AMK_bInverterOn, motor->control.AMK_bDcOn,
        motor->control.AMK_bEnable, motor->control.AMK_bErrorReset,
        motor->control.AMK_bReserve2, motor->torque_setpoint, motor->torque_limit_positive, motor->torque_limit_negative);

    // motor->sendSetpoints();
}

/* Sets the torque setpoint from -1000% to 1000% of nominal torque.
 * But this function just takes in -100% to 100% to stay consistent */
void amkSetTorque(amk_motor_t* motor, int16_t torque_setpoint)
{
    if (torque_setpoint > MAX_POSITIVE_TORQUE_SETPOINT 
        || torque_setpoint < MAX_NEGATIVE_TORQUE_SETPOINT) {
        return;
    }

    /* Scale up since unit is 0.1% of nominal torque */
    torque_setpoint *= 10;

    motor->torque_setpoint = torque_setpoint;

    motor->torque_limit_positive = 1000;

    /* NOTE: For some reason it cannot be 0, so do -0.1% (according to UIUC's team) */
    motor->torque_limit_negative = -1;
}

static void amkGetData(amk_motor_t* motor)
{
    if (!can_data.INVA_CRIT.stale) {
        motor->actual_speed = can_data.INVA_CRIT.AMK_ActualSpeed;
        motor->actual_torque = can_data.INVA_CRIT.AMK_ActualTorque;
        //TODO OVERLOAD
    }

    if (!can_data.INVA_TEMPS.stale) {
        motor->motor_temp = can_data.INVA_TEMPS.AMK_MotorTemp;
        motor->inverter_temp = can_data.INVA_TEMPS.AMK_InverterTemp;
        motor->igbt_temp = can_data.INVA_TEMPS.AMK_IGBTTemp;
    } else {
        setFault(motor->stale_fault_id, true);
    }

    if (!can_data.INVA_INFO.stale) {
        motor->status.AMK_bSystemReady = can_data.INVA_INFO.AMK_Status_bSystemReady;
        motor->status.AMK_bError = can_data.INVA_INFO.AMK_Status_bError;
        motor->status.AMK_bWarn = can_data.INVA_INFO.AMK_Status_bWarn;
        motor->status.AMK_bQuitDcOn = can_data.INVA_INFO.AMK_Status_bQuitDcOn;
        motor->status.AMK_bDcOn = can_data.INVA_INFO.AMK_Status_bDcOn;
        motor->status.AMK_bQuitInverterOn = can_data.INVA_INFO.AMK_Status_bQuitInverterOn;
        motor->status.AMK_bInverterOn = can_data.INVA_INFO.AMK_Status_bInverterOn;
        motor->status.AMK_bDerating = can_data.INVA_INFO.AMK_Status_bDerating;
        motor->dc_bus_voltage = can_data.INVA_INFO.AMK_DCBusVoltage;
    } else {
        setFault(motor->stale_fault_id, true);
    }

    if (!can_data.INVA_ERR_1.stale) {
        motor->diagnostic_num = can_data.INVA_ERR_1.AMK_DiagnosticNumber;
        motor->error_info_1 = can_data.INVA_ERR_1.AMK_ErrorInfo1;
    } else {
        setFault(motor->stale_fault_id, true);
    }

    if (!can_data.INVA_ERR_2.stale) {
        motor->error_info_2 = can_data.INVA_ERR_2.AMK_ErrorInfo2;
        motor->error_info_3 = can_data.INVA_ERR_2.AMK_ErrorInfo3;
    } else {
        setFault(motor->stale_fault_id, true);
    }
}

static void resetAmk(amk_motor_t* motor)
{
    /* State machine here based on 8.2.6 */
    switch (motor->states.reset_state) {
    case AMK_RESET_INVERTER_OFF:
        /* 1. Set AMK_bInverterOn = 0 */
        motor->control.AMK_bInverterOn = false;

        motor->states.reset_state = AMK_RESET_ERROR_RESET_ON;

        break;
    case AMK_RESET_ERROR_RESET_ON:
        motor->control.AMK_bErrorReset = true;

        motor->torque_setpoint = 0;
        motor->torque_limit_positive = 0;
        motor->torque_limit_negative = 0;

        motor->states.reset_state = AMK_RESET_ERROR_RESET_OFF;

        break;
    case AMK_RESET_ERROR_RESET_OFF:
        motor->control.AMK_bErrorReset = false;

        motor->states.reset_state = AMK_RESET_CHECK_SYSTEM_READY;

        break;
    case AMK_RESET_CHECK_SYSTEM_READY:
        if (motor->status.AMK_bSystemReady) {
            motor->states.reset_state = AMK_RESET_INVERTER_OFF;
            motor->states.running_state = AMK_RUNNING_GOOD;
        }

        break;
    }
}

static void turnAmkOn(amk_motor_t* motor)
{
    /*
     * Motor Datasheet:
     * https://www.amk-motion.com/amk-dokucd/dokucd/en/content/resources/pdf-dateien/pdk_205481_kw26-s5-fse-4q_en_.pdf
     *
     * Section 9.4 goes over turning the motors on and off 
     */

    switch (motor->states.init_state) {
    case AMK_INIT_POWER_ON:
        /* Turn on 24V DC to inverters */
        if (motor->status.AMK_bSystemReady) {
            motor->states.init_state = AMK_INIT_ENABLE;
        }

        break;
    case AMK_INIT_ENABLE:
        motor->torque_setpoint = 0;
        motor->torque_limit_positive = 0;
        motor->torque_limit_negative = 0;

        motor->control.AMK_bDcOn = true;

        motor->control.AMK_bEnable = true;

        motor->control.AMK_bInverterOn = true;

        /* FIXME: Do I need to check anything first?? */
        motor->states.main_state = AMK_STATE_RUNNING;
        break;
    }
}

static void turnAmkOff(amk_motor_t* motor)
{
    /*
     * Motor Datasheet:
     * https://www.amk-motion.com/amk-dokucd/dokucd/en/content/resources/pdf-dateien/pdk_205481_kw26-s5-fse-4q_en_.pdf
     *
     * Section 9.4 goes over turning the motors on and off 
     */

    switch(motor->states.deinit_state) {
    case AMK_DEINIT_DISABLE:
        motor->torque_setpoint = 0;
        motor->torque_limit_positive = 0;
        motor->torque_limit_negative = 0;

        motor->control.AMK_bDcOn = true;

        motor->control.AMK_bEnable = true;

        motor->control.AMK_bInverterOn = true;

        motor->states.deinit_state = AMK_DEINIT_POWER_OFF;

        break;
    case AMK_DEINIT_POWER_OFF:
        if (!(*motor->pchg_complete)) {
            motor->states.main_state = AMK_STATE_OFF;
        }
    }
}
