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
void amkInit(amk_motor_t* motor, bool* pchg_complete)
{
    *motor = (amk_motor_t){
        /* States */
        .states.state = AMK_STATE_INIT,
        .states.init_state = AMK_INIT_POWER_ON,
        .states.deinit_state = AMK_DEINIT_ZERO_SETPOINTS,
        .states.running_state = AMK_RUNNING_GOOD,
        .states.reset_state = AMK_RESET_INVERTER_OFF,

        /* Values */
        .torque_setpoint = DEFAULT_TORQUE_SETPOINT,
        .torque_limit_positive = DEFAULT_POSITIVE_TORQUE_LIMIT,
        .torque_limit_negative = DEFAULT_NEGATIVE_TORQUE_LIMIT,

        .pchg_complete = pchg_complete
    };
}

void amkPeriodic(amk_motor_t* motor)
{
    amkGetData(motor);

    switch(motor->states.state) {
    case AMK_STATE_OFF:
        if (motor->pchg_complete) {
            /* FIXME: Also need to check READY2DRIVE */
            motor->states.state = AMK_STATE_INIT;
        }
        break;
    case AMK_STATE_INIT:
        turnAmkOn(motor);
        break;
    case AMK_STATE_RUNNING:
        amkRunning(motor);
        break;
    case AMK_STATE_DEINIT:
        turnAmkOff(motor);
        break;
    }

    //motor->sendSetpoints();
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
    // torque_setpoint *= 10;

    motor->torque_setpoint = torque_setpoint;

    if (torque_setpoint < 0) {
        motor->torque_limit_negative = -1000;
        motor->torque_limit_positive = 1;
    } else {
        motor->torque_limit_positive = 1000;

        /* NOTE: For some reason it cannot be 0, so do -0.1% (according to UIUC's team) */
        motor->torque_limit_negative = -1;
    }

}

static void amkGetData(amk_motor_t* motor)
{
    if (!can_data.AMK_Actual_Values_1.stale) {
        // motor->status.AMK_bSystemReady = can_data.AMK_Actual_Values_1.AMK_Status_bSystemReady;
        // motor->status.AMK_bError = can_data.AMK_Actual_Values_1.AMK_Status_bError;
        // motor->status.AMK_bWarn = can_data.AMK_Actual_Values_1.AMK_Status_bWarn;
        // motor->status.AMK_bQuitDcOn = can_data.AMK_Actual_Values_1.AMK_Status_bQuitDcOn;
        // motor->status.AMK_bDcOn = can_data.AMK_Actual_Values_1.AMK_Status_bDcOn;
        // motor->status.AMK_bQuitInverterOn = can_data.AMK_Actual_Values_1.AMK_Status_bQuitInverterOn;
        // motor->status.AMK_bInverterOn = can_data.AMK_Actual_Values_1.AMK_Status_bInverterOn;
        // motor->status.AMK_bDerating = can_data.AMK_Actual_Values_1.AMK_Status_bDerating;
        //
        // motor->actual_torque = can_data.AMK_Actual_Values_1.AMK_ActualTorque;
        // motor->serial_num = can_data.AMK_Actual_Values_1.AMK_MotorSerialNumber;
    } else {
        setFault(motor->stale_fault_id, true);
    }

    // if (!can_data.AMK_Actual_Values_2.stale) {
    //     motor->actual_speed = can_data.AMK_Actual_Values_2.AMK_ActualSpeed;
    //     motor->dc_bus_voltage = can_data.AMK_Actual_Values_2.AMK_DCBusVoltage;
    //     motor->system_reset = can_data.AMK_Actual_Values_2.AMK_SystemReset;
    //     motor->diagnostic_num = can_data.AMK_Actual_Values_2.AMK_DiagnosticNumber;
    // } else {
    //     setFault(motor->stale_fault_id, true);
    // }

    // if (!can_data.AMK_Temperatures_1.stale) {
    //     motor->motor_temp = can_data.AMK_Temperatures_1.AMK_MotorTemp;
    //     motor->inverter_temp = can_data.AMK_Temperatures_1.AMK_InverterTemp;
    //     motor->igbt_temp = can_data.AMK_Temperatures_1.AMK_IGBTTemp;
    // } else {
    //     setFault(motor->stale_fault_id, true);
    // }
}

static void motorSendSetpoints(amk_motor_t* motor)
{
    SEND_AMK_SETPOINTS_1(motor->control.AMK_bReserve1,
                         motor->control.AMK_bInverterOn,
                         motor->control.AMK_bDcOn,
                         motor->control.AMK_bEnable,
                         motor->control.AMK_bErrorReset,
                         motor->control.AMK_bReserve2,
                         motor->torque_setpoint,
                         motor->torque_limit_positive,
                         motor->torque_limit_negative);
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

/* TODO: Not really sure what needs to be done here. We just need to push
 * these values that are determined in car.c state machine (READY2DRIVE state)
 */
static void amkRunning(amk_motor_t* motor)
{
    switch (motor->states.running_state) {
    case AMK_RUNNING_GOOD:
        /* Check for errors */
        if (motor->status.AMK_bError) {
            motor->states.running_state = AMK_RUNNING_ERROR;
        }
        // amkSetTorque(motor, 100);

        break;
    case AMK_RUNNING_ERROR:
        setFault(motor->error_fault_id, true);

        resetAmk(motor);

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
     *
     * Steps with the "r" suffix are requirement steps, the requirement needs to
     * be met before moving onto the next step.
     */

    switch (motor->states.init_state) {
    case AMK_INIT_POWER_ON:
        /* 1. Turn on 24V DC to inverters */
        /* 1r. Check AMK_bSystemReady = 1*/
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
        motor->states.state = AMK_STATE_RUNNING;

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
     *
     * Steps with the "r" suffix are requirement steps, the requirement needs to
     * be met before moving onto the next step.
     */

    /* NOTE: For some reason in the deinit state machine, we have to turn the
     * inverter off, then disable, and then check for the inverter being off.
     * This is different to how it is done in the init state machine but it
     * is correct based on the datasheet.
     */

    switch(motor->states.deinit_state) {
    case AMK_DEINIT_ZERO_SETPOINTS:
        /* 1. Set setpoint settings to 0 (AMK_TargetVelocity, AMK_TorqueLimitNegativ, AMK_TorqueLimitPositiv) */
        motor->torque_setpoint = 0;
        motor->torque_limit_positive = 0;
        motor->torque_limit_negative = 0;

        motor->states.deinit_state = AMK_DEINIT_INVERTER_OFF;

        break;

    case AMK_DEINIT_INVERTER_OFF:
        /* 2. Set AMK_bInverterOn = 0 */
        motor->control.AMK_bInverterOn = false;

        motor->states.deinit_state = AMK_DEINIT_DISABLE;

        break;
    case AMK_DEINIT_DISABLE:
        /* 3. Set AMK_bEnable = 0 */

        motor->control.AMK_bEnable = false;

        motor->states.deinit_state = AMK_DEINIT_QUIT_INVERTER_CHECK;

        break;
    case AMK_DEINIT_QUIT_INVERTER_CHECK:
        /* 4. Check AMK_bQuitInverterOn = 0 */
        if (!(motor->status.AMK_bQuitInverterOn)) {
            motor->states.deinit_state = AMK_DEINIT_DISABLE;
        }

        motor->states.deinit_state = AMK_DEINIT_DC_OFF;

        break;
    case AMK_DEINIT_DC_OFF:
        /* 5. Set AMK_bDcOn = 0 */

        motor->control.AMK_bDcOn = false;

        motor->states.deinit_state = AMK_DEINIT_DC_OFF_CHECK;

        break;
    case AMK_DEINIT_DC_OFF_CHECK:
        /* 6r. AMK_bDcOn is mirrored in AMK_Status, so should be on there */
        /* 6r. Check AMK_bQuitDcOn = 0 */
        /* When will AMK_bQuitDcOn go on? Does it take some time after 
         * DcOn is set?? */
        /* 3r. (QUE & AMK_bDcOn) -> Check AMK_bQuitDcOn = 1 */
            /* Does where do I check QUE??? */

        motor->states.deinit_state = AMK_DEINIT_PRECHARGE;

        break;
    case AMK_DEINIT_PRECHARGE:
        /* 7. Discharge DC caps; QUE should be reset (is this just DcOn?) */
        
        if (!(*motor->pchg_complete)) {
            motor->states.init_state = AMK_DEINIT_POWER_OFF;
        }

        /* If discharged, move on */
        motor->states.deinit_state = AMK_DEINIT_POWER_OFF;

        break;
    case AMK_DEINIT_POWER_OFF:
        /* 8. Turn off 24v DC to inverters */
        /* We don't do much here, just have to turn off the car I guess */
        motor->states.deinit_state = AMK_DEINIT_DONE;
        motor->states.state = AMK_STATE_OFF;

        break;
    }
}
