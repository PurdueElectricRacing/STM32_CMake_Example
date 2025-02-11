/**
 * @file amk.h
 * @author Cole Roberts (rober638@purdue.edu)
 * @author Chris McGalliard (cpmcgalliard@gmail.com)
 * @brief  Vroom
 * @version 0.1
 * @date 2025-2-11
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "amk.h"

 /**
 * Procedure: amkInit()
 * 
 * @brief Initializes all module objects
 * 
 * Must be called before all other AMK module procedures
 * 
 * @param motor Pointer to the current motor
 * @param pchg_complete Pointer to the car's precharge status
 * @param id PER chosen ID
 * 
 */

void amkInit(amk_motor_t* motor, bool* pchg_complete, uint8_t id)
{
    *motor = (amk_motor_t){
        .state = AMK_STATE_OFF,
        .pchg_complete = pchg_complete,
        .id = id,
        .torque_set_ppt_nom = ZERO_DECI_NM,
        .torque_lim_pos_ppt_nom = ZERO_DECI_NM,
        .torque_lim_neg_ppt_nom = TORQUE_LIM_NEG_DECI_NM
    };

} /* amkInit() */


 /**
 * Procedure: amkPeriodic()
 * 
 * @brief Run the AMK state machine
 * 
 * @param motor Pointer to the current motor
 * 
 */

void amkPeriodic(amk_motor_t* motor)
{
    amkGetData(motor);

    switch(motor->state)
    {
    case AMK_STATE_OFF:
        if (motor->status.AMK_bError)
        {
            // TODO look into a retry count, and look into conditions
            // for when multiple errors at once
            if (AMK_CAN_ERR_ID == motor->diagnostic_num)
            {
                amkReset(motor);
            }
            else
            {
                setFault(motor->error_fault_id, true);
            }
        }
        else
        {
            if (*(motor->pchg_complete))
            {
                // TODO also check DC on when we check pchg_complete?
                /* Do not proceed to INIT unless we are system ready */
                if ((motor->status.AMK_bSystemReady))
                {
                    // TODO: Also need to check READY2DRIVE
                    motor->state = AMK_STATE_INIT;
                }
            }
            else
            {
                // Wait for precharge and system ready
            }
        }
        break;
    
    case AMK_STATE_INIT:
        /* System is ready to run */
        if ((motor->status.AMK_bSystemReady) && *(motor->pchg_complete))
        {
            motor->torque_set_ppt_nom = 0;
            motor->torque_lim_pos_ppt_nom = 0;
            motor->torque_lim_neg_ppt_nom = 0;
            motor->control.AMK_bDcOn = true;
            motor->control.AMK_bInverterOn = true;
            motor->control.AMK_bEnable = true;
            motor->state = AMK_STATE_RUNNING;
        }
        /* System is not ready to run */
        else
        {
            motor->control.AMK_bDcOn = false;
            motor->control.AMK_bInverterOn = false;
            motor->control.AMK_bEnable = false;
            motor->state = AMK_STATE_OFF;
        }
        break;
    
    case AMK_STATE_RUNNING:
        /* System is ready to run */
        if ((motor->status.AMK_bSystemReady) && *(motor->pchg_complete))
        {
            /* System error */
            if (motor->status.AMK_bError)
            {
                // Make sure this is latching else it will set every time you
                // step through the reset function
                setFault(motor->error_fault_id, true);
                amkReset(motor);
                motor->state = AMK_STATE_OFF;
            }
            
            /* No errors, system healthy, torque command allowed */
            else
            {
                // TODO send the actual intended setpoint
                motor->torque_set_ppt_nom = 25;
                motor->torque_lim_pos_ppt_nom = 100;
                motor->torque_lim_neg_ppt_nom = -1;
            }
        }
        /* System is not ready to run */
        else
        {
            amkOff(motor);
        }
        break;
    }

    amkSendSetpoints(motor);

} /* amkPeriodic() */

/**
 * Procedure: amkSendSetpoints()
 * 
 * @brief Sends the setpoints message to the inverter
 * 
 * @param motor Pointer to the current motor
 * 
 */

 static void amkSendSetpoints(amk_motor_t* motor)
 {
    if (INVA_ID == motor->id)
    {
        SEND_INVA_SET(motor->control.AMK_bReserve1, motor->control.AMK_bInverterOn, motor->control.AMK_bDcOn,
            motor->control.AMK_bEnable, motor->control.AMK_bErrorReset,
            motor->control.AMK_bReserve2, motor->torque_set_ppt_nom, motor->torque_lim_pos_ppt_nom, motor->torque_lim_neg_ppt_nom);
        motor->control.AMK_bErrorReset = false;
    }
    else if (INVB_ID == motor->id)
    {
        SEND_INVB_SET(motor->control.AMK_bReserve1, motor->control.AMK_bInverterOn, motor->control.AMK_bDcOn,
            motor->control.AMK_bEnable, motor->control.AMK_bErrorReset,
            motor->control.AMK_bReserve2, motor->torque_set_ppt_nom, motor->torque_lim_pos_ppt_nom, motor->torque_lim_neg_ppt_nom);
        motor->control.AMK_bErrorReset = false;
    }
    else
    {
        // Sus
    }

 } /* amkSendSetpoints() */


/**
 * Procedure: amkOff()
 * 
 * @brief Moves the motor to the OFF state
 *      
 * This will forcibly move the state machine to the OFF
 *  state and prevent torque requests
 * 
 * @param motor Pointer to the current motor
 * 
 */

static void amkOff(amk_motor_t* motor)
{
    motor->control.AMK_bDcOn = false;
    motor->control.AMK_bInverterOn = false;
    motor->control.AMK_bEnable = false;
    motor->torque_set_ppt_nom = ZERO_DECI_NM;
    motor->torque_lim_pos_ppt_nom = ZERO_DECI_NM;
    motor->torque_lim_neg_ppt_nom = TORQUE_LIM_NEG_DECI_NM;
    motor->state = AMK_STATE_OFF;    

} /* amkOff() */

/* Sets the torque setpoint from -1000% to 1000% of nominal torque.
 * But this function just takes in -100% to 100% to stay consistent */

/**
 * Procedure: amkSetTorque()
 * 
 * @brief Updates the motor parameters
 *      
 * This will update the motor object with all newly received
 *  data from the CAN bus. Will send fault if any data is stale
 * 
 * @param motor Pointer to the current motor
 * @param torque_setpoint requested torque in percent of nominal
 * 
 */

void amkSetTorque(amk_motor_t* motor, int16_t torque_setpoint)
{
    if (torque_setpoint > MAX_POSITIVE_TORQUE_SETPOINT 
        || torque_setpoint < MAX_NEGATIVE_TORQUE_SETPOINT) {
        return;
    }

    /* Scale to ppt nominal */
    torque_setpoint *= 10;

    motor->torque_set_ppt_nom = torque_setpoint;
    motor->torque_lim_pos_ppt_nom = 1000;

    /* NOTE: For some reason it cannot be 0, so do -0.1% (according to UIUC's team) */
    motor->torque_lim_neg_ppt_nom = -1;

} /* amkSetTorque() */


 /**
 * Procedure: amkGetData()
 * 
 * @brief Updates the motor parameters
 *      
 * This will update the motor object with all newly received
 *  data from the CAN bus. Will send fault if any data is stale
 * 
 * @param motor Pointer to the current motor
 * 
 */

static void amkGetData(amk_motor_t* motor)
{
    if (INVA_ID == motor->id)
    {
        if (can_data.INVA_CRIT.stale  ||
            can_data.INVA_TEMPS.stale ||
            can_data.INVA_INFO.stale  ||
            can_data.INVA_ERR_1.stale ||
            can_data.INVA_ERR_2.stale)
        {
            setFault(motor->stale_fault_id, true);
        }
        else
        {
            motor->speed_act_RPM = can_data.INVA_CRIT.AMK_ActualSpeed;
            motor->torque_act_ppt_nom = can_data.INVA_CRIT.AMK_ActualTorque;
            motor->overload_inv = can_data.INVA_CRIT.AMK_DisplayOverloadInverter;
            motor->overload_motor = can_data.INVA_CRIT.AMK_DisplayOverloadMotor;
            motor->temp_motor_deci_C = can_data.INVA_TEMPS.AMK_MotorTemp;
            motor->temp_inv_deci_C = can_data.INVA_TEMPS.AMK_InverterTemp;
            motor->temp_igbt_deci_C = can_data.INVA_TEMPS.AMK_IGBTTemp;
            motor->status.AMK_bSystemReady = can_data.INVA_INFO.AMK_Status_bSystemReady;
            motor->status.AMK_bError = can_data.INVA_INFO.AMK_Status_bError;
            motor->status.AMK_bWarn = can_data.INVA_INFO.AMK_Status_bWarn;
            motor->status.AMK_bQuitDcOn = can_data.INVA_INFO.AMK_Status_bQuitDcOn;
            motor->status.AMK_bDcOn = can_data.INVA_INFO.AMK_Status_bDcOn;
            motor->status.AMK_bQuitInverterOn = can_data.INVA_INFO.AMK_Status_bQuitInverterOn;
            motor->status.AMK_bInverterOn = can_data.INVA_INFO.AMK_Status_bInverterOn;
            motor->status.AMK_bDerating = can_data.INVA_INFO.AMK_Status_bDerating;
            motor->dc_bus_voltage = can_data.INVA_INFO.AMK_DCBusVoltage;
            motor->diagnostic_num = can_data.INVA_ERR_1.AMK_DiagnosticNumber;
            motor->error_info_1 = can_data.INVA_ERR_1.AMK_ErrorInfo1;
            motor->error_info_2 = can_data.INVA_ERR_2.AMK_ErrorInfo2;
            motor->error_info_3 = can_data.INVA_ERR_2.AMK_ErrorInfo3;
        }
    }
    else if (INVB_ID == motor->id)
    {
        if (can_data.INVB_CRIT.stale  ||
            can_data.INVB_TEMPS.stale ||
            can_data.INVB_INFO.stale  ||
            can_data.INVB_ERR_1.stale ||
            can_data.INVB_ERR_2.stale)
        {
            setFault(motor->stale_fault_id, true);
        }
        else
        {
            motor->speed_act_RPM = can_data.INVB_CRIT.AMK_ActualSpeed;
            motor->torque_act_ppt_nom = can_data.INVB_CRIT.AMK_ActualTorque;
            motor->overload_inv = can_data.INVB_CRIT.AMK_DisplayOverloadInverter;
            motor->overload_motor = can_data.INVB_CRIT.AMK_DisplayOverloadMotor;
            motor->temp_motor_deci_C = can_data.INVB_TEMPS.AMK_MotorTemp;
            motor->temp_inv_deci_C = can_data.INVB_TEMPS.AMK_InverterTemp;
            motor->temp_igbt_deci_C = can_data.INVB_TEMPS.AMK_IGBTTemp;
            motor->status.AMK_bSystemReady = can_data.INVB_INFO.AMK_Status_bSystemReady;
            motor->status.AMK_bError = can_data.INVB_INFO.AMK_Status_bError;
            motor->status.AMK_bWarn = can_data.INVB_INFO.AMK_Status_bWarn;
            motor->status.AMK_bQuitDcOn = can_data.INVB_INFO.AMK_Status_bQuitDcOn;
            motor->status.AMK_bDcOn = can_data.INVB_INFO.AMK_Status_bDcOn;
            motor->status.AMK_bQuitInverterOn = can_data.INVB_INFO.AMK_Status_bQuitInverterOn;
            motor->status.AMK_bInverterOn = can_data.INVB_INFO.AMK_Status_bInverterOn;
            motor->status.AMK_bDerating = can_data.INVB_INFO.AMK_Status_bDerating;
            motor->dc_bus_voltage = can_data.INVB_INFO.AMK_DCBusVoltage;
            motor->diagnostic_num = can_data.INVB_ERR_1.AMK_DiagnosticNumber;
            motor->error_info_1 = can_data.INVB_ERR_1.AMK_ErrorInfo1;
            motor->error_info_2 = can_data.INVB_ERR_2.AMK_ErrorInfo2;
            motor->error_info_3 = can_data.INVB_ERR_2.AMK_ErrorInfo3;
        }
    }
    else
    {
        // sus
    }

} /* amkGetData() */


 /**
 * Procedure: amkReset()
 * 
 * @brief Resets the AMK
 * 
 * This will set the motor parameters up such that upon
 *  the next setpoints TX, the inverter will reset itself
 * 
 * @param motor Pointer to the current motor
 * 
 */

static void amkReset(amk_motor_t* motor)
{
    motor->control.AMK_bErrorReset = true;
    motor->control.AMK_bInverterOn = false;
    motor->torque_set_ppt_nom = ZERO_DECI_NM;
    motor->torque_lim_pos_ppt_nom = ZERO_DECI_NM;
    motor->torque_lim_neg_ppt_nom = TORQUE_LIM_NEG_DECI_NM;

} /* amkReset() */
