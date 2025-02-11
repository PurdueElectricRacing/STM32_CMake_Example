/**
 * @file amk.h
 * @author Cole Roberts (rober638@purdue.edu)
 * @brief  Vroom
 * @version 0.1
 * @date 2025-2-11
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _AMK_H_
#define _AMK_H_

#define INVA_ID (1U)
#define INVB_ID (2U)
#define AMK_CAN_ERR_ID (3587U)

#define ZERO_DECI_NM (0)
#define TORQUE_LIM_NEG_DECI_NM (-1)

#define MAX_POSITIVE_TORQUE_SETPOINT  (1000)
#define MAX_NEGATIVE_TORQUE_SETPOINT  (0)
#define MAX_POSITIVE_TORQUE_LIMIT     (1000)
#define MAX_NEGATIVE_TORQUE_LIMIT     (-1000)

#include <stdbool.h>
#include <stdint.h>

#include "common/faults/faults.h"
#include "source/main_module/can/can_parse.h"

/* -------------------------------------------------------
    AMK Inverter Status
-------------------------------------------------------- */

typedef struct
{
    uint16_t AMK_bReserve        : 8; /* Reserved */
    uint16_t AMK_bSystemReady    : 1; /* System Ready */
    uint16_t AMK_bError          : 1; /* Error */
    uint16_t AMK_bWarn           : 1; /* Warning */
    uint16_t AMK_bQuitDcOn       : 1; /* HV activiation acknowledgement */
    uint16_t AMK_bDcOn           : 1; /* HV activation level */
    uint16_t AMK_bQuitInverterOn : 1; /* Controller enable acknowledgement */
    uint16_t AMK_bInverterOn     : 1; /* Controller enable level */
    uint16_t AMK_bDerating       : 1; /* Derating (torque limitation active) */
} AMK_Status_t;

/* -------------------------------------------------------
    These parameters are required for the
    AMK Inverter Setpoints Message
    Telegram monitoring will trigger
    upon no RX in 50ms
-------------------------------------------------------- */
typedef struct
{
    uint16_t AMK_bReserve1   : 8; /* Reserved */
    uint16_t AMK_bInverterOn : 1; /* Controller Enable */
    uint16_t AMK_bDcOn       : 1; /* HV activiation */
    uint16_t AMK_bEnable     : 1; /* Driver Enable */
    uint16_t AMK_bErrorReset : 1; /* Remove Error */
    uint16_t AMK_bReserve2   : 1; /* Reserved */
} AMK_Control_t;

/* -------------------------------------------------------
    PER internal AMK states
-------------------------------------------------------- */
typedef enum
{
    AMK_STATE_OFF      = 0,
    AMK_STATE_INIT     = 1,
    AMK_STATE_DEINIT   = 2,
    AMK_STATE_RUNNING  = 3
} amk_motor_state_t;

/* -------------------------------------------------------
    AMK Main Struct
-------------------------------------------------------- */
typedef struct
{
    uint8_t id;                     /* PER chosen ID */
    amk_motor_state_t state;        /* AMK State */
    AMK_Status_t status;            /* Inverter Status */
    AMK_Control_t control;          /* Inverter Control Word */

    /* All system torque values refer to ID32771 'Nominal torque'
       and are specified to 0.1 % Mn of its value, also known
       as ppt: parts per thousand of nominal */
    int16_t torque_set_ppt_nom;     /* Torque Setpoint in 0.1% of nominal */
    int16_t torque_lim_pos_ppt_nom; /* Positive Torque Limit in 0.1% of nominal */
    int16_t torque_lim_neg_ppt_nom; /* Negative Torque Limit in 0.1% of nominal */
    int16_t torque_act_ppt_nom;     /* Actual Torque in 0.1% of nominal */
    uint16_t overload_inv;          /* Display overload inverter in 0.1% */
    uint16_t overload_motor;        /* Display overload motor in 0.1% */
    int16_t speed_act_RPM;          /* Actual Speed in revolutions-per-minute */
    int16_t temp_motor_deci_C;      /* Motor Temperature in deci-Celcius */
    int16_t temp_inv_deci_C;        /* Inverter Temperature in deci-Celcius*/
    int16_t temp_igbt_deci_C;       /* IGBT Temperature in deci-Celcius*/
    uint16_t dc_bus_voltage;        /* Inverter-Reported DC Bus Voltage */
    uint16_t diagnostic_num;        /* Inverter-Reported Diagnostic Code */
    uint32_t error_info_1;          /* Inverter-Reported Error Part 1 */
    uint32_t error_info_2;          /* Inverter-Reported Error Part 2 */
    uint32_t error_info_3;          /* Inverter-Reported Error Part 3 */
    bool* pchg_complete;            /* Pointer to the car's precharge status */
    int stale_fault_id;             /* Stale CAN fault ID */
    int error_fault_id;             /* Error CAN fault ID */

} amk_motor_t;

void amkInit(amk_motor_t* motor, bool* pchg_complete, uint8_t id);
void amkPeriodic(amk_motor_t* motor);
void amkSetTorque(amk_motor_t* motor, int16_t torque_setpoint);

static void amkOff(amk_motor_t* motor);
static void amkReset(amk_motor_t* motor);
static void amkSendSetpoints(amk_motor_t* motor);
static void amkGetData(amk_motor_t* motor);

#endif /* _AMK_H_ */
