/**
 * @file amk.h
 * @author Cole Roberts (rober638@purdue.edu)
 * @brief  Vroom
 * @version 0.1
 * @date 2024-10-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _AMK_H_
#define _AMK_H_

#include <stdbool.h>
#include <stdint.h>

/* Inverter -> CAN */
/* In AMK_Actual_Values_1 */
typedef struct {
    uint16_t AMK_bReserve        : 8;
    uint16_t AMK_bSystemReady    : 1;
    uint16_t AMK_bError          : 1;
    uint16_t AMK_bWarn           : 1;
    uint16_t AMK_bQuitDcOn       : 1;
    uint16_t AMK_bDcOn           : 1; /* Same as QUE ?? */
    uint16_t AMK_bQuitInverterOn : 1;
    uint16_t AMK_bInverterOn     : 1;
    uint16_t AMK_bDerating       : 1;
} AMK_Status_t;

/* CAN -> Inverter */
/* In AMK_Setpoints */
/* THIS NEEDS TO BE SENT EVERY 50ms */
typedef struct {
    uint16_t AMK_bReserve1   : 8;
    uint16_t AMK_bInverterOn : 1;
    uint16_t AMK_bDcOn       : 1;
    uint16_t AMK_bEnable     : 1;
    uint16_t AMK_bErrorReset : 1;
    uint16_t AMK_bReserve2   : 1;
} AMK_Control_t;

typedef struct {
    uint8_t main_state;
    uint8_t running_state;
    uint8_t init_state;
    uint8_t deinit_state;
    uint8_t reset_state;
} amk_motor_states_t;

typedef struct {
    amk_motor_states_t states;
    AMK_Status_t status;
    AMK_Control_t control;

    /* Scaling: 0.1% */
    int16_t torque_setpoint;
    int16_t torque_limit_positive;
    int16_t torque_limit_negative;

    /* Scaling: 0.1% */
    int16_t actual_torque;
    int16_t actual_speed;

    uint32_t serial_num; // for sanity checking

    /* Scaling: 0.1% */
    int16_t motor_temp;
    int16_t inverter_temp;
    int16_t igbt_temp;

    uint16_t dc_bus_voltage;
    uint16_t system_reset;

    uint16_t diagnostic_num;
    uint32_t error_info_1;
    uint32_t error_info_2;
    uint32_t error_info_3;

    bool* pchg_complete;

    int stale_fault_id;
    int error_fault_id;

    void (*sendSetpoints)();
} amk_motor_t;

typedef enum {
    AMK_INIT_POWER_ON = 0,
    AMK_INIT_ENABLE   = 1,
} amk_motor_init_state_t;

typedef enum {
    AMK_DEINIT_DISABLE   = 0,
    AMK_DEINIT_POWER_OFF = 1
} amk_motor_deinit_state_t;

typedef enum {
    AMK_RUNNING_GOOD  = 0,
    AMK_RUNNING_ERROR = 1
} amk_motor_running_state_t;

typedef enum {
    AMK_RESET_INVERTER_OFF       = 0,
    AMK_RESET_ERROR_RESET_ON     = 1,
    AMK_RESET_ERROR_RESET_OFF    = 2,
    AMK_RESET_CHECK_SYSTEM_READY = 3
} amk_motor_reset_state_t;

typedef enum {
    AMK_STATE_OFF      = 0,
    AMK_STATE_INIT     = 1,
    AMK_STATE_DEINIT   = 2,
    AMK_STATE_RUNNING  = 3
} amk_motor_state_t;

#define DEFAULT_TORQUE_SETPOINT       (0)
#define DEFAULT_POSITIVE_TORQUE_LIMIT (0)
#define DEFAULT_NEGATIVE_TORQUE_LIMIT (0)

#define MAX_POSITIVE_TORQUE_SETPOINT  (1000)
#define MAX_NEGATIVE_TORQUE_SETPOINT  (0)
#define MAX_POSITIVE_TORQUE_LIMIT     (1000)
#define MAX_NEGATIVE_TORQUE_LIMIT     (-1000)

void amkInit(amk_motor_t* motor, bool* pchg_complete, void (*sendSetpoints)());
void amkPeriodic(amk_motor_t* motor);
void amkSetTorque(amk_motor_t* motor, int16_t torque_setpoint);
static void resetAmk(amk_motor_t* motor);
static void turnAmkOn(amk_motor_t* motor);
static void amkRunning(amk_motor_t* motor);
static void turnAmkOff(amk_motor_t* motor);
static void amkGetData(amk_motor_t* motor);
static void amkReset(amk_motor_t* motor);

#endif /* _AMK_H_ */
