/**
 * @file faults.h
 * @author Aditya Anand (anand89@purdue.edu)
 * @brief Creating a library of faults to create an easy to debug atmosphere on the car
 * @version 0.1
 * @date 2022-05-11
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _FAULTS_H_
#define _FAULTS_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include "common/queue/queue.h"

#define MAX_MSG_SIZE 75

//BEGIN AUTO TOTAL DEFS
#define TOTAL_PDU_FAULTS 10
#define TOTAL_MAIN_MODULE_FAULTS 29
#define TOTAL_DASHBOARD_FAULTS 6
#define TOTAL_A_BOX_FAULTS 29
#define TOTAL_TORQUE_VECTOR_FAULTS 3
#define TOTAL_TEST_FAULTS 4
#define TOTAL_MCU_NUM 6
#define TOTAL_NUM_FAULTS 81
//END AUTO TOTAL DEFS

//BEGIN AUTO ID DEFS
#define ID_PDU_MCU_TEMP_HIGH_FAULT 0x0
#define ID_DASH_RAIL_FAULT 0x1
#define ID_ABOX_RAIL_FAULT 0x2
#define ID_MAIN_RAIL_FAULT 0x3
#define ID_DAQ_RAIL_FAULT 0x4
#define ID_V_CRIT_FAULT 0x5
#define ID_V_NONCRIT_FAULT 0x6
#define ID_FAN1_FAULT 0x7
#define ID_BULLET_RAIL_FAULT 0x8
#define ID_FAN2_FAULT 0x9
#define ID_PCHG_IMPLAUS_FAULT 0x100a
#define ID_RTD_EXIT_FAULT 0x100b
#define ID_LEFT_MC_CONN_FAULT 0x100c
#define ID_RIGHT_MC_CONN_FAULT 0x100d
#define ID_MCU_TEMP_HIGH_FAULT 0x100e
#define ID_DT_L_TEMP_OT_FAULT 0x100f
#define ID_DT_R_TEMP_OT_FAULT 0x1010
#define ID_DT_L_TEMP_HIGH_FAULT 0x1011
#define ID_DT_R_TEMP_HIGH_FAULT 0x1012
#define ID_BAT_COOL_LOOP_HIGH_FAULT 0x1013
#define ID_DT_COOL_LOOP_HIGH_FAULT 0x1014
#define ID_DT_R_THERM_DISC_FAULT 0x1015
#define ID_DT_L_THERM_DISC_FAULT 0x1016
#define ID_BATT_CL_DISC_FAULT 0x1017
#define ID_DT_CL_DISC_FAULT 0x1018
#define ID_BSPD_LATCHED_FAULT 0x1019
#define ID_BOTS_FAIL_FAULT 0x101a
#define ID_INERTIA_FAIL_FAULT 0x101b
#define ID_COCKPIT_ESTOP_FAULT 0x101c
#define ID_RIGHT_ESTOP_FAULT 0x101d
#define ID_LEFT_ESTOP_FAULT 0x101e
#define ID_HVD_DISC_FAULT 0x101f
#define ID_HUB_DISC_FAULT 0x1020
#define ID_TSMS_DISC_FAULT 0x1021
#define ID_PRECHARGE_TIME_FAULT_FAULT 0x1022
#define ID_MOTOR_R_HEAT_FAULT 0x1023
#define ID_MOTOR_L_HEAT_FAULT 0x1024
#define ID_MOTOR_R_OT_FAULT 0x1025
#define ID_MOTOR_L_OT_FAULT 0x1026
#define ID_APPS_WIRING_T1_FAULT 0x2027
#define ID_APPS_WIRING_T2_FAULT 0x2028
#define ID_BSE_FAULT 0x2029
#define ID_BSPD_FAULT 0x202a
#define ID_IMPLAUS_DETECTED_FAULT 0x202b
#define ID_APPS_BRAKE_FAULT 0x202c
#define ID_DISCHARGE_LIMIT_ENFORCE_FAULT 0x302d
#define ID_CHARGER_SAFETY_RELAY_FAULT 0x302e
#define ID_INTERNAL_HARDWARE_FAULT 0x302f
#define ID_HEATSINK_THERMISTOR_FAULT 0x3030
#define ID_SOFTWARE_FAULT 0x3031
#define ID_MAX_CELLV_HIGH_FAULT 0x3032
#define ID_MIN_CELLV_LOW_FAULT 0x3033
#define ID_PACK_OVERHEAT_ORION_FAULT 0x3034
#define ID_INTERNAL_COMMS_FAULT 0x3035
#define ID_CELL_BALANCING_FOFF_FAULT 0x3036
#define ID_WEAK_CELL_FAULT 0x3037
#define ID_LOW_CELLV_FAULT 0x3038
#define ID_OPEN_WIRE_FAULT 0x3039
#define ID_CURRENT_SENSOR_FAULT 0x303a
#define ID_MAX_CELLV_O5V_FAULT 0x303b
#define ID_CELL_ASIC_FAULT 0x303c
#define ID_WEAK_PACK_FAULT 0x303d
#define ID_FAN_MONITOR_FAULT 0x303e
#define ID_THERMISTOR_FAULT 0x303f
#define ID_EXTERNAL_COMMS_FAULT 0x3040
#define ID_REDUNDANT_PSU_FAULT 0x3041
#define ID_HV_ISOLATION_FAULT 0x3042
#define ID_INPUT_PSU_FAULT 0x3043
#define ID_CHARGE_LIMIT_ENFORCE_FAULT 0x3044
#define ID_PACK_TEMP_FAULT 0x3045
#define ID_PACK_TEMP_EXCEEDED_FAULT 0x3046
#define ID_MIN_PACK_TEMP_FAULT 0x3047
#define ID_IMD_FAULT 0x3048
#define ID_TMU_POWER_LOST_FAULT 0x3049
#define ID_TV_DISABLED_FAULT 0x404a
#define ID_TV_UNCALIBRATED_FAULT 0x404b
#define ID_NO_GPS_FIX_FAULT 0x404c
#define ID_TEST_FAULT_1_FAULT 0x504d
#define ID_TEST_FAULT_2_FAULT 0x504e
#define ID_TEST_FAULT_3_FAULT 0x504f
#define ID_TEST_FAULT_4_FAULT 0x5050
//END AUTO ID DEFS

//Macro defs for accessing aspects of id
#define GET_IDX(id) (id & 0x0FFF)
#define GET_OWNER(id) ((id & 0xF000) >> 12)


//WARNING: Doesn't affect driving state (Car can still safely drive)
//CRITICAL: Car exits ready2drive, but LV + HV systems still active
//FATAL: The Car SDC is activated
//BEGIN AUTO PRIORITY DEFS
#define PDU_MCU_TEMP_HIGH_PRIORITY 0
#define DASH_RAIL_PRIORITY 2
#define ABOX_RAIL_PRIORITY 0
#define MAIN_RAIL_PRIORITY 2
#define DAQ_RAIL_PRIORITY 0
#define V_CRIT_PRIORITY 2
#define V_NONCRIT_PRIORITY 0
#define FAN1_PRIORITY 0
#define BULLET_RAIL_PRIORITY 0
#define FAN2_PRIORITY 0
#define PCHG_IMPLAUS_PRIORITY 1
#define RTD_EXIT_PRIORITY 0
#define LEFT_MC_CONN_PRIORITY 1
#define RIGHT_MC_CONN_PRIORITY 1
#define MCU_TEMP_HIGH_PRIORITY 0
#define DT_L_TEMP_OT_PRIORITY 0
#define DT_R_TEMP_OT_PRIORITY 0
#define DT_L_TEMP_HIGH_PRIORITY 0
#define DT_R_TEMP_HIGH_PRIORITY 0
#define BAT_COOL_LOOP_HIGH_PRIORITY 0
#define DT_COOL_LOOP_HIGH_PRIORITY 0
#define DT_R_THERM_DISC_PRIORITY 0
#define DT_L_THERM_DISC_PRIORITY 0
#define BATT_CL_DISC_PRIORITY 0
#define DT_CL_DISC_PRIORITY 0
#define BSPD_LATCHED_PRIORITY 1
#define BOTS_FAIL_PRIORITY 1
#define INERTIA_FAIL_PRIORITY 1
#define COCKPIT_ESTOP_PRIORITY 1
#define RIGHT_ESTOP_PRIORITY 1
#define LEFT_ESTOP_PRIORITY 1
#define HVD_DISC_PRIORITY 0
#define HUB_DISC_PRIORITY 1
#define TSMS_DISC_PRIORITY 0
#define PRECHARGE_TIME_FAULT_PRIORITY 2
#define MOTOR_R_HEAT_PRIORITY 1
#define MOTOR_L_HEAT_PRIORITY 1
#define MOTOR_R_OT_PRIORITY 1
#define MOTOR_L_OT_PRIORITY 1
#define APPS_WIRING_T1_PRIORITY 1
#define APPS_WIRING_T2_PRIORITY 1
#define BSE_PRIORITY 1
#define BSPD_PRIORITY 1
#define IMPLAUS_DETECTED_PRIORITY 1
#define APPS_BRAKE_PRIORITY 0
#define DISCHARGE_LIMIT_ENFORCE_PRIORITY 1
#define CHARGER_SAFETY_RELAY_PRIORITY 1
#define INTERNAL_HARDWARE_PRIORITY 1
#define HEATSINK_THERMISTOR_PRIORITY 1
#define SOFTWARE_PRIORITY 1
#define MAX_CELLV_HIGH_PRIORITY 1
#define MIN_CELLV_LOW_PRIORITY 1
#define PACK_OVERHEAT_ORION_PRIORITY 1
#define INTERNAL_COMMS_PRIORITY 1
#define CELL_BALANCING_FOFF_PRIORITY 1
#define WEAK_CELL_PRIORITY 1
#define LOW_CELLV_PRIORITY 1
#define OPEN_WIRE_PRIORITY 1
#define CURRENT_SENSOR_PRIORITY 1
#define MAX_CELLV_O5V_PRIORITY 1
#define CELL_ASIC_PRIORITY 1
#define WEAK_PACK_PRIORITY 1
#define FAN_MONITOR_PRIORITY 1
#define THERMISTOR_PRIORITY 1
#define EXTERNAL_COMMS_PRIORITY 1
#define REDUNDANT_PSU_PRIORITY 1
#define HV_ISOLATION_PRIORITY 1
#define INPUT_PSU_PRIORITY 1
#define CHARGE_LIMIT_ENFORCE_PRIORITY 1
#define PACK_TEMP_PRIORITY 0
#define PACK_TEMP_EXCEEDED_PRIORITY 1
#define MIN_PACK_TEMP_PRIORITY 1
#define IMD_PRIORITY 2
#define TMU_POWER_LOST_PRIORITY 1
#define TV_DISABLED_PRIORITY 0
#define TV_UNCALIBRATED_PRIORITY 0
#define NO_GPS_FIX_PRIORITY 0
#define TEST_FAULT_1_PRIORITY 0
#define TEST_FAULT_2_PRIORITY 1
#define TEST_FAULT_3_PRIORITY 2
#define TEST_FAULT_4_PRIORITY 0
//END AUTO PRIORITY DEFS

//BEGIN AUTO MAX DEFS
#define PDU_MCU_TEMP_HIGH_MAX 50
#define DASH_RAIL_MAX 1
#define ABOX_RAIL_MAX 1
#define MAIN_RAIL_MAX 1
#define DAQ_RAIL_MAX 1
#define V_CRIT_MAX 1
#define V_NONCRIT_MAX 1
#define FAN1_MAX 1
#define BULLET_RAIL_MAX 1
#define FAN2_MAX 1
#define PCHG_IMPLAUS_MAX 1
#define RTD_EXIT_MAX 1
#define LEFT_MC_CONN_MAX 1
#define RIGHT_MC_CONN_MAX 1
#define MCU_TEMP_HIGH_MAX 50
#define DT_L_TEMP_OT_MAX 90
#define DT_R_TEMP_OT_MAX 90
#define DT_L_TEMP_HIGH_MAX 80
#define DT_R_TEMP_HIGH_MAX 80
#define BAT_COOL_LOOP_HIGH_MAX 50
#define DT_COOL_LOOP_HIGH_MAX 90
#define DT_R_THERM_DISC_MAX 200
#define DT_L_THERM_DISC_MAX 200
#define BATT_CL_DISC_MAX 200
#define DT_CL_DISC_MAX 200
#define BSPD_LATCHED_MAX 1
#define BOTS_FAIL_MAX 1
#define INERTIA_FAIL_MAX 1
#define COCKPIT_ESTOP_MAX 1
#define RIGHT_ESTOP_MAX 1
#define LEFT_ESTOP_MAX 1
#define HVD_DISC_MAX 1
#define HUB_DISC_MAX 1
#define TSMS_DISC_MAX 1
#define PRECHARGE_TIME_FAULT_MAX 9500
#define MOTOR_R_HEAT_MAX 80
#define MOTOR_L_HEAT_MAX 80
#define MOTOR_R_OT_MAX 80
#define MOTOR_L_OT_MAX 80
#define APPS_WIRING_T1_MAX 4000
#define APPS_WIRING_T2_MAX 4000
#define BSE_MAX 1
#define BSPD_MAX 182
#define IMPLAUS_DETECTED_MAX 700
#define APPS_BRAKE_MAX 1
#define DISCHARGE_LIMIT_ENFORCE_MAX 1
#define CHARGER_SAFETY_RELAY_MAX 1
#define INTERNAL_HARDWARE_MAX 1
#define HEATSINK_THERMISTOR_MAX 1
#define SOFTWARE_MAX 1
#define MAX_CELLV_HIGH_MAX 1
#define MIN_CELLV_LOW_MAX 1
#define PACK_OVERHEAT_ORION_MAX 1
#define INTERNAL_COMMS_MAX 1
#define CELL_BALANCING_FOFF_MAX 1
#define WEAK_CELL_MAX 1
#define LOW_CELLV_MAX 1
#define OPEN_WIRE_MAX 1
#define CURRENT_SENSOR_MAX 1
#define MAX_CELLV_O5V_MAX 1
#define CELL_ASIC_MAX 1
#define WEAK_PACK_MAX 1
#define FAN_MONITOR_MAX 1
#define THERMISTOR_MAX 1
#define EXTERNAL_COMMS_MAX 1
#define REDUNDANT_PSU_MAX 1
#define HV_ISOLATION_MAX 1
#define INPUT_PSU_MAX 1
#define CHARGE_LIMIT_ENFORCE_MAX 1
#define PACK_TEMP_MAX 500
#define PACK_TEMP_EXCEEDED_MAX 600
#define MIN_PACK_TEMP_MAX 10000
#define IMD_MAX 1
#define TMU_POWER_LOST_MAX 5
#define TV_DISABLED_MAX 1
#define TV_UNCALIBRATED_MAX 1
#define NO_GPS_FIX_MAX 1
#define TEST_FAULT_1_MAX 1
#define TEST_FAULT_2_MAX 1
#define TEST_FAULT_3_MAX 1
#define TEST_FAULT_4_MAX 123
//END AUTO MAX DEFS

//BEGIN AUTO MIN DEFS
#define PDU_MCU_TEMP_HIGH_MIN 0
#define DASH_RAIL_MIN 0
#define ABOX_RAIL_MIN 0
#define MAIN_RAIL_MIN 0
#define DAQ_RAIL_MIN 0
#define V_CRIT_MIN 0
#define V_NONCRIT_MIN 0
#define FAN1_MIN 0
#define BULLET_RAIL_MIN 0
#define FAN2_MIN 0
#define PCHG_IMPLAUS_MIN 0
#define RTD_EXIT_MIN 0
#define LEFT_MC_CONN_MIN 0
#define RIGHT_MC_CONN_MIN 0
#define MCU_TEMP_HIGH_MIN 0
#define DT_L_TEMP_OT_MIN -100
#define DT_R_TEMP_OT_MIN -100
#define DT_L_TEMP_HIGH_MIN -100
#define DT_R_TEMP_HIGH_MIN -100
#define BAT_COOL_LOOP_HIGH_MIN -100
#define DT_COOL_LOOP_HIGH_MIN -100
#define DT_R_THERM_DISC_MIN -30
#define DT_L_THERM_DISC_MIN -30
#define BATT_CL_DISC_MIN -30
#define DT_CL_DISC_MIN -30
#define BSPD_LATCHED_MIN 0
#define BOTS_FAIL_MIN 0
#define INERTIA_FAIL_MIN 0
#define COCKPIT_ESTOP_MIN 0
#define RIGHT_ESTOP_MIN 0
#define LEFT_ESTOP_MIN 0
#define HVD_DISC_MIN 0
#define HUB_DISC_MIN 0
#define TSMS_DISC_MIN 0
#define PRECHARGE_TIME_FAULT_MIN -10
#define MOTOR_R_HEAT_MIN -80
#define MOTOR_L_HEAT_MIN -80
#define MOTOR_R_OT_MIN -80
#define MOTOR_L_OT_MIN -80
#define APPS_WIRING_T1_MIN 100
#define APPS_WIRING_T2_MIN 100
#define BSE_MIN 0
#define BSPD_MIN 0
#define IMPLAUS_DETECTED_MIN 0
#define APPS_BRAKE_MIN 0
#define DISCHARGE_LIMIT_ENFORCE_MIN 0
#define CHARGER_SAFETY_RELAY_MIN 0
#define INTERNAL_HARDWARE_MIN 0
#define HEATSINK_THERMISTOR_MIN 0
#define SOFTWARE_MIN 0
#define MAX_CELLV_HIGH_MIN 0
#define MIN_CELLV_LOW_MIN 0
#define PACK_OVERHEAT_ORION_MIN 0
#define INTERNAL_COMMS_MIN 0
#define CELL_BALANCING_FOFF_MIN 0
#define WEAK_CELL_MIN 0
#define LOW_CELLV_MIN 0
#define OPEN_WIRE_MIN 0
#define CURRENT_SENSOR_MIN 0
#define MAX_CELLV_O5V_MIN 0
#define CELL_ASIC_MIN 0
#define WEAK_PACK_MIN 0
#define FAN_MONITOR_MIN 0
#define THERMISTOR_MIN 0
#define EXTERNAL_COMMS_MIN 0
#define REDUNDANT_PSU_MIN 0
#define HV_ISOLATION_MIN 0
#define INPUT_PSU_MIN 0
#define CHARGE_LIMIT_ENFORCE_MIN 0
#define PACK_TEMP_MIN 0
#define PACK_TEMP_EXCEEDED_MIN 0
#define MIN_PACK_TEMP_MIN 100
#define IMD_MIN 0
#define TMU_POWER_LOST_MIN 3
#define TV_DISABLED_MIN 0
#define TV_UNCALIBRATED_MIN 0
#define NO_GPS_FIX_MIN 0
#define TEST_FAULT_1_MIN 0
#define TEST_FAULT_2_MIN 0
#define TEST_FAULT_3_MIN 0
#define TEST_FAULT_4_MIN 5
//END AUTO MIN DEFS

//BEGIN AUTO LATCH DEFS
#define PDU_MCU_TEMP_HIGH_LATCH_TIME 1000
#define DASH_RAIL_LATCH_TIME 100
#define ABOX_RAIL_LATCH_TIME 100
#define MAIN_RAIL_LATCH_TIME 100
#define DAQ_RAIL_LATCH_TIME 100
#define V_CRIT_LATCH_TIME 100
#define V_NONCRIT_LATCH_TIME 100
#define FAN1_LATCH_TIME 100
#define BULLET_RAIL_LATCH_TIME 100
#define FAN2_LATCH_TIME 100
#define PCHG_IMPLAUS_LATCH_TIME 50
#define RTD_EXIT_LATCH_TIME 100
#define LEFT_MC_CONN_LATCH_TIME 3000
#define RIGHT_MC_CONN_LATCH_TIME 3000
#define MCU_TEMP_HIGH_LATCH_TIME 1000
#define DT_L_TEMP_OT_LATCH_TIME 1000
#define DT_R_TEMP_OT_LATCH_TIME 1000
#define DT_L_TEMP_HIGH_LATCH_TIME 1000
#define DT_R_TEMP_HIGH_LATCH_TIME 1000
#define BAT_COOL_LOOP_HIGH_LATCH_TIME 1000
#define DT_COOL_LOOP_HIGH_LATCH_TIME 1000
#define DT_R_THERM_DISC_LATCH_TIME 1000
#define DT_L_THERM_DISC_LATCH_TIME 1000
#define BATT_CL_DISC_LATCH_TIME 1000
#define DT_CL_DISC_LATCH_TIME 1000
#define BSPD_LATCHED_LATCH_TIME 1500
#define BOTS_FAIL_LATCH_TIME 1500
#define INERTIA_FAIL_LATCH_TIME 1500
#define COCKPIT_ESTOP_LATCH_TIME 1500
#define RIGHT_ESTOP_LATCH_TIME 1500
#define LEFT_ESTOP_LATCH_TIME 1500
#define HVD_DISC_LATCH_TIME 1500
#define HUB_DISC_LATCH_TIME 1500
#define TSMS_DISC_LATCH_TIME 1500
#define PRECHARGE_TIME_FAULT_LATCH_TIME 400
#define MOTOR_R_HEAT_LATCH_TIME 1000
#define MOTOR_L_HEAT_LATCH_TIME 1000
#define MOTOR_R_OT_LATCH_TIME 1000
#define MOTOR_L_OT_LATCH_TIME 1000
#define APPS_WIRING_T1_LATCH_TIME 10
#define APPS_WIRING_T2_LATCH_TIME 10
#define BSE_LATCH_TIME 100
#define BSPD_LATCH_TIME 100
#define IMPLAUS_DETECTED_LATCH_TIME 10
#define APPS_BRAKE_LATCH_TIME 10
#define DISCHARGE_LIMIT_ENFORCE_LATCH_TIME 5
#define CHARGER_SAFETY_RELAY_LATCH_TIME 5
#define INTERNAL_HARDWARE_LATCH_TIME 5
#define HEATSINK_THERMISTOR_LATCH_TIME 5
#define SOFTWARE_LATCH_TIME 5
#define MAX_CELLV_HIGH_LATCH_TIME 5
#define MIN_CELLV_LOW_LATCH_TIME 5
#define PACK_OVERHEAT_ORION_LATCH_TIME 5
#define INTERNAL_COMMS_LATCH_TIME 5
#define CELL_BALANCING_FOFF_LATCH_TIME 5
#define WEAK_CELL_LATCH_TIME 5
#define LOW_CELLV_LATCH_TIME 5
#define OPEN_WIRE_LATCH_TIME 5
#define CURRENT_SENSOR_LATCH_TIME 5
#define MAX_CELLV_O5V_LATCH_TIME 5
#define CELL_ASIC_LATCH_TIME 5
#define WEAK_PACK_LATCH_TIME 5
#define FAN_MONITOR_LATCH_TIME 5
#define THERMISTOR_LATCH_TIME 5
#define EXTERNAL_COMMS_LATCH_TIME 5
#define REDUNDANT_PSU_LATCH_TIME 5
#define HV_ISOLATION_LATCH_TIME 5
#define INPUT_PSU_LATCH_TIME 5
#define CHARGE_LIMIT_ENFORCE_LATCH_TIME 5
#define PACK_TEMP_LATCH_TIME 2000
#define PACK_TEMP_EXCEEDED_LATCH_TIME 500
#define MIN_PACK_TEMP_LATCH_TIME 2000
#define IMD_LATCH_TIME 100
#define TMU_POWER_LOST_LATCH_TIME 100
#define TV_DISABLED_LATCH_TIME 10
#define TV_UNCALIBRATED_LATCH_TIME 10
#define NO_GPS_FIX_LATCH_TIME 10
#define TEST_FAULT_1_LATCH_TIME 10
#define TEST_FAULT_2_LATCH_TIME 10
#define TEST_FAULT_3_LATCH_TIME 10
#define TEST_FAULT_4_LATCH_TIME 10
//END AUTO LATCH DEFS

//BEGIN AUTO UNLATCH DEFS
#define PDU_MCU_TEMP_HIGH_UNLATCH_TIME 2000
#define DASH_RAIL_UNLATCH_TIME 1000
#define ABOX_RAIL_UNLATCH_TIME 1000
#define MAIN_RAIL_UNLATCH_TIME 1000
#define DAQ_RAIL_UNLATCH_TIME 1000
#define V_CRIT_UNLATCH_TIME 1000
#define V_NONCRIT_UNLATCH_TIME 1000
#define FAN1_UNLATCH_TIME 1000
#define BULLET_RAIL_UNLATCH_TIME 1000
#define FAN2_UNLATCH_TIME 1000
#define PCHG_IMPLAUS_UNLATCH_TIME 1000
#define RTD_EXIT_UNLATCH_TIME 1000
#define LEFT_MC_CONN_UNLATCH_TIME 1000
#define RIGHT_MC_CONN_UNLATCH_TIME 1000
#define MCU_TEMP_HIGH_UNLATCH_TIME 2000
#define DT_L_TEMP_OT_UNLATCH_TIME 1000
#define DT_R_TEMP_OT_UNLATCH_TIME 1000
#define DT_L_TEMP_HIGH_UNLATCH_TIME 1000
#define DT_R_TEMP_HIGH_UNLATCH_TIME 1000
#define BAT_COOL_LOOP_HIGH_UNLATCH_TIME 1000
#define DT_COOL_LOOP_HIGH_UNLATCH_TIME 1000
#define DT_R_THERM_DISC_UNLATCH_TIME 1000
#define DT_L_THERM_DISC_UNLATCH_TIME 1000
#define BATT_CL_DISC_UNLATCH_TIME 1000
#define DT_CL_DISC_UNLATCH_TIME 1000
#define BSPD_LATCHED_UNLATCH_TIME 10000
#define BOTS_FAIL_UNLATCH_TIME 1000
#define INERTIA_FAIL_UNLATCH_TIME 1000
#define COCKPIT_ESTOP_UNLATCH_TIME 1000
#define RIGHT_ESTOP_UNLATCH_TIME 1000
#define LEFT_ESTOP_UNLATCH_TIME 1000
#define HVD_DISC_UNLATCH_TIME 1000
#define HUB_DISC_UNLATCH_TIME 1000
#define TSMS_DISC_UNLATCH_TIME 1000
#define PRECHARGE_TIME_FAULT_UNLATCH_TIME 10000
#define MOTOR_R_HEAT_UNLATCH_TIME 1000
#define MOTOR_L_HEAT_UNLATCH_TIME 1000
#define MOTOR_R_OT_UNLATCH_TIME 1000
#define MOTOR_L_OT_UNLATCH_TIME 1000
#define APPS_WIRING_T1_UNLATCH_TIME 1000
#define APPS_WIRING_T2_UNLATCH_TIME 1000
#define BSE_UNLATCH_TIME 1000
#define BSPD_UNLATCH_TIME 1000
#define IMPLAUS_DETECTED_UNLATCH_TIME 100
#define APPS_BRAKE_UNLATCH_TIME 1000
#define DISCHARGE_LIMIT_ENFORCE_UNLATCH_TIME 5
#define CHARGER_SAFETY_RELAY_UNLATCH_TIME 5
#define INTERNAL_HARDWARE_UNLATCH_TIME 5
#define HEATSINK_THERMISTOR_UNLATCH_TIME 5
#define SOFTWARE_UNLATCH_TIME 5
#define MAX_CELLV_HIGH_UNLATCH_TIME 5
#define MIN_CELLV_LOW_UNLATCH_TIME 5
#define PACK_OVERHEAT_ORION_UNLATCH_TIME 5
#define INTERNAL_COMMS_UNLATCH_TIME 5
#define CELL_BALANCING_FOFF_UNLATCH_TIME 5
#define WEAK_CELL_UNLATCH_TIME 5
#define LOW_CELLV_UNLATCH_TIME 5
#define OPEN_WIRE_UNLATCH_TIME 5
#define CURRENT_SENSOR_UNLATCH_TIME 5
#define MAX_CELLV_O5V_UNLATCH_TIME 5
#define CELL_ASIC_UNLATCH_TIME 5
#define WEAK_PACK_UNLATCH_TIME 5
#define FAN_MONITOR_UNLATCH_TIME 5
#define THERMISTOR_UNLATCH_TIME 5
#define EXTERNAL_COMMS_UNLATCH_TIME 5
#define REDUNDANT_PSU_UNLATCH_TIME 5
#define HV_ISOLATION_UNLATCH_TIME 5
#define INPUT_PSU_UNLATCH_TIME 5
#define CHARGE_LIMIT_ENFORCE_UNLATCH_TIME 5
#define PACK_TEMP_UNLATCH_TIME 5000
#define PACK_TEMP_EXCEEDED_UNLATCH_TIME 10000
#define MIN_PACK_TEMP_UNLATCH_TIME 10000
#define IMD_UNLATCH_TIME 5000
#define TMU_POWER_LOST_UNLATCH_TIME 5000
#define TV_DISABLED_UNLATCH_TIME 10
#define TV_UNCALIBRATED_UNLATCH_TIME 10
#define NO_GPS_FIX_UNLATCH_TIME 10
#define TEST_FAULT_1_UNLATCH_TIME 10
#define TEST_FAULT_2_UNLATCH_TIME 10
#define TEST_FAULT_3_UNLATCH_TIME 10
#define TEST_FAULT_4_UNLATCH_TIME 10
//END AUTO UNLATCH DEFS

//BEGIN AUTO SCREENMSG DEFS
#define PDU_MCU_TEMP_HIGH_MSG "HIGH PDU MCU TEMP\0" 
#define DASH_RAIL_MSG "Dash Rail Down\0" 
#define ABOX_RAIL_MSG "ABox Rail Down\0" 
#define MAIN_RAIL_MSG "Main Rail Down\0" 
#define DAQ_RAIL_MSG "DAQ Rail Down\0" 
#define V_CRIT_MSG "5v_crit down\0" 
#define V_NONCRIT_MSG "5V_NC Rail Down\0" 
#define FAN1_MSG "Fan 1 Rail Down\0" 
#define BULLET_RAIL_MSG "Bullet Rail Down\0" 
#define FAN2_MSG "Fan 2 Rail Down\0" 
#define PCHG_IMPLAUS_MSG "Precharge Implausibility\0" 
#define RTD_EXIT_MSG "HV not detected, idling\0" 
#define LEFT_MC_CONN_MSG "LEFT MC CONN FAIL\0" 
#define RIGHT_MC_CONN_MSG "RIGHT MC CONN FAIL\0" 
#define MCU_TEMP_HIGH_MSG "HIGH PDU MCU TEMP\0" 
#define DT_L_TEMP_OT_MSG "Left Gearbox Overheating\0" 
#define DT_R_TEMP_OT_MSG "Right Gearbox Overheating\0" 
#define DT_L_TEMP_HIGH_MSG "Left Gearbox High (>80C)\0" 
#define DT_R_TEMP_HIGH_MSG "Right Gearbox High (>80C)\0" 
#define BAT_COOL_LOOP_HIGH_MSG "Batt Coolant Loop Temp High\0" 
#define DT_COOL_LOOP_HIGH_MSG "DT Coolant Loop Temp High\0" 
#define DT_R_THERM_DISC_MSG "Right DT Therm Disconnect\0" 
#define DT_L_THERM_DISC_MSG "Left DT Therm Disconnect\0" 
#define BATT_CL_DISC_MSG "Batt CL Therm Disc\0" 
#define DT_CL_DISC_MSG "DT CL Therm Disc\0" 
#define BSPD_LATCHED_MSG "BSPD Latched. Restart Car\0" 
#define BOTS_FAIL_MSG "BOTS Activated\0" 
#define INERTIA_FAIL_MSG "Inertia Switch activated\0" 
#define COCKPIT_ESTOP_MSG "Cockpit E-Stop Pressed\0" 
#define RIGHT_ESTOP_MSG "Right E-Stop Pressed\0" 
#define LEFT_ESTOP_MSG "Left E-Stop Pressed\0" 
#define HVD_DISC_MSG "HVD Disconnected\0" 
#define HUB_DISC_MSG "Hub Interlock Disconencted\0" 
#define TSMS_DISC_MSG "TSMS Off\0" 
#define PRECHARGE_TIME_FAULT_MSG "PRCHG time exceeded\0" 
#define MOTOR_R_HEAT_MSG "Right Motor Hot (>80C)\0" 
#define MOTOR_L_HEAT_MSG "Left Motor Hot (>80C)\0" 
#define MOTOR_R_OT_MSG "Right Motor Overtemp (>90C)\0" 
#define MOTOR_L_OT_MSG "Left Motor Overtemp (>90C)\0" 
#define APPS_WIRING_T1_MSG "APPS Wiring Fail T1\0" 
#define APPS_WIRING_T2_MSG "APPS Wiring Fail T2\0" 
#define BSE_MSG "Brake Wiring Fail (BSE)\0" 
#define BSPD_MSG "BSE Wiring Fail B2\0" 
#define IMPLAUS_DETECTED_MSG "APPS Implaus Detected\0" 
#define APPS_BRAKE_MSG "APPS Brake Fault\0" 
#define DISCHARGE_LIMIT_ENFORCE_MSG "Orion Discharge Limit\0" 
#define CHARGER_SAFETY_RELAY_MSG "Orion Charger Safety Error\0" 
#define INTERNAL_HARDWARE_MSG "Orion Internal Fault\0" 
#define HEATSINK_THERMISTOR_MSG "Orion Overheating\0" 
#define SOFTWARE_MSG "Orion Software Error\0" 
#define MAX_CELLV_HIGH_MSG "Max Cell Volts too High\0" 
#define MIN_CELLV_LOW_MSG "Min Cell Volts too Low\0" 
#define PACK_OVERHEAT_ORION_MSG "Orion Pack Overheat Fault\0" 
#define INTERNAL_COMMS_MSG "Orion Internal Comms Error\0" 
#define CELL_BALANCING_FOFF_MSG "Cell Balancing Offline\0" 
#define WEAK_CELL_MSG "Weak Cell Fault\0" 
#define LOW_CELLV_MSG "Low Cell Voltage Fault\0" 
#define OPEN_WIRE_MSG "Orion Open Wire Fault\0" 
#define CURRENT_SENSOR_MSG "Orion Current Sensor Fault\0" 
#define MAX_CELLV_O5V_MSG "Max CellV > 5\0" 
#define CELL_ASIC_MSG "Orion Cell ASIC\0" 
#define WEAK_PACK_MSG "Orion Weak Pack Fault\0" 
#define FAN_MONITOR_MSG "Orion Fan Monitor Fault\0" 
#define THERMISTOR_MSG "Orion Thermistor Fault\0" 
#define EXTERNAL_COMMS_MSG "Orion External Communication Fault\0" 
#define REDUNDANT_PSU_MSG "Orion Redundant PSU Found\0" 
#define HV_ISOLATION_MSG "Orion HV Isolation Fault\0" 
#define INPUT_PSU_MSG "Orion Input PSU Fault\0" 
#define CHARGE_LIMIT_ENFORCE_MSG "Orion Charge Limit\0" 
#define PACK_TEMP_MSG "Pack Temp High (> 50)\0" 
#define PACK_TEMP_EXCEEDED_MSG "Pack Overheating\0" 
#define MIN_PACK_TEMP_MSG "Pack Minimum temp < 10\0" 
#define IMD_MSG "IMD Isolation Fault\0" 
#define TMU_POWER_LOST_MSG "Poopy! TMU Power Lost!\0" 
#define TV_DISABLED_MSG "TV Disabled\0" 
#define TV_UNCALIBRATED_MSG "TV Uncalibrated\0" 
#define NO_GPS_FIX_MSG "No GPS Fix\0" 
#define TEST_FAULT_1_MSG "Test fault 1\0" 
#define TEST_FAULT_2_MSG "Test fault 2\0" 
#define TEST_FAULT_3_MSG "Test fault 3\0" 
#define TEST_FAULT_4_MSG "Test fault 4\0" 
//END AUTO SCREENMSG DEFS

extern uint16_t most_recent_latched;

typedef enum {
    FAULT_WARNING = 0,
    FAULT_ERROR = 1,
    FAULT_FATAL = 2
} fault_priority_t;



//Contains info about fault state
typedef struct {
    bool latched;
    int f_ID;
} fault_status_t;

//Contains info about the fault as a whole
typedef struct {
    bool tempLatch;
    bool forceActive;
    fault_priority_t priority;
    uint8_t bounces;
    uint16_t time_since_latch;
    int f_max;
    int f_min;
    fault_status_t *status;
    uint32_t start_ticks;
    char* screen_MSG;
} fault_attributes_t;

extern fault_attributes_t faultArray[TOTAL_NUM_FAULTS];

//Union to package CAN messages
typedef union {
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync;
    uint8_t raw_data[8];
} __attribute__((packed)) fault_can_format_t;




//Vars
extern fault_status_t message[TOTAL_NUM_FAULTS];
extern fault_attributes_t attributes[TOTAL_NUM_FAULTS];

//Function defs
void initFaultLibrary(uint8_t mcu, q_handle_t* txQ, uint32_t ext);
bool setFault(int, int);
static void forceFault(int id, bool state);
static void unForce(int);
static void txFaultSpecific(int);
bool updateFault(uint16_t idx);
void heartBeatTask();
void updateFaults();
void killFaultLibrary();
void handleCallbacks(uint16_t id, bool latched);
bool currMCULatched();
bool warningLatched();
bool errorLatched();
bool fatalLatched();
bool otherMCUsLatched();
bool isLatched();
bool checkFault(int id);

#endif