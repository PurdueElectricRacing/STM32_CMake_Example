/**
 * @file can_parse.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _CAN_PARSE_H_
#define _CAN_PARSE_H_

#include "common/queue/queue.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/daq/can_parse_base.h"

// Make this match the node name within the can_config.json
#define NODE_NAME "torque_vector"

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_GPS_VELOCITY 0xc0002b7
#define ID_GPS_SPEED 0xc001137
#define ID_GPS_POSITION 0xc002337
#define ID_GPS_COORDINATES 0xc002377
#define ID_IMU_GYRO 0xc0002f7
#define ID_IMU_ACCEL 0xc0023b7
#define ID_BMM_MAG 0xc0023f7
#define ID_SFS_VEL 0xc016977
#define ID_SFS_ACC 0xc0169b7
#define ID_SFS_ANG_VEL 0xc016a37
#define ID_THROTTLE_VCU 0x40025b7
#define ID_THROTTLE_VCU_EQUAL 0x4002837
#define ID_MAXR 0xc002637
#define ID_TV_CAN_STATS 0x10016337
#define ID_FAULT_SYNC_TORQUE_VECTOR 0x8cab7
#define ID_TORQUEVECTOR_BL_CMD 0x409c4be
#define ID_FILT_THROTTLE_BRAKE 0x4000245
#define ID_LWS_STANDARD 0x2b0
#define ID_ORION_CURRENTS_VOLTS 0x140006f8
#define ID_DASHBOARD_TV_PARAMETERS 0x4000dc5
#define ID_MAIN_HB 0xc001901
#define ID_REAR_WHEEL_SPEEDS 0x4000381
#define ID_REAR_MOTOR_TEMPS 0x10000301
#define ID_MAX_CELL_TEMP 0xc04e604
#define ID_FAULT_SYNC_PDU 0x8cb1f
#define ID_FAULT_SYNC_MAIN_MODULE 0x8ca01
#define ID_FAULT_SYNC_DASHBOARD 0x8cac5
#define ID_FAULT_SYNC_A_BOX 0x8ca44
#define ID_FAULT_SYNC_TEST_NODE 0x8cb7f
#define ID_SET_FAULT 0x809c83e
#define ID_RETURN_FAULT_CONTROL 0x809c87e
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_GPS_VELOCITY 6
#define DLC_GPS_SPEED 4
#define DLC_GPS_POSITION 2
#define DLC_GPS_COORDINATES 8
#define DLC_IMU_GYRO 6
#define DLC_IMU_ACCEL 6
#define DLC_BMM_MAG 6
#define DLC_SFS_VEL 6
#define DLC_SFS_ACC 6
#define DLC_SFS_ANG_VEL 6
#define DLC_THROTTLE_VCU 4
#define DLC_THROTTLE_VCU_EQUAL 4
#define DLC_MAXR 2
#define DLC_TV_CAN_STATS 4
#define DLC_FAULT_SYNC_TORQUE_VECTOR 3
#define DLC_TORQUEVECTOR_BL_CMD 5
#define DLC_FILT_THROTTLE_BRAKE 3
#define DLC_LWS_STANDARD 5
#define DLC_ORION_CURRENTS_VOLTS 4
#define DLC_DASHBOARD_TV_PARAMETERS 7
#define DLC_MAIN_HB 2
#define DLC_REAR_WHEEL_SPEEDS 8
#define DLC_REAR_MOTOR_TEMPS 4
#define DLC_MAX_CELL_TEMP 2
#define DLC_FAULT_SYNC_PDU 3
#define DLC_FAULT_SYNC_MAIN_MODULE 3
#define DLC_FAULT_SYNC_DASHBOARD 3
#define DLC_FAULT_SYNC_A_BOX 3
#define DLC_FAULT_SYNC_TEST_NODE 3
#define DLC_SET_FAULT 3
#define DLC_RETURN_FAULT_CONTROL 2
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_GPS_VELOCITY(gps_vel_n_, gps_vel_e_, gps_vel_d_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_VELOCITY, .DLC=DLC_GPS_VELOCITY, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_velocity.gps_vel_n = gps_vel_n_;\
        data_a->gps_velocity.gps_vel_e = gps_vel_e_;\
        data_a->gps_velocity.gps_vel_d = gps_vel_d_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_GPS_SPEED(gps_speed_, gps_heading_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_SPEED, .DLC=DLC_GPS_SPEED, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_speed.gps_speed = gps_speed_;\
        data_a->gps_speed.gps_heading = gps_heading_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_GPS_POSITION(height_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_POSITION, .DLC=DLC_GPS_POSITION, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_position.height = height_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_GPS_COORDINATES(latitude_, longitude_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_GPS_COORDINATES, .DLC=DLC_GPS_COORDINATES, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->gps_coordinates.latitude = latitude_;\
        data_a->gps_coordinates.longitude = longitude_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_IMU_GYRO(imu_gyro_x_, imu_gyro_y_, imu_gyro_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_IMU_GYRO, .DLC=DLC_IMU_GYRO, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->imu_gyro.imu_gyro_x = imu_gyro_x_;\
        data_a->imu_gyro.imu_gyro_y = imu_gyro_y_;\
        data_a->imu_gyro.imu_gyro_z = imu_gyro_z_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_IMU_ACCEL(imu_accel_x_, imu_accel_y_, imu_accel_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_IMU_ACCEL, .DLC=DLC_IMU_ACCEL, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->imu_accel.imu_accel_x = imu_accel_x_;\
        data_a->imu_accel.imu_accel_y = imu_accel_y_;\
        data_a->imu_accel.imu_accel_z = imu_accel_z_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_BMM_MAG(bmm_mag_x_, bmm_mag_y_, bmm_mag_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_BMM_MAG, .DLC=DLC_BMM_MAG, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->bmm_mag.bmm_mag_x = bmm_mag_x_;\
        data_a->bmm_mag.bmm_mag_y = bmm_mag_y_;\
        data_a->bmm_mag.bmm_mag_z = bmm_mag_z_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_SFS_VEL(sfs_vel_x_, sfs_vel_y_, sfs_vel_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_VEL, .DLC=DLC_SFS_VEL, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_vel.sfs_vel_x = sfs_vel_x_;\
        data_a->sfs_vel.sfs_vel_y = sfs_vel_y_;\
        data_a->sfs_vel.sfs_vel_z = sfs_vel_z_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_SFS_ACC(sfs_acc_x_, sfs_acc_y_, sfs_acc_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_ACC, .DLC=DLC_SFS_ACC, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_acc.sfs_acc_x = sfs_acc_x_;\
        data_a->sfs_acc.sfs_acc_y = sfs_acc_y_;\
        data_a->sfs_acc.sfs_acc_z = sfs_acc_z_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_SFS_ANG_VEL(sfs_ang_vel_x_, sfs_ang_vel_y_, sfs_ang_vel_z_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_SFS_ANG_VEL, .DLC=DLC_SFS_ANG_VEL, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->sfs_ang_vel.sfs_ang_vel_x = sfs_ang_vel_x_;\
        data_a->sfs_ang_vel.sfs_ang_vel_y = sfs_ang_vel_y_;\
        data_a->sfs_ang_vel.sfs_ang_vel_z = sfs_ang_vel_z_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_THROTTLE_VCU(vcu_k_rl_, vcu_k_rr_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_THROTTLE_VCU, .DLC=DLC_THROTTLE_VCU, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->throttle_vcu.vcu_k_rl = vcu_k_rl_;\
        data_a->throttle_vcu.vcu_k_rr = vcu_k_rr_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_THROTTLE_VCU_EQUAL(equal_k_rl_, equal_k_rr_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_THROTTLE_VCU_EQUAL, .DLC=DLC_THROTTLE_VCU_EQUAL, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->throttle_vcu_equal.equal_k_rl = equal_k_rl_;\
        data_a->throttle_vcu_equal.equal_k_rr = equal_k_rr_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_MAXR(vcu_max_r_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_MAXR, .DLC=DLC_MAXR, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->maxR.vcu_max_r = vcu_max_r_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_TV_CAN_STATS(can_tx_overflow_, can_tx_fail_, can_rx_overflow_, can_rx_overrun_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_TV_CAN_STATS, .DLC=DLC_TV_CAN_STATS, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->tv_can_stats.can_tx_overflow = can_tx_overflow_;\
        data_a->tv_can_stats.can_tx_fail = can_tx_fail_;\
        data_a->tv_can_stats.can_rx_overflow = can_rx_overflow_;\
        data_a->tv_can_stats.can_rx_overrun = can_rx_overrun_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_FAULT_SYNC_TORQUE_VECTOR(idx_, latched_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_FAULT_SYNC_TORQUE_VECTOR, .DLC=DLC_FAULT_SYNC_TORQUE_VECTOR, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->fault_sync_torque_vector.idx = idx_;\
        data_a->fault_sync_torque_vector.latched = latched_;\
        canTxSendToBack(&msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 5 / 2 // 5 / 2 would be 250% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
#define UP_FILT_THROTTLE_BRAKE 15
#define UP_LWS_STANDARD 15
#define UP_ORION_CURRENTS_VOLTS 32
#define UP_DASHBOARD_TV_PARAMETERS 500
#define UP_MAIN_HB 500
#define UP_REAR_WHEEL_SPEEDS 15
#define UP_REAR_MOTOR_TEMPS 1000
#define UP_MAX_CELL_TEMP 500
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) \
    if (!stale &&                              \
        (curr - last) > period * STALE_THRESH) \
    stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
typedef enum {
    CAR_STATE_IDLE,
    CAR_STATE_PRECHARGING,
    CAR_STATE_ENERGIZED,
    CAR_STATE_BUZZING,
    CAR_STATE_READY2DRIVE,
    CAR_STATE_ERROR,
    CAR_STATE_FATAL,
    CAR_STATE_RESET,
    CAR_STATE_RECOVER,
    CAR_STATE_CONSTANT_TORQUE,
} car_state_t;

/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { 
    struct {
        uint64_t gps_vel_n: 16;
        uint64_t gps_vel_e: 16;
        uint64_t gps_vel_d: 16;
    } gps_velocity;
    struct {
        uint64_t gps_speed: 16;
        uint64_t gps_heading: 16;
    } gps_speed;
    struct {
        uint64_t height: 16;
    } gps_position;
    struct {
        uint64_t latitude: 32;
        uint64_t longitude: 32;
    } gps_coordinates;
    struct {
        uint64_t imu_gyro_x: 16;
        uint64_t imu_gyro_y: 16;
        uint64_t imu_gyro_z: 16;
    } imu_gyro;
    struct {
        uint64_t imu_accel_x: 16;
        uint64_t imu_accel_y: 16;
        uint64_t imu_accel_z: 16;
    } imu_accel;
    struct {
        uint64_t bmm_mag_x: 16;
        uint64_t bmm_mag_y: 16;
        uint64_t bmm_mag_z: 16;
    } bmm_mag;
    struct {
        uint64_t sfs_vel_x: 16;
        uint64_t sfs_vel_y: 16;
        uint64_t sfs_vel_z: 16;
    } sfs_vel;
    struct {
        uint64_t sfs_acc_x: 16;
        uint64_t sfs_acc_y: 16;
        uint64_t sfs_acc_z: 16;
    } sfs_acc;
    struct {
        uint64_t sfs_ang_vel_x: 16;
        uint64_t sfs_ang_vel_y: 16;
        uint64_t sfs_ang_vel_z: 16;
    } sfs_ang_vel;
    struct {
        uint64_t vcu_k_rl: 16;
        uint64_t vcu_k_rr: 16;
    } throttle_vcu;
    struct {
        uint64_t equal_k_rl: 16;
        uint64_t equal_k_rr: 16;
    } throttle_vcu_equal;
    struct {
        uint64_t vcu_max_r: 16;
    } maxR;
    struct {
        uint64_t can_tx_overflow: 8;
        uint64_t can_tx_fail: 8;
        uint64_t can_rx_overflow: 8;
        uint64_t can_rx_overrun: 8;
    } tv_can_stats;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_torque_vector;
    struct {
        uint64_t cmd: 8;
        uint64_t data: 32;
    } torquevector_bl_cmd;
    struct {
        uint64_t throttle: 12;
        uint64_t brake: 12;
    } filt_throttle_brake;
    struct {
        uint64_t LWS_ANGLE: 16;
        uint64_t LWS_SPEED: 8;
        uint64_t Ok: 1;
        uint64_t Cal: 1;
        uint64_t Trim: 1;
        uint64_t Reserved_1: 5;
        uint64_t Reserved_2: 8;
    } LWS_Standard;
    struct {
        uint64_t pack_current: 16;
        uint64_t pack_voltage: 16;
    } orion_currents_volts;
    struct {
        uint64_t tv_enabled: 1;
        uint64_t tv_deadband_val: 16;
        uint64_t tv_intensity_val: 16;
        uint64_t tv_p_val: 16;
    } dashboard_tv_parameters;
    struct {
        uint64_t car_state: 8;
        uint64_t precharge_state: 1;
    } main_hb;
    struct {
        uint64_t left_speed_mc: 16;
        uint64_t right_speed_mc: 16;
        uint64_t left_speed_sensor: 16;
        uint64_t right_speed_sensor: 16;
    } rear_wheel_speeds;
    struct {
        uint64_t left_mot_temp: 8;
        uint64_t right_mot_temp: 8;
        uint64_t left_ctrl_temp: 8;
        uint64_t right_ctrl_temp: 8;
    } rear_motor_temps;
    struct {
        uint64_t max_temp: 16;
    } max_cell_temp;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_pdu;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_main_module;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_dashboard;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_a_box;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_test_node;
    struct {
        uint64_t id: 16;
        uint64_t value: 1;
    } set_fault;
    struct {
        uint64_t id: 16;
    } return_fault_control;
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint8_t cmd;
        uint32_t data;
    } torquevector_bl_cmd;
    struct {
        uint16_t throttle;
        uint16_t brake;
        uint8_t stale;
        uint32_t last_rx;
    } filt_throttle_brake;
    struct {
        int16_t LWS_ANGLE;
        uint8_t LWS_SPEED;
        uint8_t Ok;
        uint8_t Cal;
        uint8_t Trim;
        uint8_t Reserved_1;
        uint8_t Reserved_2;
        uint8_t stale;
        uint32_t last_rx;
    } LWS_Standard;
    struct {
        int16_t pack_current;
        uint16_t pack_voltage;
        uint8_t stale;
        uint32_t last_rx;
    } orion_currents_volts;
    struct {
        uint8_t tv_enabled;
        uint16_t tv_deadband_val;
        uint16_t tv_intensity_val;
        uint16_t tv_p_val;
        uint8_t stale;
        uint32_t last_rx;
    } dashboard_tv_parameters;
    struct {
        car_state_t car_state;
        uint8_t precharge_state;
        uint8_t stale;
        uint32_t last_rx;
    } main_hb;
    struct {
        uint16_t left_speed_mc;
        uint16_t right_speed_mc;
        uint16_t left_speed_sensor;
        uint16_t right_speed_sensor;
        uint8_t stale;
        uint32_t last_rx;
    } rear_wheel_speeds;
    struct {
        uint8_t left_mot_temp;
        uint8_t right_mot_temp;
        uint8_t left_ctrl_temp;
        uint8_t right_ctrl_temp;
        uint8_t stale;
        uint32_t last_rx;
    } rear_motor_temps;
    struct {
        int16_t max_temp;
        uint8_t stale;
        uint32_t last_rx;
    } max_cell_temp;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_pdu;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_main_module;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_dashboard;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_a_box;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_test_node;
    struct {
        uint16_t id;
        uint8_t value;
    } set_fault;
    struct {
        uint16_t id;
    } return_fault_control;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
extern void torquevector_bl_cmd_CALLBACK(CanParsedData_t* msg_data_a);
extern void handleCallbacks(uint16_t id, bool latched);
extern void set_fault_daq(uint16_t id, bool value);
extern void return_fault_control(uint16_t id);
extern void send_fault(uint16_t id, bool latched);
/* END AUTO EXTERN CALLBACK */

/* BEGIN AUTO EXTERN RX IRQ */
/* END AUTO EXTERN RX IRQ */

/**
 * @brief Setup queue and message filtering
 *
 * @param q_rx_can RX buffer of CAN messages
 */
void initCANParse(void);

/**
 * @brief Pull message off of rx buffer,
 *        update can_data struct,
 *        check for stale messages
 */
void canRxUpdate();

/**
 * @brief Process any rx message callbacks from the CAN Rx IRQ
 *
 * @param rx rx data from message just recieved
 */
void canProcessRxIRQs(CanMsgTypeDef_t *rx);

extern volatile uint32_t last_can_rx_time_ms;

#endif