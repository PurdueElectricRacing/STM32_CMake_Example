/**
 * @file can_parse.h
 * @author Cole Roberts (rober638@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2024-09-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _CAN_PARSE_H_
#define _CAN_PARSE_H_

#include "common/queue/queue.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/daq/can_parse_base.h"

// Make this match the node name within the can_config.json
#define NODE_NAME "Inverter1"


// Used to represent a float as 32 bits
typedef union {
    float f;
    uint32_t u;
} FloatConvert_t;
#define FLOAT_TO_UINT32(float_) (((FloatConvert_t) float_).u)
#define UINT32_TO_FLOAT(uint32_) (((FloatConvert_t) ((uint32_t) uint32_)).f)

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_AMK_ACTUAL_VALUES_1 0x282
#define ID_AMK_ACTUAL_VALUES_2 0x284
#define ID_AMK_TEMPERATURES_1 0x286
#define ID_AMK_TEMPERATURES_2 0x288
#define ID_AMK_SETPOINTS 0x182
#define ID_AMK_TESTING 0x384
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_AMK_ACTUAL_VALUES_1 8
#define DLC_AMK_ACTUAL_VALUES_2 8
#define DLC_AMK_TEMPERATURES_1 6
#define DLC_AMK_TEMPERATURES_2 6
#define DLC_AMK_SETPOINTS 8
#define DLC_AMK_TESTING 6
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_AMK_ACTUAL_VALUES_1(AMK_Status_, AMK_ActualTorque_, AMK_MotorSerialNumber_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_AMK_ACTUAL_VALUES_1, .DLC=DLC_AMK_ACTUAL_VALUES_1, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->AMK_Actual_Values_1.AMK_Status = AMK_Status_;\
        data_a->AMK_Actual_Values_1.AMK_ActualTorque = AMK_ActualTorque_;\
        data_a->AMK_Actual_Values_1.AMK_MotorSerialNumber = AMK_MotorSerialNumber_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_AMK_ACTUAL_VALUES_2(AMK_ActualSpeed_, AMK_DCBusVoltage_, AMK_SystemReset_, AMK_DiagnosticNumber_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_AMK_ACTUAL_VALUES_2, .DLC=DLC_AMK_ACTUAL_VALUES_2, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->AMK_Actual_Values_2.AMK_ActualSpeed = AMK_ActualSpeed_;\
        data_a->AMK_Actual_Values_2.AMK_DCBusVoltage = AMK_DCBusVoltage_;\
        data_a->AMK_Actual_Values_2.AMK_SystemReset = AMK_SystemReset_;\
        data_a->AMK_Actual_Values_2.AMK_DiagnosticNumber = AMK_DiagnosticNumber_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_AMK_TEMPERATURES_1(AMK_MotorTemp_, AMK_InverterTemp_, AMK_IGBTTemp_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_AMK_TEMPERATURES_1, .DLC=DLC_AMK_TEMPERATURES_1, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->AMK_Temperatures_1.AMK_MotorTemp = AMK_MotorTemp_;\
        data_a->AMK_Temperatures_1.AMK_InverterTemp = AMK_InverterTemp_;\
        data_a->AMK_Temperatures_1.AMK_IGBTTemp = AMK_IGBTTemp_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_AMK_TEMPERATURES_2(AMK_InternalTemp_, AMK_ExternalTemp_, AMK_TempSensorMotor_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_AMK_TEMPERATURES_2, .DLC=DLC_AMK_TEMPERATURES_2, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->AMK_Temperatures_2.AMK_InternalTemp = AMK_InternalTemp_;\
        data_a->AMK_Temperatures_2.AMK_ExternalTemp = AMK_ExternalTemp_;\
        data_a->AMK_Temperatures_2.AMK_TempSensorMotor = AMK_TempSensorMotor_;\
        canTxSendToBack(&msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 30 / 2 // 5 / 2 would be 250% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
#define UP_AMK_SETPOINTS 5
#define UP_AMK_TESTING 5
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) if(!stale && \
                    (curr - last) > period * STALE_THRESH) stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { 
    struct {
        uint64_t AMK_Status: 16;
        uint64_t AMK_ActualTorque: 16;
        uint64_t AMK_MotorSerialNumber: 32;
    } AMK_Actual_Values_1;
    struct {
        uint64_t AMK_ActualSpeed: 16;
        uint64_t AMK_DCBusVoltage: 16;
        uint64_t AMK_SystemReset: 16;
        uint64_t AMK_DiagnosticNumber: 16;
    } AMK_Actual_Values_2;
    struct {
        uint64_t AMK_MotorTemp: 16;
        uint64_t AMK_InverterTemp: 16;
        uint64_t AMK_IGBTTemp: 16;
    } AMK_Temperatures_1;
    struct {
        uint64_t AMK_InternalTemp: 16;
        uint64_t AMK_ExternalTemp: 16;
        uint64_t AMK_TempSensorMotor: 16;
    } AMK_Temperatures_2;
    struct {
        uint64_t AMK_Control: 16;
        uint64_t AMK_TorqueSetpoint: 16;
        uint64_t AMK_PositiveTorqueLimit: 16;
        uint64_t AMK_NegativeTorqueLimit: 16;
    } AMK_Setpoints;
    struct {
        uint64_t AMK_InitStage: 8;
        uint64_t AMK_Control: 16;
        uint64_t AMK_Status_from_motor: 16;
        uint64_t precharge: 8;
    } AMK_Testing;
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint16_t AMK_Control;
        int16_t AMK_TorqueSetpoint;
        int16_t AMK_PositiveTorqueLimit;
        int16_t AMK_NegativeTorqueLimit;
        uint8_t stale;
        uint32_t last_rx;
    } AMK_Setpoints;
    struct {
        uint8_t AMK_InitStage;
        uint16_t AMK_Control;
        uint16_t AMK_Status_from_motor;
        uint8_t precharge;
        uint8_t stale;
        uint32_t last_rx;
    } AMK_Testing;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;
extern volatile uint32_t last_can_rx_time_ms;

/* BEGIN AUTO EXTERN CALLBACK */
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
void canRxUpdate(void);

/**
 * @brief Process any rx message callbacks from the CAN Rx IRQ
 *
 * @param rx rx data from message just recieved
 */
void canProcessRxIRQs(CanMsgTypeDef_t* rx);

#endif
