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
#define ID_INV1_ACTUAL_VALUES_1 0x283
#define ID_INV1_ACTUAL_VALUES_2 0x285
#define ID_INV1_SETPOINTS_1 0x184
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_INV1_ACTUAL_VALUES_1 8
#define DLC_INV1_ACTUAL_VALUES_2 8
#define DLC_INV1_SETPOINTS_1 8
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_INV1_ACTUAL_VALUES_1(AMK_Status_bReserve_, AMK_Status_bSystemReady_, AMK_Status_bError_, AMK_Status_bWarn_, AMK_Status_bQuitDcOn_, AMK_Status_bDcOn_, AMK_Status_bQuitInverterOn_, AMK_Status_bInverterOn_, AMK_Status_bDerating_, AMK_ActualVelocity_, AMK_TorqueCurrent_, AMK_MagnetizingCurrent_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_INV1_ACTUAL_VALUES_1, .DLC=DLC_INV1_ACTUAL_VALUES_1, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->INV1_Actual_Values_1.AMK_Status_bReserve = AMK_Status_bReserve_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bSystemReady = AMK_Status_bSystemReady_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bError = AMK_Status_bError_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bWarn = AMK_Status_bWarn_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bQuitDcOn = AMK_Status_bQuitDcOn_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bDcOn = AMK_Status_bDcOn_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bQuitInverterOn = AMK_Status_bQuitInverterOn_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bInverterOn = AMK_Status_bInverterOn_;\
        data_a->INV1_Actual_Values_1.AMK_Status_bDerating = AMK_Status_bDerating_;\
        data_a->INV1_Actual_Values_1.AMK_ActualVelocity = AMK_ActualVelocity_;\
        data_a->INV1_Actual_Values_1.AMK_TorqueCurrent = AMK_TorqueCurrent_;\
        data_a->INV1_Actual_Values_1.AMK_MagnetizingCurrent = AMK_MagnetizingCurrent_;\
        canTxSendToBack(&msg);\
    } while(0)
#define SEND_INV1_ACTUAL_VALUES_2(AMK_TempMotor_, AMK_TempInverter_, AMK_ErrorInfo_, AMK_TempIGBT_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_INV1_ACTUAL_VALUES_2, .DLC=DLC_INV1_ACTUAL_VALUES_2, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->INV1_Actual_Values_2.AMK_TempMotor = AMK_TempMotor_;\
        data_a->INV1_Actual_Values_2.AMK_TempInverter = AMK_TempInverter_;\
        data_a->INV1_Actual_Values_2.AMK_ErrorInfo = AMK_ErrorInfo_;\
        data_a->INV1_Actual_Values_2.AMK_TempIGBT = AMK_TempIGBT_;\
        canTxSendToBack(&msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 30 / 2 // 5 / 2 would be 250% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
#define UP_INV1_SETPOINTS_1 5
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) if(!stale && \
                    (curr - last) > period * STALE_THRESH) stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { 
    struct {
        uint64_t AMK_Status_bReserve: 8;
        uint64_t AMK_Status_bSystemReady: 1;
        uint64_t AMK_Status_bError: 1;
        uint64_t AMK_Status_bWarn: 1;
        uint64_t AMK_Status_bQuitDcOn: 1;
        uint64_t AMK_Status_bDcOn: 1;
        uint64_t AMK_Status_bQuitInverterOn: 1;
        uint64_t AMK_Status_bInverterOn: 1;
        uint64_t AMK_Status_bDerating: 1;
        uint64_t AMK_ActualVelocity: 16;
        uint64_t AMK_TorqueCurrent: 16;
        uint64_t AMK_MagnetizingCurrent: 16;
    } INV1_Actual_Values_1;
    struct {
        uint64_t AMK_TempMotor: 16;
        uint64_t AMK_TempInverter: 16;
        uint64_t AMK_ErrorInfo: 16;
        uint64_t AMK_TempIGBT: 16;
    } INV1_Actual_Values_2;
    struct {
        uint64_t AMK_Control_bReserve: 8;
        uint64_t AMK_Control_bInverterOn: 1;
        uint64_t AMK_Control_bDcOn: 1;
        uint64_t AMK_Control_bEnable: 1;
        uint64_t AMK_Control_bErrorReset: 1;
        uint64_t AMK_Control_bReserve2: 4;
        uint64_t AMK_TargetVelocity: 16;
        uint64_t AMK_TorqueLimitPositiv: 16;
        uint64_t AMK_TorqueLimitNegativ: 16;
    } INV1_Setpoints_1;
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint8_t AMK_Control_bReserve;
        uint8_t AMK_Control_bInverterOn;
        uint8_t AMK_Control_bDcOn;
        uint8_t AMK_Control_bEnable;
        uint8_t AMK_Control_bErrorReset;
        uint8_t AMK_Control_bReserve2;
        int16_t AMK_TargetVelocity;
        int16_t AMK_TorqueLimitPositiv;
        int16_t AMK_TorqueLimitNegativ;
        uint8_t stale;
        uint32_t last_rx;
    } INV1_Setpoints_1;
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
