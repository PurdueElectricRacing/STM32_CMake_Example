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
#include "common/phal_L4/can/can.h"

// Make this match the node name within the can_config.json
#define NODE_NAME "TEST_NODE"

#define RX_UPDATE_PERIOD 15 // ms

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_TEST_MSG 0x1400007f
#define ID_TEST_MSG2 0x140000bf
#define ID_TEST_MSG3 0x140000ff
#define ID_TEST_MSG4 0x1400013f
#define ID_TEST_MSG5 0x1400017f
#define ID_DAQ_RESPONSE_TEST_NODE 0x17ffffff
#define ID_RAW_THROTTLE_BRAKE 0x14000285
#define ID_DAQ_COMMAND_TEST_NODE 0x14000ff2
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_TEST_MSG 2
#define DLC_TEST_MSG2 2
#define DLC_TEST_MSG3 2
#define DLC_TEST_MSG4 2
#define DLC_TEST_MSG5 2
#define DLC_DAQ_RESPONSE_TEST_NODE 8
#define DLC_RAW_THROTTLE_BRAKE 8
#define DLC_DAQ_COMMAND_TEST_NODE 8
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_TEST_MSG(queue, test_sig_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_TEST_MSG, .DLC=DLC_TEST_MSG, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->test_msg.test_sig = test_sig_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_TEST_MSG2(queue, test_sig2_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_TEST_MSG2, .DLC=DLC_TEST_MSG2, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->test_msg2.test_sig2 = test_sig2_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_TEST_MSG3(queue, test_sig3_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_TEST_MSG3, .DLC=DLC_TEST_MSG3, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->test_msg3.test_sig3 = test_sig3_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_TEST_MSG4(queue, test_sig4_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_TEST_MSG4, .DLC=DLC_TEST_MSG4, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->test_msg4.test_sig4 = test_sig4_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_TEST_MSG5(queue, test_sig5_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_TEST_MSG5, .DLC=DLC_TEST_MSG5, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->test_msg5.test_sig5 = test_sig5_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_DAQ_RESPONSE_TEST_NODE(queue, daq_response_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_DAQ_RESPONSE_TEST_NODE, .DLC=DLC_DAQ_RESPONSE_TEST_NODE, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->daq_response_TEST_NODE.daq_response = daq_response_;\
        qSendToBack(&queue, &msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 3 / 2 // 3 / 2 would be 150% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
#define UP_RAW_THROTTLE_BRAKE 5
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) if(!stale && \
                    (curr - last) * RX_UPDATE_PERIOD > period * STALE_THRESH) stale = 1

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { __attribute__((packed))
    struct {
        uint64_t test_sig: 16;
    } test_msg;
    struct {
        uint64_t test_sig2: 16;
    } test_msg2;
    struct {
        uint64_t test_sig3: 16;
    } test_msg3;
    struct {
        uint64_t test_sig4: 16;
    } test_msg4;
    struct {
        uint64_t test_sig5: 16;
    } test_msg5;
    struct {
        uint64_t daq_response: 64;
    } daq_response_TEST_NODE;
    struct {
        uint64_t throttle0: 16;
        uint64_t throttle1: 16;
        uint64_t brake0: 16;
        uint64_t brake1: 16;
    } raw_throttle_brake;
    struct {
        uint64_t daq_command: 64;
    } daq_command_TEST_NODE;
    uint8_t raw_data[8];
} CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint16_t throttle0;
        uint16_t throttle1;
        uint16_t brake0;
        uint16_t brake1;
        uint8_t stale;
        uint32_t last_rx;
    } raw_throttle_brake;
    struct {
        uint64_t daq_command;
    } daq_command_TEST_NODE;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
extern void daq_command_TEST_NODE_CALLBACK(CanMsgTypeDef_t* msg_header_a);
/* END AUTO EXTERN CALLBACK */

/* BEGIN AUTO EXTERN RX IRQ */
/* END AUTO EXTERN RX IRQ */

/**
 * @brief Setup queue and message filtering
 * 
 * @param q_rx_can RX buffer of CAN messages
 */
void initCANParse(q_handle_t* q_rx_can_a);

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
void canProcessRxIRQs(CanMsgTypeDef_t* rx);

#endif