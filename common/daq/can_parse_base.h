/**
 * @file can_parse_base.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief CAN Priority-Based Queuing and Diagnostics
 * @version 0.1
 * @date 2024-04-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _CAN_PARSE_BASE_H_
#define _CAN_PARSE_BASE_H_

#if defined(STM32F407xx)
#include "common/phal_F4_F7/can/can.h"
#elif defined(STM32F732xx)
#include "common/phal_F4_F7/can/can.h"
#else
#include "common/phal_L4/can/can.h"
#endif
#include "common/queue/queue.h"
#include "common/psched/psched.h"

typedef struct {
    uint32_t tx_of;      // queue overflow
    uint32_t tx_fail;    // timed out
    uint32_t rx_of;      // queue overflow
    uint32_t rx_overrun; // fifo overrun
} can_stats_t;

#define CAN_TX_MAILBOX_CNT 3
#define CAN_TX_TIMEOUT_MS 15

extern can_stats_t can_stats;
extern q_handle_t q_tx_can1_s[CAN_TX_MAILBOX_CNT ];
extern q_handle_t q_tx_can2_s[CAN_TX_MAILBOX_CNT ];
extern q_handle_t q_rx_can;

void initCANParseBase();
void canTxUpdate(void);
void canParseIRQHandler(CAN_TypeDef *can_h);
void canTxSendToBack(CanMsgTypeDef_t *msg);

#endif
