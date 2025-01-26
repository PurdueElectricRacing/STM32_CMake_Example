#ifndef __DAQ_CAN_H__
#define __DAQ_CAN_H__

#include <stdint.h>
#include "can_parse.h"

typedef struct {
    uint32_t tx_of;      // queue overflow
    uint32_t tx_fail;    // timed out
    uint32_t rx_of;      // queue overflow
    uint32_t rx_overrun; // fifo overrun
} can_stats_t;

// 0 = CAN1, 1 = CAN2,
#define BUS_ID_CAN1 0
#define BUS_ID_CAN2 1

#define CAN_BUS_COUNT      2
#define CAN_TX_MAILBOX_CNT 3
#define CAN_TX_TIMEOUT_MS 15

extern can_stats_t can_stats[CAN_BUS_COUNT];
extern q_handle_t q_tx_can1_s[CAN_BUS_COUNT][CAN_TX_MAILBOX_CNT];

void initCANParse();
void canTxSendToBack(CanMsgTypeDef_t *msg);
void canTxUpdate(void);

#endif // __DAQ_CAN_H__
