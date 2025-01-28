
#include <string.h>

#include "common/phal_F4_F7/can/can.h"
#include "common/freertos/freertos.h"
#include "common/queue/queue.h"
#include "daq_can.h"
#include "daq_hub.h"

// TODO move to freertos queues

can_stats_t can_stats[CAN_BUS_COUNT];
q_handle_t q_tx_can1_s[CAN_BUS_COUNT][CAN_TX_MAILBOX_CNT];
static uint32_t mbx_last_send_time[CAN_BUS_COUNT][CAN_TX_MAILBOX_CNT];

#define CAN_TX_BLOCK_TIMEOUT (30 * 16000) // clock rate 16MHz, 15ms * 16000 cyc / ms

static void initCANParseBase()
{
    #if 0
    for (int bus_id = 0; bus_id < CAN_BUS_COUNT; bus_id++)
    {
        for (int j = 0; j < CAN_TX_MAILBOX_CNT; j++)
        {
            qConstruct(&q_tx_can1_s[bus_id][j], sizeof(CanMsgTypeDef_t));
            mbx_last_send_time[bus_id][j] = 0;
        }
    }
    #endif
    memset(can_stats, 0, sizeof(can_stats) * sizeof(*can_stats));
}

void initCANParse()
{
    initCANParseBase();
    //initCANFilter(); // !!! NO CAN FILTER FOR DAQ !!!!!!!!!!!!!
}

void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    #if 0
    q_handle_t *qh;
    int bus_id = (msg->Bus == CAN1) ? BUS_ID_CAN1 : BUS_ID_CAN2;
    if (msg->IDE == 1)
    {
        // extended id, check hlp
        switch((msg->ExtId >> 26) & 0b111)
        {
            case 0:
            case 1:
                qh = &q_tx_can1_s[bus_id][0];
                break;
            case 2:
            case 3:
                qh = &q_tx_can1_s[bus_id][1];
                break;
            default:
                qh = &q_tx_can1_s[bus_id][2];
                break;
        }
    }
    else
    {
        qh = &q_tx_can1_s[bus_id][0]; // IDE = 0 doesn't have an HLP
    }
    if (qSendToBack(qh, msg) != SUCCESS_G)
    {
        daq_catch_error();
        can_stats[bus_id].tx_of++;
    }
    #else
    /* Queues dont work for large loads like bootloader, so just send it now */
    #if 0
    uint32_t t = 0;
    for (int mbox = 0; mbox < CAN_TX_MAILBOX_CNT; mbox++)
    {
        if (PHAL_txMailboxFree(CAN1, mbox))
        {
            PHAL_txCANMessage(msg, mbox);
            return;
        }
    }
    while (!PHAL_txMailboxFree(CAN1, 0) && (t++ < CAN_TX_BLOCK_TIMEOUT));
    if (t < CAN_TX_BLOCK_TIMEOUT) PHAL_txCANMessage(msg, 0);
    else
    {
        daq_catch_error();
    }
    #endif
    uint32_t t = 0;
    /* Don't use multiple mailboxes to guarantee in-order transmission */
    while (!PHAL_txMailboxFree(CAN1, 0) && (t++ < CAN_TX_BLOCK_TIMEOUT));
    if (t < CAN_TX_BLOCK_TIMEOUT) PHAL_txCANMessage(msg, 0);
    else
    {
        daq_catch_error();
    }
    #endif
}

void canTxUpdate(void)
{
    #if 0
    CAN_TypeDef* Bus;
    CanMsgTypeDef_t tx_msg;

    for (int bus_id = 0; bus_id < CAN_BUS_COUNT; bus_id++)
    {
        Bus = (bus_id == BUS_ID_CAN1) ? CAN1 : CAN2;
        for (int j = 0; j < CAN_TX_MAILBOX_CNT; j++)
        {
            if (PHAL_txMailboxFree(Bus, j))
            {
                if (qReceive(&q_tx_can1_s[bus_id][j], &tx_msg) == SUCCESS_G)
                {
                    PHAL_txCANMessage(&tx_msg, j);
                    mbx_last_send_time[bus_id][j] = getTick();
                }
            }
            else if (getTick() - mbx_last_send_time[bus_id][j] > CAN_TX_TIMEOUT_MS)
            {
                daq_catch_error();
                PHAL_txCANAbort(Bus, j); // aborts tx and empties the mailbox
                can_stats[bus_id].tx_fail++;
            }
        }
    }
    #endif
}
