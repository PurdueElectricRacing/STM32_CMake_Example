#include "common/daq/can_parse_base.h"
/**
 * q_tx_can_0 -> hlp [0,1] -> mailbox 1
 * q_tx_can_1 -> hlp [2,3] -> mailbox 2 
 * q_tx_can_2 -> hlp [4,5] -> mailbox 3
*/

can_stats_t can_stats;

// TODO: support CAN2
// CAN2 cannot happen in here, move it over to MAIN stuff
q_handle_t q_tx_can1_s[CAN_TX_MAILBOX_CNT];
q_handle_t q_tx_can2_s[CAN_TX_MAILBOX_CNT];
q_handle_t q_rx_can;
uint32_t can1_mbx_last_send_time[CAN_TX_MAILBOX_CNT];
uint32_t can2_mbx_last_send_time[CAN_TX_MAILBOX_CNT];

void initCANParseBase()
{
    for (uint8_t i = 0; i < CAN_TX_MAILBOX_CNT; ++i)
    {
        qConstruct(&q_tx_can1_s[i], sizeof(CanMsgTypeDef_t));
        qConstruct(&q_tx_can2_s[i], sizeof(CanMsgTypeDef_t));
        can1_mbx_last_send_time[i] = 0;
        can2_mbx_last_send_time[i] = 0;
    }
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));
    can_stats = (can_stats_t){0};
}

void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    q_handle_t *qh;
    if (msg->IDE == 1)
    {
        // extended id, check hlp
        switch((msg->ExtId >> 26) & 0b111)
        {
            case 0:
            case 1:
                if (msg->Bus == CAN1) {
                    qh = &q_tx_can1_s[0];
                } else {
                    qh = &q_tx_can2_s[0];
                }
                break;
            case 2:
            case 3:
                if (msg->Bus == CAN1) {
                    qh = &q_tx_can1_s[1];
                } else {
                    qh = &q_tx_can2_s[1];
                }
                break;
            default:
                if (msg->Bus == CAN1) {
                    qh = &q_tx_can1_s[2];
                } else {
                    qh = &q_tx_can2_s[2];
                }
                break;
        }
    }
    else
    {
        // IDE = 0 doesn't have an HLP
        if (msg->Bus == CAN1) {
            qh = &q_tx_can1_s[0];
        } else {
            qh = &q_tx_can2_s[0];
        }
        // qh = &q_tx_can1_s[0]; // IDE = 0 doesn't have an HLP
    }
    if (qSendToBack(qh, msg) != SUCCESS_G)
    {
        can_stats.tx_of++;
    }
}

void canTxUpdate(void)
{
    CanMsgTypeDef_t tx_msg;
    // TODO: we only check if CAN1 mailbox is free -> create separate queue for
    // CAN2 is you are using it!!!!
    for (uint8_t i = 0; i < CAN_TX_MAILBOX_CNT; ++i)
    {
        if(PHAL_txMailboxFree(CAN1, i))
        {
            if (qReceive(&q_tx_can1_s[i], &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
            {
                PHAL_txCANMessage(&tx_msg, i);
                can1_mbx_last_send_time[i] = sched.os_ticks;
            }
        }
        else if (sched.os_ticks - can1_mbx_last_send_time[i] > CAN_TX_TIMEOUT_MS)
        {
            PHAL_txCANAbort(CAN1, i); // aborts tx and empties the mailbox
            can_stats.tx_fail++;
        }
    }
}

void canParseIRQHandler(CAN_TypeDef *can_h)
{
    if (can_h->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
    {
        can_h->RF0R |= CAN_RF0R_FOVR0;
        can_stats.rx_overrun++;
    }

    if (can_h->RF0R & CAN_RF0R_FULL0) // FIFO Full
        can_h->RF0R |= CAN_RF0R_FULL0;

    if (can_h->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        CanMsgTypeDef_t rx;
        rx.Bus = can_h;

        // Get either StdId or ExtId
        rx.IDE = CAN_RI0R_IDE & can_h->sFIFOMailBox[0].RIR;
        if (rx.IDE)
        {
          rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & can_h->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
        }
        else
        {
          rx.StdId = (CAN_RI0R_STID & can_h->sFIFOMailBox[0].RIR) >> CAN_RI0R_STID_Pos;
          rx.ExtId = rx.StdId; // for can_parse (assumes all are ExtId)
        }

        rx.DLC = (CAN_RDT0R_DLC & can_h->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

        rx.Data[0] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 0)  & 0xFF;
        rx.Data[1] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 8)  & 0xFF;
        rx.Data[2] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 0)  & 0xFF;
        rx.Data[5] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 8)  & 0xFF;
        rx.Data[6] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        can_h->RF0R |= (CAN_RF0R_RFOM0);
        can_h->RF0R |= (CAN_RF0R_RFOM0);

        if (qSendToBack(&q_rx_can, &rx) != SUCCESS_G)
        {
            can_stats.rx_of++;
        }
    }
}
