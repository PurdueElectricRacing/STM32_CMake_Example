#include "common/daq/can_parse_base.h"
/**
 * q_tx_can_0 -> hlp [0,1] -> mailbox 1
 * q_tx_can_1 -> hlp [2,3] -> mailbox 2
 * q_tx_can_2 -> hlp [4,5] -> mailbox 3
*/

can_stats_t can_stats;

q_handle_t q_tx_can[NUM_CAN_PERIPHERALS][CAN_TX_MAILBOX_CNT];
uint32_t can_mbx_last_send_time[NUM_CAN_PERIPHERALS][CAN_TX_MAILBOX_CNT];

q_handle_t q_rx_can;

void initCANParseBase()
{
    for (uint8_t can_periph = 0; can_periph < NUM_CAN_PERIPHERALS; can_periph++)
    {
      for (uint8_t mbx = 0; mbx < CAN_TX_MAILBOX_CNT; mbx++)
      {
        qConstruct(&q_tx_can[can_periph][mbx], sizeof(CanMsgTypeDef_t));
        can_mbx_last_send_time[can_periph][mbx] = 0;
      }
    }
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));
    can_stats = (can_stats_t){0};
}

void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    q_handle_t *qh;
    uint8_t mailbox;
    uint8_t peripheral_idx = (msg->Bus == CAN1) ? CAN1_IDX : CAN2_IDX;
    if (msg->IDE == 1)
    {
        // extended id, check hlp
        switch((msg->ExtId >> 26) & 0b111)
        {
            case 0:
            case 1:
                mailbox = CAN_MAILBOX_HIGH_PRIO;
                break;
            case 2:
            case 3:
                mailbox = CAN_MAILBOX_MED_PRIO;
                break;
            default:
                mailbox = CAN_MAILBOX_LOW_PRIO;
                break;
        }
        qh = &q_tx_can[peripheral_idx][mailbox];
    }
    else
    {
        qh = &q_tx_can[peripheral_idx][CAN_MAILBOX_HIGH_PRIO]; // IDE = 0 doesn't have an HLP
    }
    if (qSendToBack(qh, msg) != SUCCESS_G)
    {
        can_stats.can_peripheral_stats[peripheral_idx].tx_of++;
    }
}

void canTxUpdate(void)
{
    CanMsgTypeDef_t tx_msg;
    for (uint8_t i = 0; i < CAN_TX_MAILBOX_CNT; ++i)
    {
        // Handle CAN1
        if(PHAL_txMailboxFree(CAN1, i))
        {
            if (qReceive(&q_tx_can[CAN1_IDX][i], &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
            {
                PHAL_txCANMessage(&tx_msg, i);
                can_mbx_last_send_time[CAN1_IDX][i] = sched.os_ticks;
            }
        }
        else if (sched.os_ticks - can_mbx_last_send_time[CAN1_IDX][i] > CAN_TX_TIMEOUT_MS)
        {
            PHAL_txCANAbort(CAN1, i); // aborts tx and empties the mailbox
            can_stats.can_peripheral_stats[CAN1_IDX].tx_fail++;
        }
# ifdef CAN2
        // Handle CAN2
        if(PHAL_txMailboxFree(CAN2, i))
        {
            if (qReceive(&q_tx_can[CAN2_IDX][i], &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
            {
                PHAL_txCANMessage(&tx_msg, i);
                can_mbx_last_send_time[CAN2_IDX][i] = sched.os_ticks;
            }
        }
        else if (sched.os_ticks - can_mbx_last_send_time[CAN2_IDX][i] > CAN_TX_TIMEOUT_MS)
        {
            PHAL_txCANAbort(CAN2, i); // aborts tx and empties the mailbox
            can_stats.can_peripheral_stats[CAN2_IDX].tx_fail++;
        }
#endif
    }
}

void canParseIRQHandler(CAN_TypeDef *can_h)
{
    can_peripheral_stats_t *rx_stats = (can_h == CAN1) ? (&can_stats.can_peripheral_stats[CAN1_IDX]) : (&can_stats.can_peripheral_stats[CAN2_IDX]);
    if (can_h->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
    {
        can_h->RF0R |= CAN_RF0R_FOVR0;
        rx_stats->rx_overrun++;
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