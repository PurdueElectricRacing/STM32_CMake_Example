/**
 * @file can_parse.c
 * @author Cole Roberts (rober638@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2024-09-27
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "can_parse.h"

// prototypes
bool initCANFilter();

can_data_t can_data;
volatile uint32_t last_can_rx_time_ms = 0;

void initCANParse(void)
{
    initCANParseBase();
    initCANFilter();
}

void canRxUpdate(void)
{
    CanMsgTypeDef_t msg_header;
    CanParsedData_t* msg_data_a;

    if(qReceive(&q_rx_can, &msg_header) == SUCCESS_G)
    {
        msg_data_a = (CanParsedData_t *) &msg_header.Data;
        last_can_rx_time_ms = sched.os_ticks;
        /* BEGIN AUTO CASES */
        switch(msg_header.ExtId)
        {
            case ID_INV1_SETPOINTS_1:
                can_data.INV1_Setpoints_1.AMK_Control_bReserve = msg_data_a->INV1_Setpoints_1.AMK_Control_bReserve;
                can_data.INV1_Setpoints_1.AMK_Control_bInverterOn = msg_data_a->INV1_Setpoints_1.AMK_Control_bInverterOn;
                can_data.INV1_Setpoints_1.AMK_Control_bDcOn = msg_data_a->INV1_Setpoints_1.AMK_Control_bDcOn;
                can_data.INV1_Setpoints_1.AMK_Control_bEnable = msg_data_a->INV1_Setpoints_1.AMK_Control_bEnable;
                can_data.INV1_Setpoints_1.AMK_Control_bErrorReset = msg_data_a->INV1_Setpoints_1.AMK_Control_bErrorReset;
                can_data.INV1_Setpoints_1.AMK_Control_bReserve2 = msg_data_a->INV1_Setpoints_1.AMK_Control_bReserve2;
                can_data.INV1_Setpoints_1.AMK_TargetVelocity = (int16_t) msg_data_a->INV1_Setpoints_1.AMK_TargetVelocity;
                can_data.INV1_Setpoints_1.AMK_TorqueLimitPositiv = (int16_t) msg_data_a->INV1_Setpoints_1.AMK_TorqueLimitPositiv;
                can_data.INV1_Setpoints_1.AMK_TorqueLimitNegativ = (int16_t) msg_data_a->INV1_Setpoints_1.AMK_TorqueLimitNegativ;
                can_data.INV1_Setpoints_1.stale = 0;
                can_data.INV1_Setpoints_1.last_rx = sched.os_ticks;
                break;
            default:
                __asm__("nop");
        }
        /* END AUTO CASES */
    }

    /* BEGIN AUTO STALE CHECKS */
    CHECK_STALE(can_data.INV1_Setpoints_1.stale,
                sched.os_ticks, can_data.INV1_Setpoints_1.last_rx,
                UP_INV1_SETPOINTS_1);
    /* END AUTO STALE CHECKS */
}

bool initCANFilter()
{
    CAN1->MCR |= CAN_MCR_INRQ;                // Enter back into INIT state (required for changing scale)
    uint32_t timeout = 0;
    while(!(CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;
    if (timeout == PHAL_CAN_INIT_TIMEOUT)
         return false;

    CAN1->FMR  |= CAN_FMR_FINIT;              // Enter init mode for filter banks
    CAN1->FM1R |= 0x07FFFFFF;                 // Set banks 0-27 to id mode
    CAN1->FS1R |= 0x07FFFFFF;                 // Set banks 0-27 to 32-bit scale

    /* BEGIN AUTO FILTER */
    CAN1->FA1R |= (1 << 0);    // configure bank 0
    CAN1->sFilterRegister[0].FR1 = (ID_INV1_SETPOINTS_1 << 21);
    /* END AUTO FILTER */

    CAN1->FMR  &= ~CAN_FMR_FINIT;             // Enable Filters (exit filter init mode)

    // Enter back into NORMAL mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while((CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;

    return timeout != PHAL_CAN_INIT_TIMEOUT;
}


void canProcessRxIRQs(CanMsgTypeDef_t* rx)
{
    CanParsedData_t* msg_data_a;

    msg_data_a = (CanParsedData_t *) rx->Data;
    switch(rx->ExtId)
    {
        /* BEGIN AUTO RX IRQ */
        /* END AUTO RX IRQ */
        default:
            __asm__("nop");
    }
}
