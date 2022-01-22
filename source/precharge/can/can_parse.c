/**
 * @file can_parse.c
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2021-09-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "can_parse.h"

// prototypes
bool initCANFilter();

can_data_t can_data;
q_handle_t* q_rx_can_a;

void initCANParse(q_handle_t* rx_a)
{
    q_rx_can_a = rx_a;
    initCANFilter();
}

uint32_t curr_tick = 0;

void canRxUpdate()
{
    curr_tick += 1;

    CanMsgTypeDef_t msg_header;
    CanParsedData_t* msg_data_a;

    if(qReceive(q_rx_can_a, &msg_header) == SUCCESS_G)
    {
        msg_data_a = (CanParsedData_t *) &msg_header.Data;
        /* BEGIN AUTO CASES */
        if (msg_header.Bus == CAN2)
        {
            switch(msg_header.ExtId)
            {
                case ID_SOC1:
                    can_data.soc1.soc = msg_data_a->soc1.soc;
                    break;
                default:
                    __asm__("nop");
            }
        }
        else if (msg_header.Bus == CAN1)
        {
            switch(msg_header.ExtId)
            {
                default:
                    __asm__("nop");
            }
        }
        /* END AUTO CASES */
    }

    /* BEGIN AUTO STALE CHECKS */
    /* END AUTO STALE CHECKS */
}

bool initCANFilter()
{

    CAN2->MCR |= CAN_MCR_INRQ;                // Enter back into INIT state (required for changing scale)
    uint32_t timeout = 0;
    while(!(CAN2->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;
    if (timeout == PHAL_CAN_INIT_TIMEOUT)
         return false;

    CAN2->FMR  |= CAN_FMR_FINIT;              // Enter init mode for filter banks
    CAN2->FM1R |= 0x07FFFFFF;                 // Set banks 0-27 to id mode
    CAN2->FS1R |= 0x07FFFFFF;                 // Set banks 0-27 to 32-bit scale

    /* BEGIN AUTO FILTER */
    CAN2->FA1R |= (1 << 0);    // configure bank 0
    CAN2->sFilterRegister[0].FR1 = (ID_SOC1 << 3) | 4;
    /* END AUTO FILTER */

    CAN2->FMR  &= ~CAN_FMR_FINIT;             // Enable Filters (exit filter init mode)

    // Enter back into NORMAL mode
    CAN2->MCR &= ~CAN_MCR_INRQ;
    while((CAN2->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
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
