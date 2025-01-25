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
            case ID_RAW_THROTTLE_BRAKE:
                can_data.raw_throttle_brake.throttle = msg_data_a->raw_throttle_brake.throttle;
                can_data.raw_throttle_brake.throttle_right = msg_data_a->raw_throttle_brake.throttle_right;
                can_data.raw_throttle_brake.brake = msg_data_a->raw_throttle_brake.brake;
                can_data.raw_throttle_brake.brake_right = msg_data_a->raw_throttle_brake.brake_right;
                can_data.raw_throttle_brake.brake_pot = msg_data_a->raw_throttle_brake.brake_pot;
                can_data.raw_throttle_brake.stale = 0;
                can_data.raw_throttle_brake.last_rx = sched.os_ticks;
                break;
            case ID_FILT_THROTTLE_BRAKE:
                can_data.filt_throttle_brake.throttle = msg_data_a->filt_throttle_brake.throttle;
                can_data.filt_throttle_brake.brake = msg_data_a->filt_throttle_brake.brake;
                can_data.filt_throttle_brake.stale = 0;
                can_data.filt_throttle_brake.last_rx = sched.os_ticks;
                break;
            case ID_START_BUTTON:
                can_data.start_button.start = msg_data_a->start_button.start;
                break;
            case ID_MAX_CELL_TEMP:
                can_data.max_cell_temp.max_temp = (int16_t) msg_data_a->max_cell_temp.max_temp;
                can_data.max_cell_temp.stale = 0;
                can_data.max_cell_temp.last_rx = sched.os_ticks;
                break;
            case ID_LWS_STANDARD:
                can_data.LWS_Standard.LWS_ANGLE = (int16_t) msg_data_a->LWS_Standard.LWS_ANGLE;
                can_data.LWS_Standard.LWS_SPEED = msg_data_a->LWS_Standard.LWS_SPEED;
                can_data.LWS_Standard.Ok = msg_data_a->LWS_Standard.Ok;
                can_data.LWS_Standard.Cal = msg_data_a->LWS_Standard.Cal;
                can_data.LWS_Standard.Trim = msg_data_a->LWS_Standard.Trim;
                can_data.LWS_Standard.Reserved_1 = msg_data_a->LWS_Standard.Reserved_1;
                can_data.LWS_Standard.Reserved_2 = msg_data_a->LWS_Standard.Reserved_2;
                can_data.LWS_Standard.stale = 0;
                can_data.LWS_Standard.last_rx = sched.os_ticks;
                break;
            case ID_MAIN_MODULE_BL_CMD:
                can_data.main_module_bl_cmd.cmd = msg_data_a->main_module_bl_cmd.cmd;
                can_data.main_module_bl_cmd.data = msg_data_a->main_module_bl_cmd.data;
                main_module_bl_cmd_CALLBACK(msg_data_a);
                break;
            case ID_ORION_CURRENTS_VOLTS:
                can_data.orion_currents_volts.pack_current = (int16_t) msg_data_a->orion_currents_volts.pack_current;
                can_data.orion_currents_volts.pack_voltage = msg_data_a->orion_currents_volts.pack_voltage;
                can_data.orion_currents_volts.stale = 0;
                can_data.orion_currents_volts.last_rx = sched.os_ticks;
                break;
            case ID_THROTTLE_VCU:
                can_data.throttle_vcu.vcu_k_rl = (int16_t) msg_data_a->throttle_vcu.vcu_k_rl;
                can_data.throttle_vcu.vcu_k_rr = (int16_t) msg_data_a->throttle_vcu.vcu_k_rr;
                can_data.throttle_vcu.stale = 0;
                can_data.throttle_vcu.last_rx = sched.os_ticks;
                break;
            case ID_THROTTLE_VCU_EQUAL:
                can_data.throttle_vcu_equal.equal_k_rl = (int16_t) msg_data_a->throttle_vcu_equal.equal_k_rl;
                can_data.throttle_vcu_equal.equal_k_rr = (int16_t) msg_data_a->throttle_vcu_equal.equal_k_rr;
                can_data.throttle_vcu_equal.stale = 0;
                can_data.throttle_vcu_equal.last_rx = sched.os_ticks;
                break;
            case ID_AMK_ACTUAL_VALUES_1:
                can_data.AMK_Actual_Values_1.AMK_Status_bReserve = msg_data_a->AMK_Actual_Values_1.AMK_Status_bReserve;
                can_data.AMK_Actual_Values_1.AMK_Status_bSystemReady = msg_data_a->AMK_Actual_Values_1.AMK_Status_bSystemReady;
                can_data.AMK_Actual_Values_1.AMK_Status_bError = msg_data_a->AMK_Actual_Values_1.AMK_Status_bError;
                can_data.AMK_Actual_Values_1.AMK_Status_bWarn = msg_data_a->AMK_Actual_Values_1.AMK_Status_bWarn;
                can_data.AMK_Actual_Values_1.AMK_Status_bQuitDcOn = msg_data_a->AMK_Actual_Values_1.AMK_Status_bQuitDcOn;
                can_data.AMK_Actual_Values_1.AMK_Status_bDcOn = msg_data_a->AMK_Actual_Values_1.AMK_Status_bDcOn;
                can_data.AMK_Actual_Values_1.AMK_Status_bQuitInverterOn = msg_data_a->AMK_Actual_Values_1.AMK_Status_bQuitInverterOn;
                can_data.AMK_Actual_Values_1.AMK_Status_bInverterOn = msg_data_a->AMK_Actual_Values_1.AMK_Status_bInverterOn;
                can_data.AMK_Actual_Values_1.AMK_Status_bDerating = msg_data_a->AMK_Actual_Values_1.AMK_Status_bDerating;
                can_data.AMK_Actual_Values_1.AMK_ActualVelocity = (int16_t) msg_data_a->AMK_Actual_Values_1.AMK_ActualVelocity;
                can_data.AMK_Actual_Values_1.AMK_TorqueCurrent = (int16_t) msg_data_a->AMK_Actual_Values_1.AMK_TorqueCurrent;
                can_data.AMK_Actual_Values_1.AMK_MagnetizingCurrent = (int16_t) msg_data_a->AMK_Actual_Values_1.AMK_MagnetizingCurrent;
                can_data.AMK_Actual_Values_1.stale = 0;
                can_data.AMK_Actual_Values_1.last_rx = sched.os_ticks;
                break;
            case ID_AMK_ACTUAL_VALUES_2:
                can_data.AMK_Actual_Values_2.AMK_TempMotor = (int16_t) msg_data_a->AMK_Actual_Values_2.AMK_TempMotor;
                can_data.AMK_Actual_Values_2.AMK_TempInverter = (int16_t) msg_data_a->AMK_Actual_Values_2.AMK_TempInverter;
                can_data.AMK_Actual_Values_2.AMK_ErrorInfo = msg_data_a->AMK_Actual_Values_2.AMK_ErrorInfo;
                can_data.AMK_Actual_Values_2.AMK_TempIGBT = msg_data_a->AMK_Actual_Values_2.AMK_TempIGBT;
                can_data.AMK_Actual_Values_2.stale = 0;
                can_data.AMK_Actual_Values_2.last_rx = sched.os_ticks;
                break;
            case ID_FAULT_SYNC_PDU:
                can_data.fault_sync_pdu.idx = msg_data_a->fault_sync_pdu.idx;
                can_data.fault_sync_pdu.latched = msg_data_a->fault_sync_pdu.latched;
				handleCallbacks(msg_data_a->fault_sync_main_module.idx, msg_data_a->fault_sync_main_module.latched);
                break;
            case ID_FAULT_SYNC_DASHBOARD:
                can_data.fault_sync_dashboard.idx = msg_data_a->fault_sync_dashboard.idx;
                can_data.fault_sync_dashboard.latched = msg_data_a->fault_sync_dashboard.latched;
				handleCallbacks(msg_data_a->fault_sync_main_module.idx, msg_data_a->fault_sync_main_module.latched);
                break;
            case ID_FAULT_SYNC_A_BOX:
                can_data.fault_sync_a_box.idx = msg_data_a->fault_sync_a_box.idx;
                can_data.fault_sync_a_box.latched = msg_data_a->fault_sync_a_box.latched;
				handleCallbacks(msg_data_a->fault_sync_main_module.idx, msg_data_a->fault_sync_main_module.latched);
                break;
            case ID_FAULT_SYNC_TORQUE_VECTOR:
                can_data.fault_sync_torque_vector.idx = msg_data_a->fault_sync_torque_vector.idx;
                can_data.fault_sync_torque_vector.latched = msg_data_a->fault_sync_torque_vector.latched;
				handleCallbacks(msg_data_a->fault_sync_main_module.idx, msg_data_a->fault_sync_main_module.latched);
                break;
            case ID_FAULT_SYNC_TEST_NODE:
                can_data.fault_sync_test_node.idx = msg_data_a->fault_sync_test_node.idx;
                can_data.fault_sync_test_node.latched = msg_data_a->fault_sync_test_node.latched;
				handleCallbacks(msg_data_a->fault_sync_main_module.idx, msg_data_a->fault_sync_main_module.latched);
                break;
            case ID_SET_FAULT:
                can_data.set_fault.id = msg_data_a->set_fault.id;
                can_data.set_fault.value = msg_data_a->set_fault.value;
				set_fault_daq(msg_data_a->set_fault.id, msg_data_a->set_fault.value);
                break;
            case ID_RETURN_FAULT_CONTROL:
                can_data.return_fault_control.id = msg_data_a->return_fault_control.id;
				return_fault_control(msg_data_a->return_fault_control.id);
                break;
            case ID_DAQ_COMMAND_MAIN_MODULE:
                can_data.daq_command_MAIN_MODULE.daq_command = msg_data_a->daq_command_MAIN_MODULE.daq_command;
                daq_command_MAIN_MODULE_CALLBACK(&msg_header);
                break;
            default:
                __asm__("nop");
        }
        /* END AUTO CASES */
    }

    /* BEGIN AUTO STALE CHECKS */
    CHECK_STALE(can_data.raw_throttle_brake.stale,
                sched.os_ticks, can_data.raw_throttle_brake.last_rx,
                UP_RAW_THROTTLE_BRAKE);
    CHECK_STALE(can_data.filt_throttle_brake.stale,
                sched.os_ticks, can_data.filt_throttle_brake.last_rx,
                UP_FILT_THROTTLE_BRAKE);
    CHECK_STALE(can_data.max_cell_temp.stale,
                sched.os_ticks, can_data.max_cell_temp.last_rx,
                UP_MAX_CELL_TEMP);
    CHECK_STALE(can_data.LWS_Standard.stale,
                sched.os_ticks, can_data.LWS_Standard.last_rx,
                UP_LWS_STANDARD);
    CHECK_STALE(can_data.orion_currents_volts.stale,
                sched.os_ticks, can_data.orion_currents_volts.last_rx,
                UP_ORION_CURRENTS_VOLTS);
    CHECK_STALE(can_data.throttle_vcu.stale,
                sched.os_ticks, can_data.throttle_vcu.last_rx,
                UP_THROTTLE_VCU);
    CHECK_STALE(can_data.throttle_vcu_equal.stale,
                sched.os_ticks, can_data.throttle_vcu_equal.last_rx,
                UP_THROTTLE_VCU_EQUAL);
    CHECK_STALE(can_data.AMK_Actual_Values_1.stale,
                sched.os_ticks, can_data.AMK_Actual_Values_1.last_rx,
                UP_AMK_ACTUAL_VALUES_1);
    CHECK_STALE(can_data.AMK_Actual_Values_2.stale,
                sched.os_ticks, can_data.AMK_Actual_Values_2.last_rx,
                UP_AMK_ACTUAL_VALUES_2);
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
    CAN1->sFilterRegister[0].FR1 = (ID_RAW_THROTTLE_BRAKE << 3) | 4;
    CAN1->sFilterRegister[0].FR2 = (ID_FILT_THROTTLE_BRAKE << 3) | 4;
    CAN1->FA1R |= (1 << 1);    // configure bank 1
    CAN1->sFilterRegister[1].FR1 = (ID_START_BUTTON << 3) | 4;
    CAN1->sFilterRegister[1].FR2 = (ID_MAX_CELL_TEMP << 3) | 4;
    CAN1->FA1R |= (1 << 2);    // configure bank 2
    CAN1->sFilterRegister[2].FR1 = (ID_LWS_STANDARD << 21);
    CAN1->sFilterRegister[2].FR2 = (ID_MAIN_MODULE_BL_CMD << 3) | 4;
    CAN1->FA1R |= (1 << 3);    // configure bank 3
    CAN1->sFilterRegister[3].FR1 = (ID_ORION_CURRENTS_VOLTS << 3) | 4;
    CAN1->sFilterRegister[3].FR2 = (ID_THROTTLE_VCU << 3) | 4;
    CAN1->FA1R |= (1 << 4);    // configure bank 4
    CAN1->sFilterRegister[4].FR1 = (ID_THROTTLE_VCU_EQUAL << 3) | 4;
    CAN1->sFilterRegister[4].FR2 = (ID_AMK_ACTUAL_VALUES_1 << 3) | 4;
    CAN1->FA1R |= (1 << 5);    // configure bank 5
    CAN1->sFilterRegister[5].FR1 = (ID_AMK_ACTUAL_VALUES_2 << 3) | 4;
    CAN1->sFilterRegister[5].FR2 = (ID_FAULT_SYNC_PDU << 3) | 4;
    CAN1->FA1R |= (1 << 6);    // configure bank 6
    CAN1->sFilterRegister[6].FR1 = (ID_FAULT_SYNC_DASHBOARD << 3) | 4;
    CAN1->sFilterRegister[6].FR2 = (ID_FAULT_SYNC_A_BOX << 3) | 4;
    CAN1->FA1R |= (1 << 7);    // configure bank 7
    CAN1->sFilterRegister[7].FR1 = (ID_FAULT_SYNC_TORQUE_VECTOR << 3) | 4;
    CAN1->sFilterRegister[7].FR2 = (ID_FAULT_SYNC_TEST_NODE << 3) | 4;
    CAN1->FA1R |= (1 << 8);    // configure bank 8
    CAN1->sFilterRegister[8].FR1 = (ID_SET_FAULT << 3) | 4;
    CAN1->sFilterRegister[8].FR2 = (ID_RETURN_FAULT_CONTROL << 3) | 4;
    CAN1->FA1R |= (1 << 9);    // configure bank 9
    CAN1->sFilterRegister[9].FR1 = (ID_DAQ_COMMAND_MAIN_MODULE << 3) | 4;
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
