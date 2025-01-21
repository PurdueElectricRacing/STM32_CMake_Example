/**
 * @file daq_base.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Embedded DAQ protocol meant to communicate with a PC dashboard over CAN
 * @version 0.1
 * @date 2021-10-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _DAQ_BASE_H_
#define _DAQ_BASE_H_

#include <string.h>
#if defined(STM32F407xx)
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#elif defined(STM32F732xx)
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#else
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/gpio/gpio.h"
#define DAQ_EEPROM_ENABLE
#include "common/eeprom_spi/eeprom_spi.h"
#endif
#include "common/queue/queue.h"


typedef void (*read_func_ptr_t)(void* arg);
typedef void (*write_func_ptr_t)(void* arg);


#define DAQ_CMD_LENGTH    3 // bits
#define DAQ_CMD_MASK      0b111

typedef enum
{
    DAQ_CMD_READ      = 0,
    DAQ_CMD_WRITE     = 1,
    DAQ_CMD_LOAD      = 2,
    DAQ_CMD_SAVE      = 3, 
    DAQ_CMD_PUB_START = 4, 
    DAQ_CMD_PUB_STOP  = 5,
    DAQ_CMD_READ_PIN  = 6
} Daq_Cmd_t;

typedef enum
{
    DAQ_RPLY_READ        = 0,
    DAQ_RPLY_SAVE        = 1,
    DAQ_RPLY_READ_ERROR  = 2,
    DAQ_RPLY_WRITE_ERROR = 3,
    DAQ_RPLY_SAVE_ERROR  = 4,
    DAQ_RPLY_LOAD_ERROR  = 5,
    DAQ_RPLY_PUB         = 6,
    DAQ_RPLY_READ_PIN    = 7
} Daq_Rply_t;

#define DAQ_ID_LENGTH     5 // bits
#define DAQ_ID_MASK       0b11111

#define DAQ_BANK_LENGTH   4 // bits
#define DAQ_BANK_MASK     0xF
#define DAQ_PIN_LENGTH    4 // bits
#define DAQ_PIN_MASK      0xF

#define DAQ_PIN_VAL_LENGTH 2 // bits
#define DAQ_PIN_VAL_ERROR  0x2

typedef struct
{
    union {
        volatile void* read_var_a;
        read_func_ptr_t read_func_a;
    };  
    // write: either function or pointer or none
    union {
        volatile void* write_var_a;
        write_func_ptr_t write_func_a;
    };
    // flags
    struct {
        uint8_t is_read_only:   1;
        uint8_t has_read_func:  1;
        uint8_t has_write_func: 1;
    };
    uint8_t bit_length;
    // publishing
    uint8_t pub_period; // in terms of 15ms (i.e. 1 == 15ms period)
} daq_variable_t;

/**
 * @brief Call after linking read and write functions / addresses
 *        If the eeprom is enabled, configures the eeprom to contain
 *        the variables and loads default values if applicable
 * 
 * @param tx_a Address of the tx CAN buffer to send responses to
 * 
 * @return Returns false if the operation was successful
 */
bool daqInitBase(q_handle_t* tx_a, uint8_t num_variables, CAN_TypeDef* hcan, uint32_t ext_id, 
             daq_variable_t* tracked_vars_);

/**
 * @brief Call periodically at DAQ_UPDATE_PERIOD
 *        Processes pending eeprom save commands 
 *        and publishes live variables
 */
void daqPeriodicBase();

/**
 * @brief              Callback for receiving a daq message,
 *                     automatically called by can_parse.c
 * 
 * @param msg_header_a Rx CAN message
 */
void daq_command_callback(CanMsgTypeDef_t* msg_header_a);

#endif