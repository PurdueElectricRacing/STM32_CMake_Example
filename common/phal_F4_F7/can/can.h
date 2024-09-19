/**
 * @file can.h
 * @author Adam Busch (busch8@purdue.edu)
 * @brief
 * @version 0.1
 * @date 2021-03-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef _PHAL_CAN_H
#define _PHAL_CAN_H

#if defined(STM32F407xx)
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#elif defined(STM32F732xx)
#include "stm32f7xx.h"
#include "system_stm32f7xx.h"
#else
#error "Please define a MCU arch"
#endif

#include <stdbool.h>

#define PHAL_CAN_TX_TIMEOUT   (5000U)
#define PHAL_CAN_INIT_TIMEOUT (5000U)

// Bit timing recovered from http://www.bittiming.can-wiki.info/
#define PHAL_CAN_16MHz_500k (0x033a0001) // sample point = 75%, SJW = 4
#define PHAL_CAN_24MHz_500k (0x033a0002) // sample point = 75%, SJW = 4
#define PHAL_CAN_36MHz_500k (0x03270005)
#define PHAL_CAN_42MHz_500k (0x034e0003) // sample point = 75%, SJW = 4

#define PHAL_CAN_16MHz_250k (0x003a0003) // sample point = 75%
#define PHAL_CAN_24MHz_250k (0x003a0005) // sample point = 75%
#define PHAL_CAN_36MHz_250k (0x003a0008) // sample point = 75%

typedef struct
{
  CAN_TypeDef* Bus; /*!< Specifies the bus. */
  uint16_t StdId; /*!< Specifies the standard identifier. */
  uint32_t ExtId; /*!< Specifies the extended identifier. */
  uint32_t IDE; /*!< Specifies the type of identifier for the message that will be transmitted.  */
  uint32_t DLC; /*!< Specifies the length of the frame that will be transmitted. */
  uint8_t Data[8]; /*!< Contains the data to be transmitted. */
} CanMsgTypeDef_t;

/**
 * @brief Initilize CAN peripheral to bit_rate.
 *
 * @param test_mode Initilize CAN peripheral for self test mode
 * @param bit_rate  Bit rate in bps (i.e. 500000)
 *
 * @return true Peripheral sucessfully initalized
 * @return false Peripheral stalled during initilization
 */
bool PHAL_initCAN(CAN_TypeDef* bus, bool test_mode, uint32_t bit_rate);

bool PHAL_deinitCAN(CAN_TypeDef* bus);

/**
 * @brief Find an empty TX mailbox and transmit a CAN message if one is found.
 * Function will block until sucessful transmission of message until a specified timeout.
 *
 * @param can CAN peripheral to transmit with
 * @param msgId Message ID
 * @return true Sucessful TX of message.
 * @return false Unable to find empty message or transmit took too long.
 */
bool PHAL_txCANMessage(CanMsgTypeDef_t* msg, uint8_t mbx);
bool PHAL_txMailboxFree(CAN_TypeDef* bus, uint8_t mbx);
void PHAL_txCANAbort(CAN_TypeDef* bus, uint8_t mbx);

#endif // _PHAL_CAN_H
