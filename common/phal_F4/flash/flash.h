/**
 * @file hal_flash.h
 * @author Adam Busch (busch8@purdue.edu)
 * @brief 
 * @version 0.1
 * @date 2021-03-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef PER_HAL_FLASH
#define PER_HAL_FLASH

#include "stm32l4xx.h"

// Flash magic numbers obtained from family reference manual
#define FLASH_KEY_1 0x45670123
#define FLASH_KEY_2 0xCDEF89AB

// void PHAL_flashWriteU32(uint32_t* address, uint32_t value);
void PHAL_flashWriteU64(uint32_t address, uint64_t data);
void PHAL_flashErasePage(uint8_t page);


#endif