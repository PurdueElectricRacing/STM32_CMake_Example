/**
 * @file main.h
 * @author Chris McGalliard (cpmcgalliard@gmail.com)
 * @brief  Software for controlling Torque Vectoring PCB
            sensor acquisition
 * @version 0.1
 * @date 2024-11-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _MAIN_H_
#define _MAIN_H_

#include "common/faults/fault_nodes.h"
#include "common/phal_F4_F7/can/can.h"

#define FAULT_NODE_NAME NODE_TORQUE_VECTOR

#define SPI1_CSB_GYRO_PORT (GPIOA)
#define SPI1_CSB_GYRO_PIN (2)

#define SPI1_CSB_ACCEL_PORT (GPIOA)
#define SPI1_CSB_ACCEL_PIN (3)

#define SPI1_SCLK_PORT (GPIOA)
#define SPI1_SCLK_PIN (5)

#define SPI1_MISO_PORT (GPIOA)
#define SPI1_MISO_PIN (6)

#define SPI1_MOSI_PORT (GPIOA)
#define SPI1_MOSI_PIN (7)

#define UART_OUT_TX_PORT (GPIOA)
#define UART_OUT_TX_PIN (9)

#define UART_OUT_RX_PORT (GPIOA)
#define UART_OUT_RX_PIN (10)

#define VCAN_RX_PORT (GPIOA)
#define VCAN_RX_PIN (11)

#define VCAN_TX_PORT (GPIOA)
#define VCAN_TX_PIN (12)

#define SWDIO_PORT (GPIOA)
#define SWDIO_PIN (13)

#define SWCLK_PORT (GPIOA)
#define SWCLK_PIN (14)

#define SPF_PORT (GPIOA)
#define SPF_PIN (15)

#define INT_GPS_PORT (GPIOB)
#define INT_GPS_PIN (0)

#define RESET_GPS_PORT (GPIOB)
#define RESET_GPS_PIN (1)

#define PPS_GPS_PORT (GPIOB)
#define PPS_GPS_PIN (4)

#define ERR_LED_PORT (GPIOB)
#define ERR_LED_PIN (5)

#define HEARTBEAT_LED_PORT (GPIOB)
#define HEARTBEAT_LED_PIN (6)

#define CONN_LED_PORT (GPIOB)
#define CONN_LED_PIN (7)

#define SPI2_CSB_GPS_PORT (GPIOB)
#define SPI2_CSB_GPS_PIN (9)

#define SPI2_CLK_GPS_PORT (GPIOB)
#define SPI2_CLK_GPS_PIN (10)

#define SD_ERR_LED_PORT (GPIOB)
#define SD_ERR_LED_PIN (11)

#define SD_ACT_LED_PORT (GPIOB)
#define SD_ACT_LED_PIN (12)

#define SD_DET_LED_PORT (GPIOB)
#define SD_DET_LED_PIN (13)

#define SD_DET_SIG_PORT (GPIOB)
#define SD_DET_SIG_PIN (14)

#define SD_ENABLE_PORT (GPIOB)
#define SD_ENABLE_PIN (15)

#define SPI2_MISO_PORT (GPIOC)
#define SPI2_MISO_PIN (2)

#define SPI2_MOSI_PORT (GPIOC)
#define SPI2_MOSI_PIN (3)

#define SAFE_GPS_PORT (GPIOC)
#define SAFE_GPS_PIN (7)

#define SD_D0_PORT (GPIOC)
#define SD_D0_PIN (8)

#define SD_D1_PORT (GPIOC)
#define SD_D1_PIN (9)

#define SD_D2_PORT (GPIOC)
#define SD_D2_PIN (10)

#define SD_D3_PORT (GPIOC)
#define SD_D3_PIN (11)

#define SD_CLK_PORT (GPIOC)
#define SD_CLK_PIN (12)

#define SD_CMD_PORT (GPIOD)
#define SD_CMD_PIN (2)

void canTxSendToBack(CanMsgTypeDef_t *msg);

#endif