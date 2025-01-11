/**
 * @file main.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Software to measure pedal state and control the LCD display
 * @version 0.1
 * @date 2022-03-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "common/faults/fault_nodes.h"
#include "common/phal_F4_F7/can/can.h"

#define FAULT_NODE_NAME NODE_DASHBOARD

//STM32F407

// Shockpot Calibration
#define POT_VOLT_MAX_L 4.0f
#define POT_VOLT_MIN_L 4090.0f
#define POT_VOLT_MAX_R 4.0f
#define POT_VOLT_MIN_R 4060.0f
#define POT_MAX_DIST 75
#define POT_DIST_DROOP_L 55
#define POT_DIST_DROOP_R 57

typedef struct __attribute__((packed))
{
    // Do not modify this struct unless
    // you modify the ADC DMA config
    // in main.h to match
    uint16_t t1;
    uint16_t t2;
    uint16_t b1;
    uint16_t b2;
    uint16_t shock_left;
    uint16_t shock_right;
    uint16_t lv_5v_sense;
    uint16_t lv_3v3_sense;
    uint16_t lv_12v_sense;
    uint16_t lv_24_v_sense;
    uint16_t load_l;
    uint16_t load_r;
} raw_adc_values_t;

volatile extern raw_adc_values_t raw_adc_values;

typedef enum
{
    DASH_INPUT_NONE,
    DASH_INPUT_ROT_ENC,
    DASH_INPUT_UP_BUTTON,
    DASH_INPUT_DOWN_BUTTON,
    DASH_INPUT_SELECT_BUTTON,
    DASH_INPUT_START_BUTTON,
    DASH_INPUT_COUNT,
    DASH_INPUT_INVALID,
} dashboard_input_t;

// Status LED Indicators
#define CONN_LED_GPIO_Port          (GPIOE)
#define CONN_LED_Pin                (8)
#define CONN_LED_MS_THRESH          (500)
#define HEART_LED_GPIO_Port         (GPIOE)
#define HEART_LED_Pin               (9)
#define ERROR_LED_GPIO_Port         (GPIOE)
#define ERROR_LED_Pin               (7)
#define PRCHG_LED_GPIO_Port         (GPIOE)
#define PRCHG_LED_Pin               (1)
#define IMD_LED_GPIO_Port           (GPIOE)
#define IMD_LED_Pin                 (2)
#define BMS_LED_GPIO_Port           (GPIOE)
#define BMS_LED_Pin                 (3)

// Status Inputs
#define START_BTN_GPIO_Port         (GPIOD)
#define START_BTN_Pin               (11)

#define BRK_STAT_TAP_GPIO_Port      (GPIOB)
#define BRK_STAT_TAP_Pin            (9)
#define BRK_FAIL_TAP_GPIO_Port      (GPIOA)
#define BRK_FAIL_TAP_Pin            (6)

#define DAQ_SWITCH_GPIO_Port        (GPIOD)
#define DAQ_SWITCH_Pin              (8)

// Rotary Encoder
#define ENC_A_GPIO_Port             (GPIOD)
#define ENC_A_Pin                   (10)
#define ENC_B_GPIO_Port             (GPIOD)
#define ENC_B_Pin                   (9)
#define ENC_NUM_STATES              (4)

// CAN
#define VCAN_RX_GPIO_Port           (GPIOD)
#define VCAN_RX_Pin                 (0)
#define VCAN_TX_GPIO_Port           (GPIOD)
#define VCAN_TX_Pin                 (1)

// SPI Peripherals
#define SPI2_SCK_GPIO_Port          (GPIOB)
#define SPI2_SCK_Pin                (13)
#define SPI2_MISO_GPIO_Port         (GPIOB)
#define SPI2_MISO_Pin               (14)
#define SPI2_MOSI_GPIO_Port         (GPIOB)
#define SPI2_MOSI_Pin               (15)

#define EEPROM_nWP_GPIO_Port        (GPIOB)
#define EEPROM_nWP_Pin              (11)
#define EEPROM_NSS_GPIO_Port        (GPIOB)
#define EEPROM_NSS_Pin              (12)

// Throttle
#define THTL_1_GPIO_Port            (GPIOA)
#define THTL_1_Pin                  (2)
#define THTL_1_ADC_CHNL             (2)
#define THTL_2_GPIO_Port            (GPIOA)
#define THTL_2_Pin                  (3)
#define THTL_2_ADC_CHNL             (3)


// Aux Button inputs
#define B_SELECT_GPIO_Port          (GPIOD)
#define B_SELECT_Pin                (12)
#define B_DOWN_GPIO_Port            (GPIOD)
#define B_DOWN_Pin                  (13)
#define B_UP_GPIO_Port              (GPIOD)
#define B_UP_Pin                    (14)

// Brake
#define BRK_1_GPIO_Port             (GPIOA)
#define BRK_1_Pin                   (0)
#define BRK_1_ADC_CHNL              (0)
#define BRK_2_GPIO_Port             (GPIOA)
#define BRK_2_Pin                   (1)
#define BRK_2_ADC_CHNL              (1)

// Shock Pots
#define SHOCK_POT_L_GPIO_Port       (GPIOC)
#define SHOCK_POT_L_Pin             (0)
#define SHOCK_POT_L_ADC_CH          (10)
#define SHOCK_POT_R_GPIO_Port       (GPIOC)
#define SHOCK_POT_R_Pin             (1)
#define SHOCK_POT_R_ADC_CH          (11)

// Normal Force
#define LOAD_FL_GPIO_Port           (GPIOB)
#define LOAD_FL_Pin                 (0)
#define LOAD_FL_ADC_CH              (8)
#define LOAD_FR_GPIO_Port           (GPIOB)
#define LOAD_FR_Pin                 (1)
#define LOAD_FR_ADC_CH              (9)

// LCD
#define LCD_UART                    (USART1)
#define LCD_UART_TX_GPIO_Port       (GPIOA)
#define LCD_UART_TX_Pin             (9)
#define LCD_UART_RX_GPIO_Port       (GPIOA)
#define LCD_UART_RX_Pin             (10)
#define LCD_NUM_PAGES               (9) // Number encoder selectable pages

// LV Status
#define LV_5V_V_SENSE_GPIO_Port     (GPIOC)
#define LV_5V_V_SENSE_Pin           (2)
#define LV_5V_V_SENSE_ADC_CHNL      (12)
#define LV_3V3_V_SENSE_GPIO_Port    (GPIOC)
#define LV_3V3_V_SENSE_Pin          (3)
#define LV_3V3_V_SENSE_ADC_CHNL     (13)
#define LV_12_V_SENSE_GPIO_Port     (GPIOC)
#define LV_12_V_SENSE_Pin           (4)
#define LV_12_V_SENSE_ADC_CHNL      (14)
#define LV_24_V_SENSE_GPIO_Port     (GPIOC)
#define LV_24_V_SENSE_Pin           (5)
#define LV_24_V_SENSE_ADC_CHNL      (15)
#define LV_24_V_FAULT_GPIO_Port     (GPIOC)
#define LV_24_V_FAULT_Pin           (8)
#define LV_5V_SCALE                 (0.413F)

// Voltage Sensing Resistors (in kOhms)
#define LV_3V3_PULLUP               (4.3F)
#define LV_3V3_PULLDOWN             (10.0F)
#define LV_5V_PULLUP                (4.3F)
#define LV_5V_PULLDOWN              (3.3F)
#define LV_12V_PULLUP               (15.8F)
#define LV_12V_PULLDOWN             (3.3F)
#define LV_24V_PULLUP               (47.0F)
#define LV_24V_PULLDOWN             (3.3F)
#define ADC_MAX_VALUE               (4095)
#define ADC_REF_VOLTAGE             (3.3F)

void canTxSendToBack(CanMsgTypeDef_t *msg);

#endif
