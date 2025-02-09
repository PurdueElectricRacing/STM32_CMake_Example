/**
 * @file main.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Software for controlling vehicle power
 *         distribution and monitoring
 * @version 0.1
 * @date 2023-11-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _MAIN_H_
#define _MAIN_H_


//STM32F407VGT6

#include "common/faults/fault_nodes.h"
#include "common/phal_F4_F7/can/can.h"

#define FAULT_NODE_NAME NODE_PDU

// Status Indicators
#define ERR_LED_GPIO_Port           (GPIOC)
#define ERR_LED_Pin                 (13)
#define CONN_LED_GPIO_Port          (GPIOC)
#define CONN_LED_Pin                (14)
#define CONN_LED_MS_THRESH          (500)
#define HEARTBEAT_GPIO_Port         (GPIOC)
#define HEARTBEAT_Pin               (15)

// CAN
#define VCAN_RX_GPIO_Port           (GPIOD)
#define VCAN_RX_Pin                 (0)
#define VCAN_TX_GPIO_Port           (GPIOD)
#define VCAN_TX_Pin                 (1)

// EEPROM
#define SPI2_SCK_GPIO_Port          (GPIOB)
#define SPI2_SCK_Pin                (13)
#define SPI2_MISO_GPIO_Port         (GPIOB)
#define SPI2_MISO_Pin               (14)
#define SPI2_MOSI_GPIO_Port         (GPIOB)
#define SPI2_MOSI_Pin               (15)

#define EEPROM_nWP_GPIO_Port        (GPIOB)
#define EEPROM_nWP_Pin              (12)
#define EEPROM_NSS_GPIO_Port        (GPIOB)
#define EEPROM_NSS_Pin              (11)

// LED CTRL
#define SPI1_SCK_GPIO_Port          (GPIOB)
#define SPI1_SCK_Pin                (3)
#define SPI1_MOSI_GPIO_Port         (GPIOB)
#define SPI1_MOSI_Pin               (5)

#define LED_CTRL_LAT_GPIO_Port      (GPIOD)
#define LED_CTRL_LAT_Pin            (7)
#define LED_CTRL_BLANK_GPIO_Port    (GPIOB)
#define LED_CTRL_BLANK_Pin          (6)

// NOTE: Schematic refers to coolant hardware as 1 and 2
// MAPPING
// 1 = Battery Cooling
// 2 = Drivetrain Cooling

// Flow Rate
#define FLOW_RATE_1_TIM             (TIM3)
#define FLOW_RATE_1_TIM_CH          (1)
#define FLOW_RATE_1_GPIO_Port       (GPIOC)
#define FLOW_RATE_1_Pin             (6)
#define FLOW_RATE_1_AF              (2)

#define FLOW_RATE_2_TIM             (TIM8)
#define FLOW_RATE_2_TIM_CH          (2)
#define FLOW_RATE_2_GPIO_Port       (GPIOC)
#define FLOW_RATE_2_Pin             (7)
#define FLOW_RATE_2_AF              (3)

// Fan Control
#define FAN_1_PWM_TIM               (TIM1)
#define FAN_1_PWM_TIM_CH            (1)
#define FAN_1_PWM_GPIO_Port         (GPIOE)
#define FAN_1_PWM_Pin               (9)
#define FAN_1_PWM_AF                (1)

#define FAN_2_PWM_TIM               (TIM1)
#define FAN_2_PWM_TIM_CH            (2)
#define FAN_2_PWM_GPIO_Port         (GPIOE)
#define FAN_2_PWM_Pin               (11)
#define FAN_2_PWM_AF                (1)

#define FAN_1_TACH_TIM              (TIM4)
#define FAN_1_TACH_TIM_CH           (2)
#define FAN_1_TACH_GPIO_Port        (GPIOB)
#define FAN_1_TACH_Pin              (7)
#define FAN_1_TACH_AF               (2)

#define FAN_2_TACH_TIM              (TIM10)
#define FAN_2_TACH_TIM_CH           (1)
#define FAN_2_TACH_GPIO_Port        (GPIOB)
#define FAN_2_TACH_Pin              (8)
#define FAN_2_TACH_AF               (3)

// Pump Switches (High Power)
#define PUMP_1_CTRL_GPIO_Port       (GPIOB)
#define PUMP_1_CTRL_Pin             (10)
#define PUMP_1_IMON_GPIO_Port       (GPIOA)
#define PUMP_1_IMON_Pin             (6)
#define PUMP_1_IMON_ADC_CHNL        (6)

#define PUMP_2_CTRL_GPIO_Port       (GPIOE)
#define PUMP_2_CTRL_Pin             (15)
#define PUMP_2_IMON_GPIO_Port       (GPIOA)
#define PUMP_2_IMON_Pin             (5)
#define PUMP_2_IMON_ADC_CHNL        (5)

// Auxiliary Switch (High Power)
#define AUX_HP_CTRL_GPIO_Port       (GPIOD)
#define AUX_HP_CTRL_Pin             (12)
#define AUX_HP_IMON_GPIO_Port       (GPIOC)
#define AUX_HP_IMON_Pin             (3)
#define AUX_HP_IMON_ADC_CHNL        (13)

// Shutdown Circuit (SDC) Switch (High Power)
#define SDC_IMON_GPIO_Port          (GPIOA)
#define SDC_IMON_Pin                (0)
#define SDC_IMON_ADC_CHNL           (0)

// Fan Switches (Low Power)
#define FAN_1_CTRL_GPIO_Port        (GPIOD)
#define FAN_1_CTRL_Pin              (9)
#define FAN_1_NFLT_GPIO_Port        (GPIOD)
#define FAN_1_NFLT_Pin              (8)
#define FAN_1_CS_GPIO_Port          (GPIOA)
#define FAN_1_CS_Pin                (2)
#define FAN_1_CS_ADC_CHNL           (2)

#define FAN_2_CTRL_GPIO_Port        (GPIOD)
#define FAN_2_CTRL_Pin              (11)
#define FAN_2_NFLT_GPIO_Port        (GPIOD)
#define FAN_2_NFLT_Pin              (10)
#define FAN_2_CS_GPIO_Port          (GPIOA)
#define FAN_2_CS_Pin                (3)
#define FAN_2_CS_ADC_CHNL           (3)

// Main Module Switch (Low Power)
#define MAIN_CTRL_GPIO_Port         (GPIOD)
#define MAIN_CTRL_Pin               (15)
#define MAIN_NFLT_GPIO_Port         (GPIOD)
#define MAIN_NFLT_Pin               (14)
#define MAIN_CS_GPIO_Port           (GPIOC)
#define MAIN_CS_Pin                 (2)
#define MAIN_CS_ADC_CHNL            (12)

// Dashboard Switch (Low Power)
#define DASH_NFLT_GPIO_Port         (GPIOA)
#define DASH_NFLT_Pin               (8)
#define DASH_CS_GPIO_Port           (GPIOC)
#define DASH_CS_Pin                 (1)
#define DASH_CS_ADC_CHNL            (11)

// Accumulator (ABox) Switch (Low Power)
#define ABOX_NFLT_GPIO_Port         (GPIOA)
#define ABOX_NFLT_Pin               (10)
#define ABOX_CS_GPIO_Port           (GPIOC)
#define ABOX_CS_Pin                 (0)
#define ABOX_CS_ADC_CHNL            (10)

// Bullet (Antennae) Switch (Low Power)
#define BLT_CTRL_GPIO_Port          (GPIOE)
#define BLT_CTRL_Pin                (13)
#define BLT_NFLT_GPIO_Port          (GPIOE)
#define BLT_NFLT_Pin                (14)

// 5V Critical Switch (5V)
#define CRIT_5V_CTRL_GPIO_Port      (GPIOE)
#define CRIT_5V_CTRL_Pin            (2)
#define CRIT_5V_NFLT_GPIO_Port      (GPIOE)
#define CRIT_5V_NFLT_Pin            (1)

// 5V Non-Critical Switch (5V)
#define NCRIT_5V_CTRL_GPIO_Port     (GPIOE)
#define NCRIT_5V_CTRL_Pin           (6)
#define NCRIT_5V_NFLT_GPIO_Port     (GPIOE)
#define NCRIT_5V_NFLT_Pin           (5)

// DAQ Switch (5V)
#define DAQ_NFLT_GPIO_Port          (GPIOE)
#define DAQ_NFLT_Pin                (3)

// 5V Fan Switch (5V)
#define FAN_5V_CTRL_GPIO_Port       (GPIOE)
#define FAN_5V_CTRL_Pin             (8)
#define FAN_5V_NFLT_GPIO_Port       (GPIOE)
#define FAN_5V_NFLT_Pin             (7)

// LV Battery BMS
#define LV_BMS_STAT_GPIO_Port       (GPIOA)
#define LV_BMS_STAT_Pin             (15)
#define LV_BMS_TX_GPIO_Port         (GPIOC)
#define LV_BMS_TX_Pin               (10)
#define LV_BMS_RX_GPIO_Port         (GPIOC)
#define LV_BMS_RX_Pin               (11)

// LV Status
#define LV_24V_V_SENSE_GPIO_Port    (GPIOB)
#define LV_24V_V_SENSE_Pin          (0)
#define LV_24V_V_SENSE_ADC_CHNL     (8)
#define LV_24V_I_SENSE_GPIO_Port    (GPIOA)
#define LV_24V_I_SENSE_Pin          (4)
#define LV_24V_I_SENSE_ADC_CHNL     (4)

#define LV_5V_V_SENSE_GPIO_Port     (GPIOC)
#define LV_5V_V_SENSE_Pin           (5)
#define LV_5V_V_SENSE_ADC_CHNL      (15)
#define LV_5V_I_SENSE_GPIO_Port     (GPIOA)
#define LV_5V_I_SENSE_Pin           (1)
#define LV_5V_I_SENSE_ADC_CHNL      (1)

#define LV_3V3_V_SENSE_GPIO_Port    (GPIOC)
#define LV_3V3_V_SENSE_Pin          (4)
#define LV_3V3_V_SENSE_ADC_CHNL     (14)

#define EXTERNAL_THERM_GPIO_Port    (GPIOA)
#define EXTERNAL_THERM_Pin          (7)
#define EXTERNAL_THERM_ADC_CHNL     (7)

#define INTERNAL_THERM_ADC_CHNL     (17)

// ADC Configuration
#define ADC_REF_mV (3300UL) // mV
typedef struct
{
    // Do not modify this struct unless
    // you modify the ADC DMA config
    // in main.c to match
    uint16_t pump_1_imon;
    uint16_t pump_2_imon;
    uint16_t aux_hp_imon;
    uint16_t sdc_imon;
    uint16_t fan_1_cs;
    uint16_t fan_2_cs;
    uint16_t main_cs;
    uint16_t dash_cs;
    uint16_t abox_cs;
    uint16_t lv_24_v_sense;
    uint16_t lv_24_i_sense;
    uint16_t lv_5_v_sense;
    uint16_t lv_5_i_sense;
    uint16_t lv_3v3_v_sense;
    uint16_t external_therm;
    uint16_t internal_therm;
}__attribute__((packed)) ADCReadings_t;
volatile extern ADCReadings_t adc_readings;

void canTxSendToBack(CanMsgTypeDef_t *msg);
#endif