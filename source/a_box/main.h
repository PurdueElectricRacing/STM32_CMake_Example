#ifndef MAIN_H_
#define MAIN_H_


#include "common/faults/fault_nodes.h"
#include "common/phal_F4_F7/can/can.h"


#define FAULT_NODE_NAME NODE_A_BOX


// Current Sense
#define I_SENSE_CH1_GPIO_Port (GPIOA) 
#define I_SENSE_CH1_Pin (0)
#define I_SENSE_CH1_ADC_CHNL  (0)
#define I_SENSE_CH2_GPIO_Port (GPIOA)
#define I_SENSE_CH2_Pin (1)
#define I_SENSE_CH2_ADC_CHNL  (1)


// BMS Status
#define BMS_STATUS_GPIO_Port (GPIOD)
#define BMS_STATUS_Pin (1)
#define BMS_DISCHARGE_ENABLE_Port (GPIOD)
#define BMS_DISCHARGE_ENABLE_Pin (0)
#define BMS_CHARGE_ENABLE_Port (GPIOD)
#define BMS_CHARGE_ENABLE_Pin (3)
#define BMS_CHARGER_SAFETY_Port (GPIOD)
#define BMS_CHARGER_SAFETY_Pin (2)


// IMD Data
#define IMD_HS_PWM_GPIO_Port (GPIOB)
#define IMD_HS_PWM_Pin (3)
#define IMD_LS_PWM_GPIO_Port (GPIOA)
#define IMD_LS_PWM_Pin (15)
#define IMD_STATUS_GPIO_Port (GPIOB)
#define IMD_STATUS_Pin (5)


// Status LEDs
#define HEARTBEAT_LED_GPIO_Port (GPIOE)
#define HEARTBEAT_LED_Pin       (13)
#define CONN_LED_GPIO_Port      (GPIOE)
#define CONN_LED_Pin            (14)
#define ERROR_LED_GPIO_Port     (GPIOE)
#define ERROR_LED_Pin           (15)


// TMU Mux Input Pins
#define MUX_A_Port  (GPIOC)
#define MUX_A_Pin   (9)
#define MUX_B_Port  (GPIOC)
#define MUX_B_Pin   (8)
#define MUX_C_Port  (GPIOC)
#define MUX_C_Pin   (7)
#define MUX_D_Port  (GPIOC)
#define MUX_D_Pin   (6)

// TMU Mux Output Pins
#define TMU_1_1_Port (GPIOA)
#define TMU_1_1_Pin (2)
#define TMU_1_2_Port (GPIOA)
#define TMU_1_2_Pin (3)
#define TMU_2_1_Port (GPIOA)
#define TMU_2_1_Pin (4)
#define TMU_2_2_Port (GPIOA)
#define TMU_2_2_Pin (5)
#define TMU_3_1_Port (GPIOA)
#define TMU_3_1_Pin (6)
#define TMU_3_2_Port (GPIOA)
#define TMU_3_2_Pin (7)

#define TMU_4_1_Port (GPIOC)
#define TMU_4_1_Pin (0)
#define TMU_4_2_Port (GPIOC)
#define TMU_4_2_Pin (1)
#define TMU_5_1_Port (GPIOC)
#define TMU_5_1_Pin (2)
#define TMU_5_2_Port (GPIOC)
#define TMU_5_2_Pin (3)

// Board Temp
#define BOARD_TEMP_Port (GPIOB)
#define BOARD_TEMP_Pin (0)

// 5V Monitoring
#define VSENSE_5V_Port (GPIOA)
#define VSENSE_5V_Pin (8)
#define VSENSE_5V_ADC_CHNL  (7) // not sure if channel has changed

// ADC Channels - Ref (see additional functions): https://www.st.com/resource/en/datasheet/dm00037051.pdf#page=48 
// TMU
#define TMU_1_1_ADC_CHANNEL (2) // ADC123
#define TMU_1_2_ADC_CHANNEL (3) // ADC123
#define TMU_2_1_ADC_CHANNEL (4) // ADC12
#define TMU_2_2_ADC_CHANNEL (5) // ADC12
#define TMU_3_1_ADC_CHANNEL (6) // ADC12
#define TMU_3_2_ADC_CHANNEL (7) // ADC12

#define TMU_4_1_ADC_CHANNEL (10) // ADC123 
#define TMU_4_2_ADC_CHANNEL (11) // ADC123
#define TMU_5_1_ADC_CHANNEL (12) // ADC123
#define TMU_5_2_ADC_CHANNEL (13) // ADC123
// Current Sense
#define I_SENSE_CH1_ADC_CHANNEL (0)
#define I_SENSE_CH2_ADC_CHANNEL (1)
// Vref
#define ADC_VREF 3.3F
#define ADC_ADDR_SIZE 0xFFFU
// Current sense resistors in ohms
#define R1_ISENSE 1000
#define R2_ISENSE 2000

typedef struct 
{
    // Do not modify this struct unless
    // you modify the ADC DMA config
    // in main.c to match
    uint16_t tmu_1;
    uint16_t tmu_2;
    uint16_t tmu_3;
    uint16_t tmu_4;
    // CAN I DO THIS??
    uint16_t isense_ch1;
    uint16_t isense_ch2;
}__attribute__((packed)) ADCReadings_t;
volatile extern ADCReadings_t adc_readings;

extern bool bms_daq_override;
extern bool bms_daq_stat;

void canTxSendToBack(CanMsgTypeDef_t *msg);

#endif
