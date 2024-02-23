/* System Includes */
#include "common/bootloader/bootloader_common.h"
#include "common/phal_F4_F7/adc/adc.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/psched/psched.h"
#include "common/faults/faults.h"

/* Module Includes */
#include "main.h"
#include "can_parse.h"
#include "daq.h"
#include "led.h"

GPIOInitConfig_t gpio_config[] = {
    // Status Indicators
    GPIO_INIT_OUTPUT(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_OUTPUT_LOW_SPEED),
    // CAN
    GPIO_INIT_CANRX_PD0,
    GPIO_INIT_CANTX_PD1,
    // EEPROM
    GPIO_INIT_SPI2_SCK_PB13,
    GPIO_INIT_SPI2_MISO_PB14,
    GPIO_INIT_SPI2_MOSI_PB15,
    GPIO_INIT_OUTPUT(EEPROM_nWP_GPIO_Port, EEPROM_nWP_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(EEPROM_NSS_GPIO_Port, EEPROM_NSS_Pin, GPIO_OUTPUT_LOW_SPEED),
    // LED CTRL
    GPIO_INIT_SPI1_SCK_PB3,
    GPIO_INIT_SPI1_MOSI_PB5,
    GPIO_INIT_OUTPUT(LED_CTRL_LAT_GPIO_Port, LED_CTRL_LAT_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(LED_CTRL_BLANK_GPIO_Port, LED_CTRL_BLANK_Pin, GPIO_OUTPUT_LOW_SPEED),
    // Flow Rate
    GPIO_INIT_AF(FLOW_RATE_1_GPIO_Port, FLOW_RATE_1_Pin, FLOW_RATE_1_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(FLOW_RATE_2_GPIO_Port, FLOW_RATE_2_Pin, FLOW_RATE_2_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),
    // Fan Control
    GPIO_INIT_AF(FAN_1_PWM_GPIO_Port, FAN_1_PWM_Pin, FAN_1_PWM_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(FAN_2_PWM_GPIO_Port, FAN_2_PWM_Pin, FAN_2_PWM_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(FAN_1_TACH_GPIO_Port, FAN_1_TACH_Pin, FAN_1_TACH_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(FAN_2_TACH_GPIO_Port, FAN_2_TACH_Pin, FAN_2_TACH_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),
    // Pump Switches
    GPIO_INIT_OUTPUT(PUMP_1_CTRL_GPIO_Port, PUMP_1_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_ANALOG(PUMP_1_IMON_GPIO_Port, PUMP_1_IMON_Pin),
    GPIO_INIT_OUTPUT(PUMP_2_CTRL_GPIO_Port, PUMP_2_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_ANALOG(PUMP_2_IMON_GPIO_Port, PUMP_2_IMON_Pin),
    // Auxiliary Switch
    GPIO_INIT_OUTPUT(AUX_HP_CTRL_GPIO_Port, AUX_HP_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_ANALOG(AUX_HP_IMON_GPIO_Port, AUX_HP_IMON_Pin),
    // SDC Switch
    GPIO_INIT_OUTPUT(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_ANALOG(SDC_IMON_GPIO_Port, SDC_IMON_Pin),
    // Fan Switches
    GPIO_INIT_OUTPUT(FAN_1_CTRL_GPIO_Port, FAN_1_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(FAN_1_NFLT_GPIO_Port, FAN_1_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_ANALOG(FAN_1_CS_GPIO_Port, FAN_1_CS_Pin),
    GPIO_INIT_OUTPUT(FAN_2_CTRL_GPIO_Port, FAN_2_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(FAN_2_NFLT_GPIO_Port, FAN_2_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_ANALOG(FAN_2_CS_GPIO_Port, FAN_2_CS_Pin),
    // Main Module
    // Disable software control GPIO_INIT_OUTPUT(MAIN_CTRL_GPIO_Port, MAIN_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(MAIN_NFLT_GPIO_Port, MAIN_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_ANALOG(MAIN_CS_GPIO_Port, MAIN_CS_Pin),
    // Dashboard
    // Disable software control GPIO_INIT_OUTPUT(DASH_CTRL_GPIO_Port, DASH_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(DASH_NFLT_GPIO_Port, DASH_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_ANALOG(DASH_CS_GPIO_Port, DASH_CS_Pin),
    // ABox
    // Disable software control GPIO_INIT_OUTPUT(ABOX_CTRL_GPIO_Port, ABOX_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(ABOX_NFLT_GPIO_Port, ABOX_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_ANALOG(ABOX_CS_GPIO_Port, ABOX_CS_Pin),
    // Bullet
    GPIO_INIT_OUTPUT(BLT_CTRL_GPIO_Port, BLT_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(BLT_NFLT_GPIO_Port, BLT_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    // 5V Critical Switch
    // Disable software control GPIO_INIT_OUTPUT(CRIT_5V_CTRL_GPIO_Port, CRIT_5V_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(CRIT_5V_NFLT_GPIO_Port, CRIT_5V_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    // 5V Non-Critical Switch
    GPIO_INIT_OUTPUT(NCRIT_5V_CTRL_GPIO_Port, NCRIT_5V_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(NCRIT_5V_NFLT_GPIO_Port, NCRIT_5V_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    // DAQ
    GPIO_INIT_OUTPUT(DAQ_CTRL_GPIO_Port, DAQ_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(DAQ_NFLT_GPIO_Port, DAQ_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    // 5V Fan
    GPIO_INIT_OUTPUT(FAN_5V_CTRL_GPIO_Port, FAN_5V_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(FAN_5V_NFLT_GPIO_Port, FAN_5V_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    // LV Battery BMS
    GPIO_INIT_INPUT(LV_BMS_STAT_GPIO_Port, LV_BMS_STAT_Pin, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_USART3TX_PC10,
    GPIO_INIT_USART3RX_PC11,
    // LV Status
    GPIO_INIT_ANALOG(LV_24V_V_SENSE_GPIO_Port, LV_24V_V_SENSE_Pin),
    GPIO_INIT_ANALOG(LV_24V_I_SENSE_GPIO_Port, LV_24V_I_SENSE_Pin),
    GPIO_INIT_ANALOG(LV_5V_V_SENSE_GPIO_Port, LV_5V_V_SENSE_Pin),
    GPIO_INIT_ANALOG(LV_5V_I_SENSE_GPIO_Port, LV_5V_I_SENSE_Pin),
    GPIO_INIT_ANALOG(LV_3V3_V_SENSE_GPIO_Port, LV_3V3_V_SENSE_Pin),
    GPIO_INIT_ANALOG(EXTERNAL_THERM_GPIO_Port, EXTERNAL_THERM_Pin),
};

/* ADC Configuration */
ADCInitConfig_t adc_config = {
    .clock_prescaler = ADC_CLK_PRESC_6, // Desire ADC clock to be 30MHz (upper bound), clocked from APB2 (160/6=27MHz)
    .resolution      = ADC_RES_12_BIT,
    .data_align      = ADC_DATA_ALIGN_RIGHT,
    .cont_conv_mode  = true,
    .adc_number      = 1,
    .dma_mode        = ADC_DMA_CIRCULAR
};


/* SPI Configuration */
dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

SPI_InitConfig_t spi_config = {
    .data_len  = 8,
    .nss_sw = false,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI1
};

/* With 11 items, 16 prescaler, and 640 sample time, each channel gets read every 1.4ms */
volatile ADCReadings_t adc_readings;
ADCChannelConfig_t adc_channel_config[] = {
    {.channel=PUMP_1_IMON_ADC_CHNL,    .rank=1,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=PUMP_2_IMON_ADC_CHNL,    .rank=2,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=AUX_HP_IMON_ADC_CHNL,    .rank=3,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=SDC_IMON_ADC_CHNL,       .rank=4,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=FAN_1_CS_ADC_CHNL,       .rank=5,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=FAN_2_CS_ADC_CHNL,       .rank=6,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=MAIN_CS_ADC_CHNL,        .rank=7,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=DASH_CS_ADC_CHNL,        .rank=8,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=ABOX_CS_ADC_CHNL,        .rank=9,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_24V_V_SENSE_ADC_CHNL, .rank=10, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_24V_I_SENSE_ADC_CHNL, .rank=11, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_5V_V_SENSE_ADC_CHNL,  .rank=12, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_5V_I_SENSE_ADC_CHNL,  .rank=13, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_3V3_V_SENSE_ADC_CHNL, .rank=14, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=EXTERNAL_THERM_ADC_CHNL, .rank=15, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=INTERNAL_THERM_ADC_CHNL, .rank=16, .sampling_time=ADC_CHN_SMP_CYCLES_480},
};
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &adc_readings,
            sizeof(adc_readings) / sizeof(adc_readings.lv_3v3_v_sense), 0b01);

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =160000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

void HardFault_Handler();
void preflightAnimation();
void preflightChecks(void);
void canTxUpdate();
void heatBeatLED();
void sendtestmsg();

q_handle_t q_tx_can;
q_handle_t q_rx_can;

int main()
{
    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));

    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    if(!PHAL_SPI_init(&spi_config))
    {
        HardFault_Handler();
    }

    PHAL_writeGPIO(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, 1);
    PHAL_writeGPIO(DAQ_CTRL_GPIO_Port, DAQ_CTRL_Pin, 1);

    PHAL_writeGPIO(LED_CTRL_BLANK_GPIO_Port, LED_CTRL_BLANK_Pin, 1);

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    configureAnim(preflightAnimation, preflightChecks, 40, 1500);

    /* Schedule Periodic tasks here */
    taskCreate(heatBeatLED, 500);
    taskCreate(sendtestmsg, 100);
    taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);
    taskCreate(LED_periodic, 500);
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);
    schedStart();
    return 0;
}

void preflightChecks(void) {
    static uint8_t state;

    switch (state++)
    {
        case 0:
            if(!PHAL_initCAN(CAN1, false))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
           break;
        case 1:
           initCANParse(&q_rx_can);
           if(daqInit(&q_tx_can))
               HardFault_Handler();
           break;
        default:
            registerPreflightComplete(1);
            state = 255; // prevent wrap around
    }
}

void preflightAnimation(void) {
    static uint32_t time;
    static int led_number;
    static bool led_decrement = false;

    PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 0);
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 0);
    PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);

    switch (time++ % 6)
    {
        case 0:
        case 5:
            PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 1);
            break;
        case 1:
        case 4:
            PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
            break;
        case 2:
        case 3:
            PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
            break;
    }

    if(led_number < 14 && !led_decrement)
    {
        led_number++;
        LED_control(led_number, ON);
    }
    else if(led_number >= 14 && !led_decrement)
    {
        led_decrement = true;
    }
    else
    {
        led_number--;
        LED_control(led_number, OFF);
        led_number = (led_number == 0) ? 0 : led_number;
    }
    
    
}

void heatBeatLED()
{
    PHAL_toggleGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
}

void sendtestmsg()
{
    static uint8_t test_1;
    static uint8_t test_2;
    static uint8_t test_3;

    if (test_3 == 0)
        test_3 = 1;

    SEND_PDU_TEST(q_tx_can, test_1++, test_2, test_3);

    test_2 += 5;
    test_3 *= 2;

}

void canTxUpdate()
{
   CanMsgTypeDef_t tx_msg;
   if (qReceive(&q_tx_can, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
   {
       PHAL_txCANMessage(&tx_msg);
   }
}

void CAN1_RX0_IRQHandler()
{
   if (CAN1->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
       CAN1->RF0R &= !(CAN_RF0R_FOVR0);

   if (CAN1->RF0R & CAN_RF0R_FULL0) // FIFO Full
       CAN1->RF0R &= !(CAN_RF0R_FULL0);

   if (CAN1->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
   {
       CanMsgTypeDef_t rx;
       rx.Bus = CAN1;

       // Get either StdId or ExtId
       if (CAN_RI0R_IDE & CAN1->sFIFOMailBox[0].RIR)
       {
         rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN1->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
       }
       else
       {
         rx.StdId = (CAN_RI0R_STID & CAN1->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;
       }

       rx.DLC = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

       rx.Data[0] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 0)  & 0xFF;
       rx.Data[1] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 8)  & 0xFF;
       rx.Data[2] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
       rx.Data[3] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
       rx.Data[4] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 0)  & 0xFF;
       rx.Data[5] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 8)  & 0xFF;
       rx.Data[6] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
       rx.Data[7] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

       CAN1->RF0R |= (CAN_RF0R_RFOM0);
       qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
   }
}

void pdu_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
   if (can_data.pdu_bl_cmd.cmd == BLCMD_RST)
       Bootloader_ResetForFirmwareDownload();
}

void HardFault_Handler()
{
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
    while(1)
    {
        __asm__("nop");
    }
}