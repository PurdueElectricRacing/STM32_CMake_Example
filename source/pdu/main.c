/* System Includes */
#include "common/bootloader/bootloader_common.h"
#include "common/common_defs/common_defs.h"
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
#include "auto_switch.h"
#include "can_parse.h"
#include "daq.h"
#include "fan_control.h"
#include "led.h"
#include "cooling.h"
#include "flow_rate.h"

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
    GPIO_INIT_INPUT(DASH_NFLT_GPIO_Port, DASH_NFLT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_ANALOG(DASH_CS_GPIO_Port, DASH_CS_Pin),
    // ABox
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
            sizeof(adc_readings) / sizeof(adc_readings.lv_24_v_sense), 0b01);

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
void heatBeatLED();
void send_iv_readings();
void send_flowrates();

// To correctly execute preflight algorithm
uint8_t led_anim_complete;

int main()
{
    /* Data Struct init */
    PHAL_trimHSI(HSI_TRIM_PDU);
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    if(!PHAL_initADC(ADC1, &adc_config, adc_channel_config,
        sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
    {
        HardFault_Handler();
    }
    if(!PHAL_initDMA(&adc_dma_config))
    {
        HardFault_Handler();
    }
    PHAL_startTxfer(&adc_dma_config);
    PHAL_startADC(ADC1);
    led_anim_complete = 0;

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    configureAnim(preflightAnimation, preflightChecks, 20, 750);

    /* Schedule Periodic tasks here */
    taskCreate(heatBeatLED, 500);
    taskCreate(heartBeatTask, 100);
    taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);
    taskCreate(LED_periodic, 500);
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);
    taskCreate(autoSwitchPeriodic, 15);
    taskCreate(update_cooling_periodic, 100);
    taskCreate(send_iv_readings, 500);
    taskCreate(checkSwitchFaults, 100);
    taskCreate(send_flowrates, 200);
    schedStart();
    return 0;
}

void preflightChecks(void) {
    static uint8_t state;

    switch (state++)
    {
        case 0:
            if(!PHAL_initCAN(CAN1, false, VCAN_BPS))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
           break;
        case 1:
           initCANParse();
           if(daqInit(&q_tx_can[CAN1_IDX][CAN_MAILBOX_LOW_PRIO]))
               HardFault_Handler();
           break;
        case 2:
            if(!PHAL_SPI_init(&spi_config))
            {
                HardFault_Handler();
            }
            PHAL_writeGPIO(LED_CTRL_BLANK_GPIO_Port, LED_CTRL_BLANK_Pin, 1);
            break;
        case 3:
            fanControlInit();
            break;
        case 4:
            coolingInit();
            flowRateInit();
            break;
        case 5:
            initFaultLibrary(FAULT_NODE_NAME, &q_tx_can[CAN1_IDX][CAN_MAILBOX_HIGH_PRIO], ID_FAULT_SYNC_PDU);
            break;
        default:
            if (led_anim_complete)
            {
                // Initialize default 'ON' rails
                setSwitch(SW_SDC, 1);
                setSwitch(SW_DAQ, 1);
                setSwitch(SW_NCRIT_5V, 1);
                setSwitch(SW_MAIN, 1);
                setSwitch(SW_ABOX, 1);
                setSwitch(SW_DASH, 1);
                setSwitch(SW_CRIT_5V, 1);
                setSwitch(SW_BLT, 1);
                registerPreflightComplete(1);
            }
            state = 255; // prevent wrap around
            break;
    }
}

void preflightAnimation(void) {
    static uint32_t time;
    static int led_number;
    static bool led_decrement = false;

    PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 0);
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 0);
    PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);

    switch (time++ % 24)
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

    if(led_number < MAX_NUM_LED && !led_decrement)
    {
        led_number++;
        LED_control(led_number, LED_ON);
    }
    else if(led_number >= MAX_NUM_LED && !led_decrement)
    {
        led_decrement = true;
    }
    else
    {
        led_number--;
        LED_control(led_number, LED_OFF);
        if (led_number == 0)
        {
            led_anim_complete = 1;
        }
    }


}

void send_flowrates()
{
    SEND_FLOWRATES(getFlowRate1(), getFlowRate2());
}

void heatBeatLED()
{
    PHAL_toggleGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
         PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    else PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);

    static uint8_t trig;
    if (trig) SEND_PDU_CAN_STATS(can_stats.can_peripheral_stats[CAN1_IDX].tx_of,
                                 can_stats.can_peripheral_stats[CAN1_IDX].tx_fail,
                                 can_stats.rx_of, can_stats.can_peripheral_stats[CAN1_IDX].rx_overrun);
    trig = !trig;
}

void CAN1_RX0_IRQHandler()
{
    canParseIRQHandler(CAN1);
}

void pdu_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
   if (can_data.pdu_bl_cmd.cmd == BLCMD_RST)
       Bootloader_ResetForFirmwareDownload();
}

void send_iv_readings() {
    // Set LV Batt faults
    // setFault(ID_LV_BATT_FIFTY_FAULT, auto_switches.voltage.in_24v);
    setFault(ID_LV_GETTING_LOW_FAULT, auto_switches.voltage.in_24v);
    setFault(ID_LV_CRITICAL_LOW_FAULT, auto_switches.voltage.in_24v);
    // Send CAN messages containing voltage and current data
    SEND_V_RAILS(auto_switches.voltage.in_24v, auto_switches.voltage.out_5v, auto_switches.voltage.out_3v3);
    SEND_RAIL_CURRENTS(auto_switches.current[CS_24V], auto_switches.current[CS_5V]);
    SEND_PUMP_AND_FAN_CURRENT(auto_switches.current[SW_PUMP_1], auto_switches.current[SW_PUMP_2], auto_switches.current[SW_FAN_1], auto_switches.current[SW_FAN_2]);
    SEND_OTHER_CURRENTS(auto_switches.current[SW_SDC], auto_switches.current[SW_AUX], auto_switches.current[SW_DASH], auto_switches.current[SW_ABOX], auto_switches.current[SW_MAIN]);
}

void HardFault_Handler()
{
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
    while(1)
    {
        __asm__("nop");
    }
}
