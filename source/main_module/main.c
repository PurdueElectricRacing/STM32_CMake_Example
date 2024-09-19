/* System Includes */
#include "common/bootloader/bootloader_common.h"
#include "common/common_defs/common_defs.h"
#include "common/faults/faults.h"
// #include "common/modules/wheel_speeds/wheel_speeds.h"
#include "common/phal_F4_F7/adc/adc.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/plettenberg/plettenberg.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/plettenberg/plettenberg.h"
#include "common/psched/psched.h"
#include "common/queue/queue.h"
#include "common/amk/amk.h"

/* TODO: Move CAN2 stuff here since can parse base is dumb */
#include "common/phal_F4_F7/can/can.h"

/* Module Includes */
#include "car.h"
#include "can_parse.h"
#include "cooling.h"
#include "daq.h"
#include "main.h"

GPIOInitConfig_t gpio_config[] = {
    // Internal Status Indicators
    GPIO_INIT_OUTPUT(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_OUTPUT_LOW_SPEED),

    // External Status Indicators
    GPIO_INIT_OUTPUT(BRK_LIGHT_GPIO_Port, BRK_LIGHT_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(BRK_BUZZER_STAT_GPIO_Port, BRK_BUZZER_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),
    // GPIO_INIT_INPUT(TSAL_LVAL_STAT_GPIO_Port, TSAL_LVAL_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),

    // CAN
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12,

    GPIO_INIT_CAN2RX_PB12,
    GPIO_INIT_CAN2TX_PB13,

    //SPI Peripherals
    GPIO_INIT_SPI1_SCK_PA5,
    GPIO_INIT_SPI1_MISO_PA6,
    GPIO_INIT_SPI1_MOSI_PA7,
    GPIO_INIT_OUTPUT(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),

    GPIO_INIT_OUTPUT(EEPROM_nWP_GPIO_Port, EEPROM_nWP_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(EEPROM_NSS_GPIO_Port, EEPROM_NSS_Pin, GPIO_OUTPUT_LOW_SPEED),

    // Shutdown Circuits
    GPIO_INIT_OUTPUT(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S0_GPIO_Port, SDC_MUX_S0_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S1_GPIO_Port, SDC_MUX_S1_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S2_GPIO_Port, SDC_MUX_S2_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S3_GPIO_Port, SDC_MUX_S3_Pin, GPIO_OUTPUT_LOW_SPEED),

    GPIO_INIT_INPUT(SDC_MUX_DATA_GPIO_Port, SDC_MUX_DATA_Pin, GPIO_INPUT_OPEN_DRAIN),

    // HV Bus Information
    // GPIO_INIT_ANALOG(V_MC_SENSE_GPIO_Port, V_MC_SENSE_Pin),
    // GPIO_INIT_ANALOG(V_BAT_SENSE_GPIO_Port, V_BAT_SENSE_Pin),
    GPIO_INIT_INPUT(BMS_STAT_GPIO_Port, BMS_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_INPUT(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),

    // Wheel Speed
    GPIO_INIT_AF(MOTOR_R_WS_GPIO_Port, MOTOR_R_WS_Pin, MOTOR_R_WS_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(MOTOR_L_WS_GPIO_Port, MOTOR_L_WS_Pin, MOTOR_L_WS_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),

    // Shock Pots
    GPIO_INIT_ANALOG(SHOCK_POT_L_GPIO_Port, SHOCK_POT_L_Pin),
    GPIO_INIT_ANALOG(SHOCK_POT_R_GPIO_Port, SHOCK_POT_R_Pin),

    // Load Sensor
    GPIO_INIT_ANALOG(LOAD_L_GPIO_Port, LOAD_L_Pin),
    GPIO_INIT_ANALOG(LOAD_R_GPIO_Port, LOAD_R_Pin),

    // Thermistor Analog Multiplexer
    GPIO_INIT_OUTPUT(THERM_MUX_S0_GPIO_Port, THERM_MUX_S0_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(THERM_MUX_S1_GPIO_Port, THERM_MUX_S1_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(THERM_MUX_S2_GPIO_Port, THERM_MUX_S2_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_ANALOG(THERM_MUX_D_GPIO_Port, THERM_MUX_D_Pin)
};

/* ADC Configuration */
ADCInitConfig_t adc_config = {
    .clock_prescaler = ADC_CLK_PRESC_6,
    .resolution      = ADC_RES_12_BIT,
    .data_align      = ADC_DATA_ALIGN_RIGHT,
    .cont_conv_mode  = true,
    .dma_mode        = ADC_DMA_CIRCULAR,
    .adc_number      = 1
};

// TODO: Update this comment with the correct sample time
/* With 11 items, 16 prescaler, and 640 sample time, each channel gets read every 1.4ms */
volatile ADCReadings_t adc_readings;
ADCChannelConfig_t adc_channel_config[] = {
    {.channel=V_MC_SENSE_ADC_CHNL,     .rank=1,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=V_BAT_SENSE_ADC_CHNL,    .rank=2,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=SHOCK_POT_L_ADC_CHNL,    .rank=3,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=SHOCK_POT_R_ADC_CHNL,    .rank=4,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=THERM_MUX_D_ADC_CHNL,    .rank=5,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LOAD_L_ADC_CHNL,         .rank=6,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LOAD_R_ADC_CHNL,         .rank=7,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=INTERNAL_THERM_ADC_CHNL, .rank=8,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
};
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &adc_readings,
            sizeof(adc_readings) / sizeof(adc_readings.v_mc), 0b01);

/* SPI Configuration */
// dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
// dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

// SPI_InitConfig_t spi_config = {
//     .data_len  = 8,
//     .nss_sw = false,
//     .nss_gpio_port = EEPROM_NSS_GPIO_Port,
//     .nss_gpio_pin = EEPROM_NSS_Pin,
//     .rx_dma_cfg = &spi_rx_dma_config,
//     .tx_dma_cfg = &spi_tx_dma_config,
//     .periph = SPI1
// };




// extern uint32_t APB1ClockRateHz;
// extern uint32_t APB2ClockRateHz;
// extern uint32_t AHBClockRateHz;
// extern uint32_t PLLClockRateHz;

// #define TargetCoreClockrateHz 96000000
// ClockRateConfig_t clock_config = {
//     .system_source              =SYSTEM_CLOCK_SRC_PLL,
//     .pll_src                    =PLL_SRC_HSI16,
//     .vco_output_rate_target_hz  =192000000,
//     .system_clock_target_hz     =TargetCoreClockrateHz,
//     .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
//     .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
//     .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
// };

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

#define TargetCoreClockrateHz 144000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =288000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
};

/* Function Prototypes */
void preflightAnimation(void);
void preflightChecks(void);
void heartBeatLED();
// void usartTxUpdate(void);
// void usartIdleIRQ(volatile usart_init_t *huart, volatile usart_rx_buf_t *rx_buf);
void send_fault(uint16_t, bool);
extern void HardFault_Handler();
void interpretLoadSensor(void);
float voltToForce(uint16_t load_read);

// q_handle_t q_tx_usart_l;
// q_handle_t q_tx_usart_r;

uint16_t num_failed_msgs_r;
uint16_t num_failed_msgs_l;

extern q_handle_t q_tx_can2_s[CAN_TX_MAILBOX_CNT];
extern uint32_t can2_mbx_last_send_time[CAN_TX_MAILBOX_CNT];

void can2TxUpdate(void)
{
    CanMsgTypeDef_t tx_msg;
    for (uint8_t i = 0; i < CAN_TX_MAILBOX_CNT; ++i)
    {
        if(PHAL_txMailboxFree(CAN2, i))
        {
            if (qReceive(&q_tx_can2_s[i], &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
            {
                PHAL_txCANMessage(&tx_msg, i);
                can2_mbx_last_send_time[i] = sched.os_ticks;
            }
        }
        else if (sched.os_ticks - can2_mbx_last_send_time[i] > CAN_TX_TIMEOUT_MS)
        {
            PHAL_txCANAbort(CAN2, i); // aborts tx and empties the mailbox
            can_stats.can_peripheral_stats[CAN2_IDX].tx_fail++;
        }
    }
}

int main(void){
    /* Data Struct Initialization */
    // qConstruct(&q_tx_usart_l, MC_MAX_TX_LENGTH);
    // qConstruct(&q_tx_usart_r, MC_MAX_TX_LENGTH);

    /* HAL Initialization */
    PHAL_trimHSI(HSI_TRIM_MAIN_MODULE);
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    PHAL_writeGPIO(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, 1);

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    configureAnim(preflightAnimation, preflightChecks, 60, 750);

    taskCreate(coolingPeriodic, 50);
    taskCreate(heartBeatLED, 500);
    taskCreate(monitorSDCPeriodic, 20);
    taskCreate(carHeartbeat, 500);
    taskCreate(carPeriodic, 15);
    taskCreate(interpretLoadSensor, 15);
    taskCreate(updateSDCFaults, 300);
    taskCreate(heartBeatTask, 100);
    taskCreate(send_shockpots, 15);
    taskCreate(parseMCDataPeriodic, MC_LOOP_DT);
    taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);
    // taskCreate(memFg, MEM_FG_TIME);
    taskCreateBackground(canTxUpdate);
    // taskCreateBackground(can2TxUpdate);
    taskCreateBackground(canRxUpdate);
    // taskCreateBackground(usartTxUpdate);
    // taskCreateBackground(memBg);
    // uint8_t i = 0;
    // calibrateSteeringAngle(&i);
    // for (uint8_t i = 0; i < 10; i++)
    //     SEND_LWS_CONFIG(0x05, 0, 0); // reset cal
    // SEND_LWS_CONFIG(0x03, 0, 0); // start new

    schedStart();

    return 0;
}

void preflightChecks(void) {
    static uint8_t state;

    switch (state++)
    {
        /* TODO: Change this to init AMK CAN */
        case 0:
            if(!PHAL_initCAN(CAN1, false, VCAN_BPS))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
            break;
        case 1:
            if(!PHAL_initCAN(CAN2, false, VCAN_BPS))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
            if(!PHAL_initCAN(CAN2, false, MCAN_BPS))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN2_RX0_IRQn);
            // spi_config.data_rate = APB2ClockRateHz / 16; // 5 MHz
            // if (!PHAL_SPI_init(&spi_config))
            //     HardFault_Handler();
            // if (initMem(EEPROM_nWP_GPIO_Port, EEPROM_nWP_Pin, &spi_config, 1, 1) != E_SUCCESS)
            //     HardFault_Handler();
           break;
        case 2:
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
           break;
        case 3:
            break;
        case 4:
           /* Module Initialization */
           carInit();
           coolingInit();
           break;
       case 5:
           initCANParse();
           if(daqInit(&q_tx_can[CAN1_IDX][CAN_MAILBOX_LOW_PRIO]))
               HardFault_Handler();
            initFaultLibrary(FAULT_NODE_NAME, &q_tx_can[CAN1_IDX][CAN_MAILBOX_HIGH_PRIO], ID_FAULT_SYNC_MAIN_MODULE);
           break;
        default:
            registerPreflightComplete(1);
            state = 255; // prevent wrap around
    }
}

void preflightAnimation(void) {
    static uint32_t time;

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
}

void heartBeatLED(void)
{
    static uint8_t trig;
    // TODO: fix HB LED
    PHAL_toggleGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
         PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    else PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);

    // Send every other time (1000 ms)
    if (trig) {
        SEND_MCU_STATUS(sched.skips, (uint8_t) sched.fg_time.cpu_use,
                                           (uint8_t) sched.bg_time.cpu_use,
                                           sched.error);
    }
    else
    {
        SEND_MAIN_MODULE_CAN_STATS(can_stats.can_peripheral_stats[CAN1_IDX].tx_of, can_stats.can_peripheral_stats[CAN2_IDX].tx_of,
                                   can_stats.can_peripheral_stats[CAN1_IDX].tx_fail, can_stats.can_peripheral_stats[CAN2_IDX].tx_fail,
                                   can_stats.rx_of, can_stats.can_peripheral_stats[CAN1_IDX].rx_overrun, can_stats.can_peripheral_stats[CAN2_IDX].rx_overrun);
    }
    trig = !trig;
}

#define SCALE_F = (1 + (3.4/6.6))
float voltToForce(uint16_t load_read) {
    /*
    //Return in newtons
    float v_out_load_l = adc_readings.load_l / 4095 * 3.3;
    float v_out_load_r = adc_readings.load_r / 4095 * 3.3;
    //voltage -> weight
    //V_out = (V_in * R_2) / (R_1 + R_2)
    //Solve for V_in
    //R_1 = 3.4K
    //R_2 = 6.6K
    float v_in_load_l = (v_out_load_l * 10) / 6.6;
    float v_in_load_r = (v_out_load_r * 10) / 6.6;
    //voltage * 100 = mass
    //weight (in newtons) = mass * g
    float force_load_l = v_in_load_l * 100 * g;
    float force_load_r = v_in_load_r * 100 * g;
    */
    float g = 9.8;
    // float val = ((load_read / 4095.0 * 3.3) * 10.0)
    float val = ((load_read / 4095.0 * 3.3) * (1.0 + (3.4/6.6)));
    // return ( val / 6.6) * 100.0 * g;
    return val * 100.0 * g;
}

void interpretLoadSensor(void) {
    float force_load_l = voltToForce(adc_readings.load_l);
    float force_load_r = voltToForce(adc_readings.load_r);
    //send a can message w/ minimal force info
    //every 15 milliseconds
    SEND_LOAD_SENSOR_READINGS(force_load_l, force_load_r);


}

/* CAN Message Handling */
void CAN1_RX0_IRQHandler()
{
    canParseIRQHandler(CAN1);
}

void CAN2_RX0_IRQHandler()
{
    canParseIRQHandler(CAN2);
}

void main_module_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.main_module_bl_cmd.cmd == BLCMD_RST)
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
