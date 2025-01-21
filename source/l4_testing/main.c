/* System Includes */
#include "stm32l432xx.h"
#include "common/bootloader/bootloader_common.h"
#include "common/psched/psched.h"
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/rcc/rcc.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/adc/adc.h"
#include "common/phal_L4/usart/usart.h"
#include "common/phal_L4/tim/tim.h"
#include "common/phal_L4/dma/dma.h"
#include "common/eeprom_spi/eeprom_spi.h"
#include <math.h>

/* Module Includes */
#include "main.h"
#include "can_parse.h"
#include "daq.h"
#include "common/modules/wheel_speeds/wheel_speeds.h"

#include "common/faults/faults.h"


GPIOInitConfig_t gpio_config[] = {
  GPIO_INIT_CANRX_PA11,
  GPIO_INIT_CANTX_PA12,
//   GPIO_INIT_USART1TX_PA9,
//   GPIO_INIT_USART1RX_PA10,
// #if EEPROM_ENABLED
//   GPIO_INIT_I2C3_SCL_PA7,
//   GPIO_INIT_I2C3_SDA_PB4,
// #endif
  GPIO_INIT_OUTPUT(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_OUTPUT_LOW_SPEED),
//   GPIO_INIT_OUTPUT(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_OUTPUT_LOW_SPEED),
//   GPIO_INIT_OUTPUT(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_OUTPUT_LOW_SPEED),
  GPIO_INIT_OUTPUT(LED1_GPIO_Port, LED1_Pin, GPIO_OUTPUT_LOW_SPEED),
  GPIO_INIT_AF(GPIOA, 0, 1, GPIO_OUTPUT_ULTRA_SPEED, GPIO_TYPE_AF, GPIO_INPUT_OPEN_DRAIN),
  GPIO_INIT_AF(GPIOA, 1, 1, GPIO_OUTPUT_ULTRA_SPEED, GPIO_TYPE_AF, GPIO_INPUT_OPEN_DRAIN),
//   GPIO_INIT_INPUT(BUTTON_1_GPIO_Port, BUTTON_1_Pin, GPIO_INPUT_PULL_DOWN),
//   GPIO_INIT_AF(TIM1_GPIO_Port, TIM1_Pin, TIM1_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_TYPE_AF, GPIO_INPUT_PULL_UP),
//   GPIO_INIT_AF(TIM2_GPIO_Port, TIM2_Pin, TIM2_AF, GPIO_OUTPUT_ULTRA_SPEED, GPIO_TYPE_AF, GPIO_INPUT_PULL_UP),
//   GPIO_INIT_ANALOG(POT_GPIO_Port, POT_Pin),
//   GPIO_INIT_ANALOG(POT2_GPIO_Port, POT2_Pin),
//   GPIO_INIT_ANALOG(POT3_GPIO_Port, POT3_Pin)
    // SPI
    GPIO_INIT_AF(SPI_SCLK_GPIO_Port, SPI_SCLK_Pin,  5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin,  5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MISO_GPIO_Port, SPI_MISO_Pin,  5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_OUTPUT(SPI_CS_EEPROM_GPIO_Port, SPI_CS_EEPROM_Pin, GPIO_OUTPUT_HIGH_SPEED),
};

ADCInitConfig_t adc_config = {
    .clock_prescaler = ADC_CLK_PRESC_6,
    .resolution      = ADC_RES_12_BIT,
    .data_align      = ADC_DATA_ALIGN_RIGHT,
    .cont_conv_mode  = true,
    .overrun         = true,
    .dma_mode        = ADC_DMA_CIRCULAR
};

ADCChannelConfig_t adc_channel_config[] = {
    {.channel=POT_ADC_Channel, .rank=1, .sampling_time=ADC_CHN_SMP_CYCLES_2_5},
    {.channel=POT2_ADC_Channel, .rank=2, .sampling_time=ADC_CHN_SMP_CYCLES_2_5},
    {.channel=POT3_ADC_Channel, .rank=3, .sampling_time=ADC_CHN_SMP_CYCLES_2_5}
};

usart_init_t huart1 = {
    .baud_rate = 9600,
    .word_length = WORD_8,
    .stop_bits = SB_ONE,
    .parity = PT_NONE,
    .mode = MODE_TX_RX,
    .hw_flow_ctl = HW_DISABLE,
    .ovsample = OV_16,
    .obsample = OB_DISABLE,
    .adv_feature = {
                        .auto_baud = false,
                        .ab_mode = AB_START,
                        .tx_inv = false,
                        .rx_inv = false,
                        .data_inv = false,
                        .tx_rx_swp = false,
                        .overrun = false,
                        .dma_on_rx_err = false,
                        .msb_first = false,
                   },
};

dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

SPI_InitConfig_t spi_config = {
    .data_len  = 8,
    .nss_sw = false,
    .nss_gpio_port = SPI_CS_EEPROM_GPIO_Port,
    .nss_gpio_pin = SPI_CS_EEPROM_Pin,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config
};

static uint16_t adc_conversions[3];
uint8_t charge_enable;
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) adc_conversions, 3, 0b01);

WheelSpeed_t left_wheel =  {.tim=TIM2, .invert=false};
WheelSpeed_t right_wheel = {.tim=TIM2, .invert=false};
// TODO: test invert
WheelSpeeds_t wheel_speeds = {.l=&left_wheel, .r=&right_wheel};

#define TargetCoreClockrateHz 80000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_MSI,
    .msi_output_rate_target_hz  =16000000,
    .vco_output_rate_target_hz  =160000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (4)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (4)),
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

/* Function Prototypes */
void myCounterTest();
void usartTXTest();
void canReceiveTest();
void adcConvert();
void coolingPeriodic();
void canSendTest();
void Error_Handler();
void SysTick_Handler();
void canTxUpdate();
void setRed(uint8_t* on);
void setGreen(uint8_t* on);
void setBlue(uint8_t* on);
void readRed(uint8_t* on);
void readGreen(uint8_t* on);
void readBlue(uint8_t* on);
void ledBlink();
extern void HardFault_Handler();
void init_ADC();
void testCaller(void);

q_handle_t q_tx_can_0, q_tx_can_1, q_tx_can_2;
q_handle_t q_rx_can;

dma_init_t usart_tx_dma_config = USART1_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART1_RXDMA_CONT_CONFIG(NULL, 2);

uint8_t my_counter = 0;
uint16_t my_counter2 = 85; // Warning: daq variables with eeprom capability may
                           // initialize to something else
uint8_t green_on = 0;

uint64_t faults = 0;

int main (void)
{
    // Main stack pointer is saved as the first entry in the .isr_entry
    Bootloader_ConfirmApplicationLaunch();
    /* Data Struct init */
    qConstruct(&q_tx_can_0, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_tx_can_1, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_tx_can_2, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));


    huart1.tx_dma_cfg = &usart_tx_dma_config;
    huart1.rx_dma_cfg = &usart_rx_dma_config;

    /* HAL Initilization */
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    volatile uint16_t var = 0;

    // if(!PHAL_initCAN(CAN1, false, 250000))
    // {
    //     HardFault_Handler();
    // }
//     if(!PHAL_initUSART(USART1, &huart1, APB2ClockRateHz))
//     {
//         HardFault_Handler();
//     }
// #if EEPROM_ENABLED
//     if(!PHAL_initI2C(I2C3))
//     {
//         HardFault_Handler();
//     }
// #endif
//     if(!PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
//     {
//         HardFault_Handler();
//     }
//     if(!PHAL_initDMA(&adc_dma_config))
//     {
//         HardFault_Handler();
//     }
//     PHAL_startTxfer(&adc_dma_config);
//     PHAL_startADC(ADC1);
//     if(!PHAL_initPWMIn(TIM1, APB2ClockRateHz / TIM_CLOCK_FREQ, TI1FP1))
//     {
//         HardFault_Handler();
//     }
//     if(!PHAL_initPWMChannel(TIM1, CC1, CC_INTERNAL, false))
//     {
//         HardFault_Handler();
//     }
    /*
    if(!PHAL_initPWMIn(TIM2, APB1ClockRateHz / TIM_CLOCK_FREQ, TI1FP1))
    {
        HardFault_Handler();
    }
    if(!PHAL_initPWMChannel(TIM2, CC1, CC_INTERNAL, false))
    {
        HardFault_Handler();
    }*/
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    // spi_config.data_rate = APB2ClockRateHz / 4; // 5 MHz
    // if (!PHAL_SPI_init(&spi_config))
    //     HardFault_Handler();
    // initMem(GPIOA, 3, &spi_config, 1, 1);
    // uint8_t  page[MICRO_PG_SIZE] = {0};
    // page[0] = 1;
    // page[1] = 2;
    // page[2] = 4;
    // writePage(0, page, 3);
    // while (1)
    // {
    //     readPage(0, page);
    // }

    // signify start of initialization
    // PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);

    /* Module init */
    // initCANParse(&q_rx_can);
    // wheelSpeedsInit();
    wheelSpeedsInit(&wheel_speeds);

    // linkReada(DAQ_ID_TEST_VAR, &my_counter);
    // linkReada(DAQ_ID_TEST_VAR2, &my_counter2);
    // linkWritea(DAQ_ID_TEST_VAR2, &my_counter2);
    // linkReadFunc(DAQ_ID_RED_ON, (read_func_ptr_t) readRed);
    // linkReadFunc(DAQ_ID_GREEN_ON, (read_func_ptr_t) readGreen);
    // linkReadFunc(DAQ_ID_BLUE_ON, (read_func_ptr_t) readBlue);
    // linkWriteFunc(DAQ_ID_RED_ON, (write_func_ptr_t) setRed);
    // linkWriteFunc(DAQ_ID_GREEN_ON, (write_func_ptr_t) setGreen);
    // linkWriteFunc(DAQ_ID_BLUE_ON, (write_func_ptr_t) setBlue);
    // if(daqInit(&q_tx_can_2))
    // {
    //     PHAL_writeGPIO(LED1_GPIO_Port, LED1_Pin, 1);
    //     HardFault_Handler();
    // }

    initFaultLibrary(FAULT_NODE_NAME, &q_tx_can_0, ID_FAULT_SYNC_TEST_NODE);

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    // taskCreate(usartTXTest, 1000);
    taskCreate(ledBlink, 500);
    taskCreate(wheelSpeedsPeriodic, 15);
    //Fault Stuff
    taskCreate(heartBeatTask, 100);
    //Fault Stuff
    // taskCreate(adcConvert, 50);
    // taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);
    // taskCreate(canSendTest, 50);
    // taskCreate(memFg, MEM_FG_TIME);
    // taskCreate(wheelSpeedsPeriodic, 15);
    // taskCreate(myCounterTest, 50);
    // taskCreateBackground(canTxUpdate);
    // taskCreateBackground(canRxUpdate);
    // taskCreateBackground(memBg);


    // signify end of initialization
    // PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
    schedStart();

    return 0;
}

void adcConvert()
{
    uint16_t pot1 = adc_conversions[0];
    SEND_ADC_VALUES(adc_conversions[0],
                              adc_conversions[1],
                              adc_conversions[2]);
}

void ledBlink()
{
    if (can_data.test_stale.stale)
    {
    //     PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, true);
    // }
    // else
    // {
    //     PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, false);
    // }
    // PHAL_toggleGPIO(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    // if (otherMCUsLatched()) {
        PHAL_writeGPIO(LED_GREEN_GPIO_Port, 3, 1);
        // forceFault(ID_TEST_FAULT_4_FAULT, 0);
    }
    else {
        PHAL_writeGPIO(LED_GREEN_GPIO_Port, 3, 0);
        // forceFault(ID_TEST_FAULT_4_FAULT, 1);
    }
}

uint8_t data_buf[26];
void usartTXTest()
{
    uint8_t i = 0;
    for (; i < 26; i++) data_buf[i] = 'a' + i;

    PHAL_usartTxDma(USART1, &huart1, (uint16_t *) data_buf, i);
}

void myCounterTest()
{
    my_counter += 1;
    if (my_counter >= 0xFF)
    {
        my_counter = 0;
    }
}

void setRed(uint8_t* on)
{
    PHAL_writeGPIO(LED_RED_GPIO_Port, LED_RED_Pin, *on);
}
void setBlue(uint8_t* on)
{
    PHAL_writeGPIO(LED_BLUE_GPIO_Port, LED_BLUE_Pin, *on);
}
void setGreen(uint8_t* on)
{
    PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, *on);
}
void readRed(uint8_t* on)
{
    *on = PHAL_readGPIO(LED_RED_GPIO_Port, LED_RED_Pin);
}
void readGreen(uint8_t* on)
{
    *on = PHAL_readGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}
void readBlue(uint8_t* on)
{
    *on = PHAL_readGPIO(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}

uint16_t counter = 1;
uint16_t counter2 = 1;
uint16_t adc_reading = 0;

void canSendTest()
{
    // SEND_TEST_MSG((int16_t) (500 * sin(((double) counter)/100)));
    SEND_TEST_MSG2(counter2);

    counter += 1;
    counter2 += 2;
    if (counter2 >= 0xFFF)
    {
        counter2 = 1;
    }

    // SEND_TEST_MSG3(counter2);
    // SEND_TEST_MSG4(counter2);
    // SEND_TEST_MSG5(0xFFF - counter2);
    // SEND_CAR_STATE(CAR_STATE_FLIPPED);
    //PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, green_on);
    PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, config.green_on);
}

void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    if (msg->IDE == 1)
    {
        // extended id, check hlp
        switch((msg->ExtId >> 26) & 0b111)
        {
            case 0:
            case 1:
                qSendToBack(&q_tx_can_0, msg);
                break;
            case 2:
            case 3:
                qSendToBack(&q_tx_can_1, msg);
                break;
            default:
                qSendToBack(&q_tx_can_2, msg);
                break;
        }
    }
    else
    {
        qSendToBack(&q_tx_can_0, &msg);
    }
}

void canTxUpdate(void)
{
    CanMsgTypeDef_t tx_msg;
    if(PHAL_txMailboxFree(CAN1, 0))
    {
        if (qReceive(&q_tx_can_0, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
        {
            PHAL_txCANMessage(&tx_msg, 0);
        }
    }
    if(PHAL_txMailboxFree(CAN1, 1))
    {
        if (qReceive(&q_tx_can_1, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
        {
            PHAL_txCANMessage(&tx_msg, 1);
        }
    }
    if(PHAL_txMailboxFree(CAN1, 2))
    {
        if (qReceive(&q_tx_can_2, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
        {
            PHAL_txCANMessage(&tx_msg, 2);
        }
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

        rx.Data[0] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 0) & 0xFF;
        rx.Data[1] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
        rx.Data[2] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 0) & 0xFF;
        rx.Data[5] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
        rx.Data[6] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        CAN1->RF0R     |= (CAN_RF0R_RFOM0);

        qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
    }
}

void l4_testing_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.l4_testing_bl_cmd.cmd == BLCMD_RST)
        Bootloader_ResetForFirmwareDownload();
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}