/* System Includes */
#include "stm32l432xx.h"
#include "common/psched/psched.h"
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/rcc/rcc.h"
#include "common/phal_L4/gpio/gpio.h"
#if EEPROM_ENABLED
#include "common/phal_L4/i2c/i2c.h"
#include "common/eeprom/eeprom.h"
#endif
#include <math.h>

/* Module Includes */
#include "main.h"
#include "can_parse.h"
#include "daq.h"


GPIOInitConfig_t gpio_config[] = {
  GPIO_INIT_CANRX_PA11,
  GPIO_INIT_CANTX_PA12,
#if EEPROM_ENABLED
  GPIO_INIT_I2C3_SCL_PA7,
  GPIO_INIT_I2C3_SDA_PB4,
#endif
  GPIO_INIT_OUTPUT(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_OUTPUT_LOW_SPEED),
  GPIO_INIT_OUTPUT(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_OUTPUT_LOW_SPEED),
  GPIO_INIT_OUTPUT(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_OUTPUT_LOW_SPEED),
  GPIO_INIT_INPUT(BUTTON_1_GPIO_Port, BUTTON_1_Pin, GPIO_INPUT_PULL_DOWN)
};

ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .system_clock_target_hz     =80000000,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =160000000,
    .ahb_clock_target_hz        =80000000,
    .apb1_clock_target_hz       =80000000,// / 16,
    .apb2_clock_target_hz       =80000000 / 16,
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

/* Function Prototypes */
void myCounterTest();
void canReceiveTest();
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

q_handle_t q_tx_can;
q_handle_t q_rx_can;

uint8_t my_counter = 0;
uint16_t my_counter2 = 85; // Warning: daq variables with eeprom capability may
                           // initialize to something else

int main (void)
{
    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));

    /* HAL Initilization */
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    if(!PHAL_initCAN(CAN1, false))
    {
        HardFault_Handler();
    }
#if EEPROM_ENABLED
    if(!PHAL_initI2C())
    {
        HardFault_Handler();
    }
#endif

    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    // signify start of initialization
    PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);

    /* Module init */
    initCANParse(&q_rx_can);

    linkReada(DAQ_ID_TEST_VAR, &my_counter);
    linkReada(DAQ_ID_TEST_VAR2, &my_counter2);
    linkWritea(DAQ_ID_TEST_VAR2, &my_counter2);
    linkReadFunc(DAQ_ID_RED_ON, (read_func_ptr_t) readRed);
    linkReadFunc(DAQ_ID_GREEN_ON, (read_func_ptr_t) readGreen);
    linkReadFunc(DAQ_ID_BLUE_ON, (read_func_ptr_t) readBlue);
    linkWriteFunc(DAQ_ID_RED_ON, (write_func_ptr_t) setRed);
    linkWriteFunc(DAQ_ID_GREEN_ON, (write_func_ptr_t) setGreen);
    linkWriteFunc(DAQ_ID_BLUE_ON, (write_func_ptr_t) setBlue);
    if(daqInit(&q_tx_can))
    {
        HardFault_Handler();
    }

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    taskCreate(canRxUpdate, RX_UPDATE_PERIOD);
    taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);
    taskCreate(canSendTest, 5);
    //taskCreate(canReceiveTest, 500);
    //taskCreate(myCounterTest, 50);
    //taskCreate(ledBlink, 500);
    taskCreateBackground(canTxUpdate);

    // signify end of initialization
    PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);

    schedStart();
    
    return 0;
}

void ledBlink()
{
    PHAL_toggleGPIO(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
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

void canSendTest()
{
#if EEPROM_ENABLED
    SEND_TEST_MSG(q_tx_can, (uint16_t) (500 * sin(((double) counter)/100) + 501));
    SEND_TEST_MSG2(q_tx_can, counter2);
    SEND_TEST_MSG3(q_tx_can, counter2);
    SEND_TEST_MSG4(q_tx_can, counter2);
    SEND_TEST_MSG5(q_tx_can, 0xFFF - counter2);
#else
    SEND_TEST_MSG_2(q_tx_can, (uint16_t) (500 * sin(((double) counter)/100) + 501));
    SEND_TEST_MSG2_2(q_tx_can, counter2);
    SEND_TEST_MSG3_2(q_tx_can, counter2);
    SEND_TEST_MSG4_2(q_tx_can, counter2);
    SEND_TEST_MSG5_2(q_tx_can, 0xFFF - counter2);
#endif

    counter += 1;
    counter2 += 2;
    if (counter2 >= 0xFFF)
    {
        counter2 = 1;
    }

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

void HardFault_Handler()
{
    PHAL_writeGPIO(LED_RED_GPIO_Port, LED_RED_Pin, 1);
    while(1)
    {
        __asm__("nop");
    }
}