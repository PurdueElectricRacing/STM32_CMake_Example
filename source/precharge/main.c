/* System Includes */
#include "stm32l496xx.h"
#include "system_stm32l4xx.h"
#include "can_parse.h"
#include "common/psched/psched.h"
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/quadspi/quadspi.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/rcc/rcc.h"
#include "common/phal_L4/spi/spi.h"

/* Module Includes */
#include "main.h"
#include "bmi088.h"


/* PER HAL Initilization Structures */
GPIOInitConfig_t gpio_config[] = {
    // CAN
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12,
    GPIO_INIT_CAN2RX_PB12,
    GPIO_INIT_CAN2TX_PB13,

    // SPI
    GPIO_INIT_AF(SPI_SCLK_GPIO_Port, SPI_SCLK_Pin,  5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin,  5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MISO_GPIO_Port, SPI_MISO_Pin,  5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_OUTPUT(SPI_CS_ACEL_GPIO_Port, SPI_CS_ACEL_Pin, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, GPIO_OUTPUT_HIGH_SPEED),

    // Status and HV Monitoring
    GPIO_INIT_OUTPUT(BMS_STATUS_GPIO_Port, BMS_STATUS_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(IMD_HS_PWM_GPIO_Port, IMD_HS_PWM_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_INPUT(IMD_LS_PWM_GPIO_Port, IMD_LS_PWM_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_INPUT(IMD_STATUS_GPIO_Port, IMD_STATUS_Pin, GPIO_INPUT_OPEN_DRAIN),

    // LEDs
    GPIO_INIT_OUTPUT(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
};

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

q_handle_t q_tx_can;
q_handle_t q_rx_can;

void heartbeat_task();
void PHAL_FaultHandler();
extern void HardFault_Handler();
void canTxUpdate();

dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

SPI_InitConfig_t spi_config = {
    .data_rate = TargetCoreClockrateHz / 64,
    .data_len  = 8,
    .nss_sw = true,
    .nss_gpio_port = SPI_CS_GYRO_GPIO_Port,
    .nss_gpio_pin = SPI_CS_GYRO_Pin,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config
};

BMI088_Handle_t bmi_config = {
    .acel_csb_gpio_port = SPI_CS_ACEL_GPIO_Port,
    .acel_csb_pin = SPI_CS_ACEL_Pin,
    .gyro_csb_gpio_port = SPI_CS_GYRO_GPIO_Port,
    .gyro_csb_pin = SPI_CS_GYRO_Pin,
    .gyro_datarate = GYRO_DR_1000Hz_116Hz,
    .gyro_range = GYRO_RANGE_250,
    .spi = &spi_config
};

int16_t x, y, z;
int main (void)
{
    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));  

    /* HAL Initilization */
    if (0 != PHAL_configureClockRates(&clock_config))
        PHAL_FaultHandler();

    if (1 != PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
        PHAL_FaultHandler();

    if (1 != PHAL_initCAN(CAN1, false))
        PHAL_FaultHandler();

    if (1 != PHAL_initCAN(CAN2, false))
        PHAL_FaultHandler();

    spi_config.data_rate = APB2ClockRateHz / 16;
    if (!PHAL_SPI_init(&spi_config))
        PHAL_FaultHandler();

    

    PHAL_writeGPIO(SPI_CS_ACEL_GPIO_Port, SPI_CS_ACEL_Pin, 1);
    PHAL_writeGPIO(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, 1);
    
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN2_RX0_IRQn);

    /* Module init */
    schedInit(APB1ClockRateHz * 2); // See Datasheet DS11451 Figure. 4 for clock tree
    initCANParse(&q_rx_can);

    if (!BMI088_init(&bmi_config))
        PHAL_FaultHandler();
    while(1)
    {
        BMI088_readGyro(&bmi_config, &x, &y, &z);
    }
    

    /* Task Creation */
    schedInit(SystemCoreClock);
    taskCreate(canRxUpdate, RX_UPDATE_PERIOD);
    taskCreate(canTxUpdate, 5);
    taskCreate(heartbeat_task, 1000);

    /* No Way Home */
    schedStart();

    return 0;
}

void PHAL_FaultHandler()
{
    asm("bkpt");
    HardFault_Handler();
}

void heartbeat_task()
{
    PHAL_toggleGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin);
    SEND_BALANCE_REQUEST(q_tx_can, 100);
}

// *** Compulsory CAN Tx/Rx callbacks ***
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
        canProcessRxIRQs(&rx);
        qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
    }
}

void CAN2_RX0_IRQHandler()
{
    if (CAN2->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        CAN2->RF0R &= !(CAN_RF0R_FOVR0); 

    if (CAN2->RF0R & CAN_RF0R_FULL0) // FIFO Full
        CAN2->RF0R &= !(CAN_RF0R_FULL0); 

    if (CAN2->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        CanMsgTypeDef_t rx;

        // Get either StdId or ExtId
        if (CAN_RI0R_IDE & CAN2->sFIFOMailBox[0].RIR)
        { 
          rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN2->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
        }
        else
        {
          rx.StdId = (CAN_RI0R_STID & CAN2->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;
        }

        rx.DLC = (CAN_RDT0R_DLC & CAN2->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

        rx.Data[0] = (uint8_t) (CAN2->sFIFOMailBox[0].RDLR >> 0) & 0xFF;
        rx.Data[1] = (uint8_t) (CAN2->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
        rx.Data[2] = (uint8_t) (CAN2->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t) (CAN2->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t) (CAN2->sFIFOMailBox[0].RDHR >> 0) & 0xFF;
        rx.Data[5] = (uint8_t) (CAN2->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
        rx.Data[6] = (uint8_t) (CAN2->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t) (CAN2->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        CAN2->RF0R     |= (CAN_RF0R_RFOM0);
        canProcessRxIRQs(&rx);
        qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
    }
}