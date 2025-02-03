#include "common/bootloader/bootloader_common.h"

#if defined(STM32L496xx) || defined(STM32L432xx)
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/rcc/rcc.h"
#endif
#if defined(STM32F407xx) || defined(STM32F732xx)
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#endif

/* Module Includes */
#include "can_parse.h"
#include "node_defs.h"
#include "bootloader.h"


/* PER HAL Initilization Structures */
GPIOInitConfig_t gpio_config[] = {
    CAN_RX_GPIO_CONFIG,
    CAN_TX_GPIO_CONFIG,
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .use_hse                    =true,
    .use_pll                    =false,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

void HardFault_Handler();
void canTxSendToBack(CanMsgTypeDef_t *msg);
static void send_pending_can(void);
static void BL_CANPoll(void);

q_handle_t q_tx_can;
q_handle_t q_rx_can;

#define BOOTLOADER_INITIAL_TIMEOUT 3000   // wait 3s at start
#define CAN_TX_BLOCK_TIMEOUT (30 * 16000) // clock rate 16MHz, 15ms * 16000 cyc / ms
static volatile uint32_t bootloader_ms = 0; // systick

int main(void)
{
    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));
    bootloader_ms = 0;

#ifdef HSI_TRIM_BL_NODE
    PHAL_trimHSI(HSI_TRIM_BL_NODE);
#endif
    if (0 != PHAL_configureClockRates(&clock_config))
        HardFault_Handler();

    if (1 != PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
        HardFault_Handler();

    // Init bare minimum peripherals (systick, can1, crc)
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    if (1 != PHAL_initCAN(CAN1, false, VCAN_BPS))
        HardFault_Handler();

    initCANParse(&q_rx_can);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    // BL_sendStatusMessage(BLSTAT_BOOT, 1); // TODO send initial message

    // bootloader can loop
    while (bootloader_ms < BOOTLOADER_INITIAL_TIMEOUT || BL_flashStarted())
    {
        BL_CANPoll();
    }

    // dont init can or systick before this
    BL_checkAndBoot();

    while (1) // infinite bootloader can loop
    {
        BL_CANPoll();
    }
}

static void BL_CANPoll(void)
{
    send_pending_can();
    while (!qIsEmpty(&q_rx_can))
        canRxUpdate();
    send_pending_can();
}

// Sends all pending messages in the tx queue, doesn't require systick to be active
static void send_pending_can(void)
{
    CanMsgTypeDef_t tx_msg;
    while (qReceive(&q_tx_can, &tx_msg) == SUCCESS_G)
    {
        uint32_t t = 0;
        while (!PHAL_txMailboxFree(CAN1, 0) && (t++ < CAN_TX_BLOCK_TIMEOUT));
        if (t < CAN_TX_BLOCK_TIMEOUT) PHAL_txCANMessage(&tx_msg, 0);
        // TODO: count errors?
    }
}

void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    qSendToBack(&q_tx_can, msg);
}

void CAN1_RX0_IRQHandler()
{
    if (CAN1->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        CAN1->RF0R &= ~(CAN_RF0R_FOVR0);

    if (CAN1->RF0R & CAN_RF0R_FULL0) // FIFO Full
        CAN1->RF0R &= ~(CAN_RF0R_FULL0);

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

void SysTick_Handler(void)
{
    bootloader_ms++;
}

void HardFault_Handler()
{
    NVIC_SystemReset();
    while(1)
        ;
}
