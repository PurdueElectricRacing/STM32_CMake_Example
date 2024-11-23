#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/psched/psched.h"
#include "main.h"
#include <stddef.h>

GPIOInitConfig_t gpio_config[] = {
    // TODO: LED Pin def
    // Using UART5 on F7
    // DMA streams would be?
    // UART5 RX on DMA1 stream 0 PD2 alt function 8
    // UART5_TX on DMA1 stream 7 PC12 alt function 8
    GPIO_INIT_OUTPUT(ERR_LED_PORT, ERR_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_PORT, CONN_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_UART5TX_PC12,
    GPIO_INIT_UART5RX_PD2,
};

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

dma_init_t usart_ftdi_tx_dma_config = UART5_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_ftdi_rx_dma_config = UART5_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t huart_ftdi =
{
    .baud_rate = 115200,
    .word_length = WORD_8,
    .hw_flow_ctl = HW_DISABLE,
    .stop_bits = SB_ONE,
    .parity = PT_NONE,
    .obsample = OB_DISABLE,
    .ovsample = OV_16,
    .periph      = UART5,
    .wake_addr = false,
    .usart_active_num = USART5_ACTIVE_IDX,
    .tx_errors = 0,
    .rx_errors = 0,
    .tx_dma_cfg = &usart_ftdi_tx_dma_config,
    .rx_dma_cfg = &usart_ftdi_rx_dma_config
};

void HardFault_Handler();
void send_something();
void preflightAnimation(void);
void preflightChecks(void);

int main()
{
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    
    /* Task Creation */
    schedInit(APB1ClockRateHz);
    configureAnim(preflightAnimation, preflightChecks, 74, 1000);
        
    /* Schedule Periodic tasks here */
    taskCreate(send_something, 1000);
    schedStart();
    return 0;
}

void preflightChecks(void)
{
    static uint16_t state;

    switch (state++)
    {
    case 0:
        break;
    case 2:
        /* USART initialization */
        if (!PHAL_initUSART(&huart_ftdi, APB1ClockRateHz))
        {
            HardFault_Handler();
        }
        break;
    default:
        if (state > 750)
        {
            registerPreflightComplete(1);
            state = 750; // prevent wrap around
        }
        break;
    }
}

void preflightAnimation(void)
{
    static uint32_t time;

    PHAL_writeGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, 0);
    PHAL_writeGPIO(ERR_LED_PORT, ERR_LED_PIN, 0);
    PHAL_writeGPIO(CONN_LED_PORT, CONN_LED_PIN, 0);

    switch (time++ % 6)
    {
    case 0:
    case 5:
        PHAL_writeGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, 1);
        break;
    case 1:
    case 2:
    case 3:
        PHAL_writeGPIO(ERR_LED_PORT, ERR_LED_PIN, 1);
        break;
    case 4:
        PHAL_writeGPIO(CONN_LED_PORT, CONN_LED_PIN, 1);
        break;
    }
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}

void send_something() {
    char* txmsg = "Hello World!\n";
    PHAL_usartTxDma(&huart_ftdi, (uint16_t *)txmsg, 13);
}