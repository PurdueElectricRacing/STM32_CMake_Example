/**
 * @file led_blink.c
 * @author Eileen Yoon (eyn@purdue.edu)
 * @brief  Demo for F4 testing structure
 *         - psched LED blink + uart serial printf
 * @version 0.1
 * @date 2025-01-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "f4_testing.h"

// Guard so cmake doesn't compile all tests
#if (F4_TESTING_CHOSEN == TEST_LED_BLINK)

#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/psched/psched.h"
#include "common/log/log.h"

void HardFault_Handler();
void ledblink();

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_OUTPUT(GPIOD, 12, GPIO_OUTPUT_LOW_SPEED), // F407VGT disco LEDs
    GPIO_INIT_OUTPUT(GPIOD, 13, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 14, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 15, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_USART2TX_PA2,
    GPIO_INIT_USART2RX_PA3,
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

volatile uint32_t tick_ms; // Systick 1ms counter

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .use_hse                    =false,
    .use_pll                    =false,
    .vco_output_rate_target_hz  =160000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

dma_init_t usart_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t usart_config = {
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART2,
   .wake_addr = false,
   .usart_active_num = USART2_ACTIVE_IDX,
   .tx_dma_cfg = &usart_tx_dma_config,
   .rx_dma_cfg = &usart_rx_dma_config
};
DEBUG_PRINTF_USART_DEFINE(&usart_config) // use LTE uart lmao

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
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    if(!PHAL_initUSART(&usart_config, APB1ClockRateHz))
    {
        HardFault_Handler();
    }
    log_yellow("PER PER PER\n");

    schedInit(APB1ClockRateHz);
    taskCreate(ledblink, 500);
    schedStart();

    return 0;
}

void ledblink()
{
    PHAL_toggleGPIO(GPIOD, 13);
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}

#endif // F4_TESTING_CHOSEN == TEST_LED_BLINK
