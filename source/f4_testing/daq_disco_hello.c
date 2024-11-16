#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/psched/psched.h"

// basic hello world test template for f4 disco
#include "common/log/log.h"
#include "f4_testing_common.h" // include header
// then define ifdef guard for file
#ifdef F4_TESTING_DAQ_DISCO_HELLO
void test_uart(void); // do all stuff under here

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
   .tx_dma_cfg = &usart_tx_dma_config, // &usart_tx_dma_config
   .rx_dma_cfg = &usart_rx_dma_config, // usart_rx_dma_config
};
DEBUG_PRINTF_USART_DEFINE(&usart_config)

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_USART_TX(GPIOA, 2),
    GPIO_INIT_USART_RX(GPIOA, 3),
    GPIO_INIT_OUTPUT(GPIOD, 13, GPIO_OUTPUT_LOW_SPEED),
};

#define TargetCoreClockrateHz 96000000
ClockRateConfig_t clock_config = {
    .system_source              = SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    = PLL_SRC_HSI16,
    .vco_output_rate_target_hz  = 192000000, //288000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 1),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 1),
};

int daq_disco_hello_main(void) // define main routine
{
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    if(!PHAL_initUSART(&usart_config, APB2ClockRateHz))
    {
        HardFault_Handler();
    }
    debug_printf("%s: UART initialized\n", "DAQ");

    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    schedInit(APB1ClockRateHz);
    taskCreate(test_uart, 250);
    schedStart();

    return 0;
}

void test_uart(void)
{
    PHAL_toggleGPIO(GPIOD, 13);
    debug_printf("%s: 0x%08x: hello world\n", "DAQ", tick_ms);
}

#endif // F4_TESTING_DAQ_DISCO_HELLO
