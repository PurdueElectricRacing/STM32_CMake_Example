#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/usart/usart.h"

#include "f4_testing_common.h"

volatile uint32_t tick_ms; // systick 1ms counter
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

int main()
{
#if defined(F4_TESTING_DAQ_DISCO_HELLO)
    daq_disco_hello_main();
#elif defined(F4_TESTING_DAQ_DISCO_UART)
    daq_disco_uart_main();
#elif defined(F4_TESTING_DASH_UART_SPI_ADC)
    dash_uart_spi_adc_main();
#elif defined(F4_TESTING_DAQ_DISCO_PDS_CAN)
    daq_disco_pds_can_main();
#else
    #error "No tests given"
#endif

    return 0;
}

// common routines
void SysTick_Handler(void)
{
    tick_ms++;
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}
