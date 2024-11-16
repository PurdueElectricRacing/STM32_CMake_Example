#ifndef _F4_TESTING_COMMON_H_
#define _F4_TESTING_COMMON_H_

#define F4_TESTING_DAQ_DISCO_UART

//#define F4_TESTING_DAQ_DISCO_HELLO
//#define F4_TESTING_DAQ_DISCO_UART
//#define F4_TESTING_DASH_UART_SPI_ADC

extern volatile uint32_t tick_ms; // Systick 1ms counter
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

void HardFault_Handler(void);
void SysTick_Handler(void);

#ifdef F4_TESTING_DAQ_DISCO_UART
int daq_disco_uart_main();
#endif

#ifdef F4_TESTING_DAQ_DISCO_HELLO
int daq_disco_hello_main();
#endif

#ifdef F4_TESTING_DASH_UART_SPI_ADC
int dash_uart_spi_adc_main(void);
#endif

#endif // _F4_TESTING_COMMON_H_
