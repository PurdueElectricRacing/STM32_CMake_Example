
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/usart/usart.h"

// DAQ uart test F4
#include "common/log/log.h"
#include <string.h>
#include "f4_testing_common.h"

#ifdef F4_TESTING_DAQ_DISCO_UART

#if 0
// USART6
dma_init_t usart_tx_dma_config = USART6_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART6_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t lte_usart_config = {
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART6,
   .wake_addr = false,
   .usart_active_num = USART6_ACTIVE_IDX,
   .tx_dma_cfg = &usart_tx_dma_config, // &usart_tx_dma_config
   .rx_dma_cfg = &usart_rx_dma_config, // usart_rx_dma_config
};
#else
dma_init_t usart_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t lte_usart_config = {
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
#endif

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_USART_TX(GPIOA, 2),
    GPIO_INIT_USART_RX(GPIOA, 3),
    GPIO_INIT_OUTPUT(GPIOD, 13, GPIO_OUTPUT_LOW_SPEED),
};

#if 1
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
#else
#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =16000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};
#endif

static inline int _iodev_write(usart_init_t* handle, char *buffer, int size)
{
    PHAL_usartTxBl(&lte_usart_config, (uint8_t *)buffer, size);
    //PHAL_usartTxDma(&lte_usart_config, (uint16_t *)buffer, size);
    return 0;
}

static inline int _iodev_putchar(usart_init_t* handle, uint8_t c)
{
    return _iodev_write(&lte_usart_config, (uint8_t *)&c, sizeof(uint8_t));
}

static inline int _iodev_printf(usart_init_t* handle, const char *fmt, ...)
{
    va_list args;
    char buffer[512];
    int i;

    va_start(args, fmt);
    i = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    _iodev_write(handle, buffer, i);

    return i;
}

typedef struct __attribute__((packed))
{
    uint32_t tick;      //!< ms timestamp of reception
    uint32_t type;
    uint32_t size;
    uint32_t data;
} daq_uart_frame_t;

#define STARTCODE 0xDEADBEEF

static void uart_frame_handler(void);

static void send_frame(uint32_t data)
{
    uint32_t startcode = STARTCODE;
    daq_uart_frame_t s;
    memset(&s, 0, sizeof(s));
    s.tick = tick_ms;
    s.type = 0x10000000 | 1;
    s.size = 8;
    s.data = data;
    PHAL_usartTxBl(&lte_usart_config, (uint8_t *)&startcode, sizeof(startcode));
    PHAL_usartTxBl(&lte_usart_config, (uint8_t *)&s, sizeof(s));
}

static bool receive_frame_start(void)
{
    uint8_t b = 0;

    PHAL_usartRxBl(&lte_usart_config, &b, 1);
    if (b == (STARTCODE & 0xff))
    {
        PHAL_usartRxBl(&lte_usart_config, &b, 1);
        if (b == ((STARTCODE & (0xff << 8)) >> 8))
        {
            PHAL_usartRxBl(&lte_usart_config, &b, 1);
            if (b == ((STARTCODE & (0xff << 16)) >> 16))
            {
                PHAL_usartRxBl(&lte_usart_config, &b, 1);
                if (b == ((STARTCODE & (0xff << 24)) >> 24))
                {
                    return true;
                }
            }
        }
    }

    return false;
}

static void uart_frame_handler(void)
{
    if (receive_frame_start())
    {
        PHAL_toggleGPIO(GPIOD, 13);
        daq_uart_frame_t frame;
        PHAL_usartRxBl(&lte_usart_config, (uint8_t *)&frame, sizeof(frame));
        switch (frame.type)
        {
            case 0x1: // heartbeat
                break;
            case 0x2: // helloworld
                _iodev_printf(&lte_usart_config, "Hello World\n");
                break;
        }
        send_frame(frame.data + 1); // ack
    }
}

int daq_disco_uart_main()
{
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    if(!PHAL_initUSART(&lte_usart_config, APB2ClockRateHz))
    {
        HardFault_Handler();
    }
    _iodev_printf(&lte_usart_config, "%s: UART initialized\n", "DAQ");

    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    while (1)
    {
        uart_frame_handler();
    }

    return 0;
}

#endif // F4_TESTING_DAQ_DISCO_UART
