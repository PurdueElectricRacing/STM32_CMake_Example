#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/adc/adc.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/psched/psched.h"
#include "string.h"

#include "main.h"

volatile raw_adc_values_t raw_adc_values;

#pragma pack(push, 1)
typedef struct {
    int32_t ground_speed;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t north_velocity;
    int32_t east_velocity;
    int32_t down_velocity;
    int32_t heading;
    bool has_fix;
} SensorData;
#pragma pack(pop)


/* ADC Configuration */
ADCInitConfig_t adc_config = {
   .clock_prescaler = ADC_CLK_PRESC_6,
   .resolution      = ADC_RES_12_BIT,
   .data_align      = ADC_DATA_ALIGN_RIGHT,
   .cont_conv_mode  = true,
   .adc_number      = 1,
//    .overrun         = true,
   .dma_mode        = ADC_DMA_CIRCULAR
};


dma_init_t usart_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t lcd = {
   .baud_rate   = 250000,
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

#define WHO_AM_I 0x8F
#define I_AM_HIM 0x3F

// TODO: check prescaler for udpate rate
ADCChannelConfig_t adc_channel_config[] = {
   {.channel=0, .rank=1, .sampling_time=ADC_CHN_SMP_CYCLES_480},
};
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &raw_adc_values, sizeof(raw_adc_values) / sizeof(raw_adc_values.testval), 0b01);

GPIOInitConfig_t gpio_config[] = {
    // GPIO_INIT_ANALOG(GPIOA, 0),
    GPIO_INIT_OUTPUT(GPIOD, 13, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 12, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 14, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 15, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_USART2TX_PA2,
    GPIO_INIT_USART2RX_PA3,

    GPIO_INIT_OUTPUT(SPI_CS_PORT, SPI_CS_PIN, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_AF(SPI_SCK_PORT, SPI_SCK_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MOSI_PORT, SPI_MOSI_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MISO_PORT, SPI_MISO_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),

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


dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);
SPI_InitConfig_t spi_config = {
    .data_len  = 8,
    .nss_sw = true,
    .nss_gpio_port = SPI_CS_PORT,
    .nss_gpio_pin = SPI_CS_PIN,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI1
};

SPI_InitConfig_t spi_config_nonDMA = {
    .data_len  = 8,
    .nss_sw = true,
    .nss_gpio_port = SPI_CS_PORT,
    .nss_gpio_pin = SPI_CS_PIN,
    .rx_dma_cfg = 0,
    .tx_dma_cfg = 0,
    .periph = SPI1
};
char msg[100];


void HardFault_Handler();

void ledblink();
void testUsart();

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
    // if (!PHAL_SPI_init(&spi_config))
    //     HardFault_Handler();
    // if(!PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
    // {
    //     HardFault_Handler();
    // }
    if(!PHAL_initUSART(&lcd, APB1ClockRateHz))
    {
        HardFault_Handler();
    }
    // if(!PHAL_initDMA(&adc_dma_config))
    // {
    //     HardFault_Handler();
    // }
    //PHAL_writeGPIO(SPI_CS_PORT, SPI_CS_PIN, 1);
    //PHAL_startTxfer(&adc_dma_config);
    //PHAL_startADC(ADC1);
    PHAL_usartRxDma(&lcd, (uint16_t *) msg, 5, 1);
        /* Task Creation */
    schedInit(APB1ClockRateHz);
    //taskCreate(ledblink, 50);
    taskCreate(testUsart, 100);
    
    /* Schedule Periodic tasks here */
    schedStart();
    return 0;
}


void usart_recieve_complete_callback(usart_init_t *handle)
{
    if (handle == &lcd)
    {
        PHAL_toggleGPIO(GPIOD, 15);
    }
    else
    {
        PHAL_writeGPIO(GPIOD, 15, 0);
    }
}
// Global variable
SensorData sensor_data = {
    .ground_speed = 50,
    .longitude = 123456789,
    .latitude = 987654321,
    .height = 150,
    .north_velocity = 30,
    .east_velocity = 20,
    .down_velocity = 0,
    .heading = 90,
    .has_fix = true
};

// Counter to track iterations
int iteration_count = 0;

void testUsart()
{
    if (sensor_data.ground_speed > 150)
        sensor_data.ground_speed -= 20;
    else
        sensor_data.ground_speed += 5;

    sensor_data.longitude += sensor_data.east_velocity * 100;
    sensor_data.latitude += sensor_data.north_velocity * 100;

    if (sensor_data.height > 300)
        sensor_data.height -= 20;
    else
        sensor_data.height += 10;

    if (sensor_data.north_velocity > 50)
        sensor_data.north_velocity -= 5;
    else if (sensor_data.north_velocity < -50)
        sensor_data.north_velocity += 5;
    else
        sensor_data.north_velocity += 2;

    if (sensor_data.east_velocity > 50)
        sensor_data.east_velocity -= 5;
    else if (sensor_data.east_velocity < -50)
        sensor_data.east_velocity += 5;
    else
        sensor_data.east_velocity += 2;

    sensor_data.heading = (sensor_data.heading + 10) % 360;

    iteration_count++;

    if (iteration_count >= 100) {
        sensor_data.has_fix = false;
    } else if (iteration_count >= 200) {
        sensor_data.has_fix = true;
        iteration_count = 0;
    }

    PHAL_usartTxDma(&lcd, (uint16_t *)&sensor_data, sizeof(SensorData));
}


void ledblink()
{
    uint8_t out_data[3] = {WHO_AM_I, 0, 0};
    uint8_t in_data[3] = {0};
    // PHAL_SPI_transfer_noDMA(&spi_config, &out_data, 1, 1, in_data);
    while (PHAL_SPI_busy(&spi_config))
        ;
    PHAL_SPI_transfer(&spi_config, out_data, 2, in_data);
    while (PHAL_SPI_busy(&spi_config))
        ;

    uint8_t in_data_nonDMA[3] = {0};
    PHAL_SPI_transfer_noDMA(&spi_config_nonDMA, out_data, 1, 1, in_data_nonDMA);
    if (in_data[1] == I_AM_HIM)
        PHAL_writeGPIO(GPIOD, 13, 1);
    else
        PHAL_writeGPIO(GPIOD, 13, 0);

    if (in_data_nonDMA[1] == I_AM_HIM)
        PHAL_writeGPIO(GPIOD, 12, 1);
    else
        PHAL_writeGPIO(GPIOD, 12, 0);
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}