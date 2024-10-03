#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "os.h"

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_OUTPUT(GPIOD, 13, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(GPIOD, 12, GPIO_OUTPUT_LOW_SPEED),
};

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =16000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

void HardFault_Handler();

TaskHandle_t xOrangeLEDTaskHandle;
void vOrangeLEDTask(void *pvParameters) {
    for(;;) {
        PHAL_toggleGPIO(GPIOD, 13);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

TaskHandle_t xGreenLEDTaskHandle;
void vGreenLEDTask(void *pvParameters) {
    for(;;) {
        PHAL_toggleGPIO(GPIOD, 12);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main()
{
    if(0 != PHAL_configureClockRates(&clock_config)) {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t))) {
        HardFault_Handler();
    }
    TIM2_Init();
    xTaskCreate(vOrangeLEDTask, "Orange LED Task", 128, NULL, 1, &xOrangeLEDTaskHandle);
    xTaskCreate(vGreenLEDTask, "Green LED Task", 128, NULL, 1, &xGreenLEDTaskHandle);
    vTaskStartScheduler();
    while(1) {
        asm("nop");
    }

    return 0;
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}