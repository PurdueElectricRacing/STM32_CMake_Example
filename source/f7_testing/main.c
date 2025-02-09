#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/psched/psched.h"

GPIOInitConfig_t gpio_config[] = {
// TODO: LED Pin def
};

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

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

void HardFault_Handler();

int main()
{
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    // if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    // {
    //     HardFault_Handler();
    // }
        /* Task Creation */
    schedInit(APB1ClockRateHz);
        /* Schedule Periodic tasks here */
    schedStart();
    return 0;
}

void HardFault_Handler()
{
    while(1)
    {
        __asm__("nop");
    }
}
