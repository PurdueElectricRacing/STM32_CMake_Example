#include "common/phal_L4/gpio/gpio.h"



bool PHAL_initGPIO(GPIOInitConfig_t config[], uint8_t config_len)
{

    uint8_t afr_i;
    
    for (int i = 0; i < config_len; i++)
    {
        // Enable clock
        // Not sure if this should live in the GPIO init or a seperate RCC HAL lib
        switch ((uint32_t) config[i].bank)
        {
            case (uint32_t) GPIOA:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
                break;

            case (uint32_t) GPIOB:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
                break;

            case (uint32_t) GPIOC:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
                break;

            case (uint32_t) GPIOH:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
                break;
            
#ifdef STM32L496xx
            case (uint32_t) GPIOE:
                RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
                break;
#endif
            default:
                return false;
                break;
        }

        // Setup GPIO output type
        config[i].bank->MODER &= ~(GPIO_MODER_MODE0_Msk << (GPIO_MODER_MODE1_Pos * config[i].pin));
        config[i].bank->MODER |= (config[i].type & GPIO_MODER_MODE0_Msk) << (GPIO_MODER_MODE1_Pos * config[i].pin);
        
        switch(config[i].type)
        {
            case GPIO_TYPE_OUTPUT:
                config[i].bank->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk << (GPIO_OSPEEDR_OSPEED1_Pos * config[i].pin));
                config[i].bank->OSPEEDR |= (config[i].config.ospeed & GPIO_OSPEEDR_OSPEED0_Msk) << (GPIO_OSPEEDR_OSPEED1_Pos * config[i].pin);
  
                config[i].bank->OTYPER &= ~(GPIO_OTYPER_OT0_Msk << (GPIO_OTYPER_OT1_Pos * config[i].pin));
                config[i].bank->OTYPER |= (config[i].config.otype & GPIO_OTYPER_OT0_Msk) << (GPIO_OTYPER_OT1_Pos * config[i].pin);
                break;
                
            case GPIO_TYPE_INPUT:
                config[i].bank->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk << (GPIO_PUPDR_PUPD1_Pos * config[i].pin));
                config[i].bank->PUPDR |= (config[i].config.pull & GPIO_PUPDR_PUPD0_Msk) << (GPIO_PUPDR_PUPD1_Pos * config[i].pin);
                break;

            case GPIO_TYPE_AF:
                afr_i = config[i].pin > 7 ? 1 : 0;
                config[i].bank->AFR[afr_i] &= ~(GPIO_AFRL_AFSEL0_Msk << (GPIO_AFRL_AFSEL1_Pos * (config[i].pin % 8)));
                config[i].bank->AFR[afr_i] |= (config[i].config.af_num & GPIO_AFRL_AFSEL0_Msk) << (GPIO_AFRL_AFSEL1_Pos * (config[i].pin % 8));

                config[i].bank->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk << (GPIO_OSPEEDR_OSPEED1_Pos * config[i].pin));
                config[i].bank->OSPEEDR |= (config[i].config.ospeed & GPIO_OSPEEDR_OSPEED0_Msk) << (GPIO_OSPEEDR_OSPEED1_Pos * config[i].pin);

                //config[i].bank->OTYPER &= ~(GPIO_OTYPER_OT0_Msk << (GPIO_OTYPER_OT1_Pos * config[i].pin));
                //config[i].bank->OTYPER |= (config[i].config.otype & GPIO_OTYPER_OT0_Msk) << (GPIO_OTYPER_OT1_Pos * config[i].pin);

                //config[i].bank->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk << (GPIO_PUPDR_PUPD1_Pos * config[i].pin));
                //config[i].bank->PUPDR |= (config[i].config.pull & GPIO_PUPDR_PUPD0_Msk) << (GPIO_PUPDR_PUPD1_Pos * config[i].pin);
                break;
            
            case GPIO_TYPE_ANALOG:
                break;
            
            default:
                return false;
        }
    }

    return true;
}

bool PHAL_readGPIO(GPIO_TypeDef* bank, uint8_t pin)
{
    return (bank->IDR >> pin) & 0b1;
}

void PHAL_writeGPIO(GPIO_TypeDef* bank, uint8_t pin, bool value)
{
    bank->BSRR |= 1 << (pin + (16 * (!value))); // BSRR has "set" as bottom 16 bits and "reset" as top 16 
}

void PHAL_toggleGPIO(GPIO_TypeDef* bank, uint8_t pin)
{
    PHAL_writeGPIO(bank, pin, !PHAL_readGPIO(bank, pin));
}