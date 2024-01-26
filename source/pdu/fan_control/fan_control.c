#include "fan_control.h"
#include "common/phal_F4_F7/gpio/gpio.h"

extern uint32_t APB2ClockRateHz;

/* My notes
* timer starts at inital value and counts up to value in ARR. It goes high
* and then stays high until the timer counts down to the value in CCRx.
*
* pwm frequency is determined by:
*   - prescalar
*   - internal clock
*   - ARR
* Duty cycle is determined by CCRx
*
* https://deepbluembedded.com/stm32-pwm-example-timer-pwm-mode-tutorial/ Image 
* on this page explains this all very well (formulas here as well)
*/

bool fanControlInit()
{
    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN; // Enable clock to TIM1
    FAN_PWM_TIM -> CR1 &= ~TIM_CR1_CEN; // Disable counter (turn off timer)

    /* Solving for PSC:
     * PWMfreq = Fclk / ((ARR + 1) × (PSC + 1)).
     *
     * PWMfreq is known, and we are setting ARR to 99 for ease of calculations.
     *
     * Therefore we wolve for the below formula for the desired PSC based on the
     * necessary PWMfreq specified by the fan vendor.
    */
    FAN_PWM_TIM -> PSC = (APB2ClockRateHz / (PWM_FREQUENCY_HZ * 100)) - 1;

    // NOTE: Set up regs so that CCR1 can be directly set from 0-100
    FAN_PWM_TIM -> ARR = 100 - 1; // Using this for ease of calculations

    FAN_PWM_TIM -> CCR1 = 0; // Start with it off
    FAN_PWM_TIM -> CCR2 = 0; // Start with it off

    // Set PWM mode 1 (active while CNT <= CCR1)
    FAN_PWM_TIM -> CCMR1 &= ~TIM_CCMR1_OC1M;
    FAN_PWM_TIM -> CCMR1 &= ~TIM_CCMR1_OC2M;
    FAN_PWM_TIM -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM Mode 1 ch1
    FAN_PWM_TIM -> CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // PWM Mode 1 ch2
    FAN_PWM_TIM -> CCMR1 |= TIM_CCMR1_OC2PE |TIM_CCMR1_OC1PE; // Enable preload register ch1 & ch2

    FAN_PWM_TIM -> CCER |= TIM_CCER_CC2E | TIM_CCER_CC1E; // Enable output compare 1 & 2 
    // do I even need 2?
    FAN_PWM_TIM -> BDTR |= TIM_BDTR_MOE; // Enable main output

    FAN_PWM_TIM -> CR1 &= ~TIM_CR1_DIR; // Set to upcounting mode
    FAN_PWM_TIM -> CR1 |= TIM_CR1_ARPE; // Necessary in upcounting mode

    FAN_PWM_TIM -> CR1 |= TIM_CR1_CEN; // Enable counter (turn on timer)

    // Enable FAN_PWM_TIM
    FAN_PWM_TIM->CR1 |= TIM_CR1_CEN;    

    /* FAN_1_TACH_TIM */
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    FAN_1_TACH_TIM -> CR1 &= ~TIM_CR1_CEN; // Disable counter (turn off timer)

    /* Set input capture mode */
    FAN_1_TACH_TIM -> CCER &= ~TIM_CCER_CC2E; // Turn of the channel (necessary to write CC2S bits)
    FAN_1_TACH_TIM -> CCMR1 &= ~TIM_CCMR1_CC2S;
    FAN_1_TACH_TIM -> CCMR1 |= TIM_CCMR1_CC2S_0; // Map IC2 to TI2

    FAN_1_TACH_TIM -> CCER &= ~TIM_CCER_CC2P; // Capture input on the rising edge
    FAN_1_TACH_TIM -> CCER |= TIM_CCER_CC2E; // Enable the channel

    FAN_1_TACH_TIM -> CR1 |= TIM_CR1_CEN; // Enable timer

    return true;
}

// This speed will be between 0-100%
void setFanSpeed(uint8_t fan_speed)
{
    // Duty cycle is (CCR1 / ARR)%. So CCR1 = (ARR / duty cycle)
    FAN_PWM_TIM -> CCR1 = (FAN_PWM_TIM -> ARR) * (fan_speed / 100.0);
}
