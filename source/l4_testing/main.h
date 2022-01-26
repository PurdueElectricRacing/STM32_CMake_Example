#ifndef MAIN_H_
#define MAIN_H_

// Inputs
#define BUTTON_1_Pin (8)
#define BUTTON_1_GPIO_Port (GPIOA)
#define POT_Pin (0)
#define POT_GPIO_Port (GPIOA)
#define POT_ADC_Channel (5)

// Status LEDs
#define LED_GREEN_Pin (0)
#define LED_GREEN_GPIO_Port (GPIOB)
#define LED_RED_Pin (1)
#define LED_RED_GPIO_Port (GPIOB)
#define LED_BLUE_Pin (7)
#define LED_BLUE_GPIO_Port (GPIOB)

// TIM Pins
#define TIM1_GPIO_Port (GPIOA)
#define TIM1_Pin (8)
#define TIM1_AF (1)

#define TIM2_GPIO_Port (GPIOA)
#define TIM2_Pin (0)
#define TIM2_AF (1)

#define TIM16_GPIO_Port (GPIOA)
#define TIM16_Pin (6)
#define TIM16_AF (14)

#endif