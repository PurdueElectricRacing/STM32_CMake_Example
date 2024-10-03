
#include "FreeRTOS.h"
#include "task.h"

/* vApplicationIdleHook */
void vApplicationIdleHook(void);

/* vApplicationTickHook */
void vApplicationTickHook(void);

/* vApplicationMallocFailedHook */
void vApplicationMallocFailedHook(void);

/* vApplicationStackOverflowHook */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);

// Function to initialize TIM2 as a timebase
void TIM2_Init(void);
void __attribute__((weak)) SysTick_Handler(void);

// Timer 2 interrupt handler
void TIM2_IRQHandler(void);