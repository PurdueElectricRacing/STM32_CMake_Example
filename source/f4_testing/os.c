#include "os.h"
#include "common/phal_F4_F7/rcc/rcc.h"

/* vApplicationIdleHook */
void vApplicationIdleHook(void) {
    // Optional idle task code
}

/* vApplicationTickHook */
void vApplicationTickHook(void) {
    // Code to execute on each system tick
}

/* vApplicationMallocFailedHook */
void vApplicationMallocFailedHook(void) {
    // Handle memory allocation failure
    taskDISABLE_INTERRUPTS();
    for(;;);  // Infinite loop to trap or restart the system
}

/* vApplicationStackOverflowHook */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // Handle stack overflow
    taskDISABLE_INTERRUPTS();
    for(;;);  // Infinite loop to trap or restart the system
}

// Function to initialize TIM2 as a timebase
void TIM2_Init(void) {
    // Enable clock for TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Set Prescaler to get 1 MHz timer clock (assuming 16 MHz system clock)
    TIM2->PSC = 16 - 1;   // Prescaler value
    TIM2->ARR = 1000 - 1; // Auto-reload value for 1 ms period

    // Enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // Start the timer
    TIM2->CR1 |= TIM_CR1_CEN;  // Enable the timer

    // Set interrupt priority and enable interrupt
    NVIC_SetPriority(TIM2_IRQn, 5); // Set your desired priority
    NVIC_EnableIRQ(TIM2_IRQn);
}
void __attribute__((weak)) SysTick_Handler(void) {
    // Default implementation (can be empty)
}

// Timer 2 interrupt handler
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Check if update interrupt flag is set
        TIM2->SR &= ~TIM_SR_UIF; // Clear the interrupt flag

        // Increment FreeRTOS tick counter
        xPortSysTickHandler(); // Call FreeRTOS SysTick handler
    }
}

static StaticTask_t xIdleTaskTCB;
static StackType_t xIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = xIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* vApplicationGetTimerTaskMemory */
static StaticTask_t xTimerTaskTCB;
static StackType_t xTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize) {
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = xTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
