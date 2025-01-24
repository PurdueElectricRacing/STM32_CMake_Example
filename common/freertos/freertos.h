#ifndef __COMMON_FREERTOS_H__
#define __COMMON_FREERTOS_H__

#include "cmsis_os2.h"
#include <stdint.h>

#if 0
// usage as drop-in replacement for psched:

#include "common/freertos/freertos.h" (also add FREERTOS to cmake.txt LIBS=)

void heartbeat_LED() { PHAL_toggleGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN); }
defineThreadStack(heartbeat_LED, 500, osPriorityNormal, 256); // define up here so its global
defineStaticQueue(tcp_tx_queue, timestamped_frame_t, TCP_TX_ITEM_COUNT);
defineStaticSemaphore(myHandle);

int main(void)
{
    osKernelInitialize(); // s/schedInit/osKernelInitialize

    // Do all hardware initialization + rtos object creations here
    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    NVIC_SetPriority(CAN1_RX0_IRQn, 6); // set priority >= 6, see configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY in FreeRTOSConfig.h

    createThread(heartbeat_LED); // s/taskCreate/createThread/
    tcp_tx_queue = createStaticQueue(tcp_tx_queue, timestamped_frame_t, TCP_TX_ITEM_COUNT);
    myHandle = createStaticSemaphore();

    osKernelStart(); // s/schedStart/osKernelStart
}

Very good resource: Mastering the FreeRTOS™ Real Time Kernel
https://www.freertos.org/Documentation/02-Kernel/07-Books-and-manual/01-RTOS_book

Key notes:
- under the freertos kernel, a task can either be in running or blocked state. you can only block a task (in the freertos sense) by calling an rtos blocking function, e.g. osDelay, semaphore wait, queue receive, etc.
- blocking in the freertos sense does not mean it blocks in the usual sense (i.e. polls and waits within that function), it will switch to another task.
- specifically freertos will always execute the highest priority task that can run (i.e. not in blocked state)

Other notes:
- if it crashes try increasing stack size
- lock all hardware acceses, e.g. spi transfers. best to raise a semaphore in the ISR when its done
- switch queues (e.g can queues) to freertos queues so freertos knows to block. TODO CAN wrapper
- if the total given loop time is less than the time it takes to actually execute all the tasks, lower priority tasks (or the tasks started last) will not be executed (task starved)
- use static allocations for all freertos objects (queues, sema) hence the macros here
- no static local variables
- direct task notifications are useful for state machines. e.g.

e.g. eth_send_udp_periodic() blocks until a direct notification from the state updater eth_update_connection_state()

static void eth_update_connection_state(void)
    switch (dh.eth_state)
        case ETH_IDLE:
            if (eth_init() == ETH_ERROR_NONE)
            {
                dh.eth_state = ETH_LINK_UP;
                xTaskNotify(getTaskHandle(eth_send_udp_periodic), 0, eNoAction);
            }

static void eth_send_udp_periodic(void)
    if (dh.eth_state != ETH_LINK_UP)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

- note eth_send_udp_periodic child task has to be started before eth_update_connection_state so it can enter blocking state

#endif

typedef struct {
    void (*taskFunction)();
    uint32_t delay;
    osThreadAttr_t attrs;
    osThreadId_t handle;
} ThreadWrapper;

void rtosWrapper(void *);

// Cursed macro
#define threadWrapperName(NAME) (threadWrapper_##NAME)
// TASK: task function name
// DELAY: delay in ticks (should be ms)
// PRIORITY: one of osPriorityNormal, osPriorityHigh, etc
// STACK: stack size
#define __defineThread(TASK, DELAY, PRIORITY, STACK)\
    ThreadWrapper threadWrapperName(TASK) = { \
        .taskFunction = &(TASK),     \
        .delay = (DELAY),            \
        .attrs = {                   \
            .priority = (PRIORITY),  \
            .stack_size = (STACK),   \
            .name = "\""#TASK"\"",   \
        }                            \
    };                               \

#define defineThread(T, D, P) __defineThread(T, D, P, 1024) // TODO calculate stack size
#define defineThreadStack(T, D, P, S) __defineThread(T, D, P, S)

#define __createThread(NAME)\
    threadWrapperName(NAME).handle = osThreadNew(rtosWrapper, &(threadWrapperName(NAME)), &(threadWrapperName(NAME)).attrs);
#define createThread(NAME) __createThread(NAME)

#define getTaskHandle(NAME) threadWrapperName(NAME).handle

// queues
#define defineStaticQueue(NAME, ITEM, COUNT)                      \
    osMessageQueueId_t NAME;                                      \
    static uint8_t ucQueueStorageArea_##NAME[sizeof(ITEM) * (COUNT)]; \
    static osMessageQueueAttr_t attr_##NAME = {                   \
        .name = #NAME,                                            \
        .mq_mem = ucQueueStorageArea_##NAME,                      \
        .mq_size = sizeof(ucQueueStorageArea_##NAME),             \
        .cb_mem = NULL,                                           \
        .cb_size = 0                                              \
    };

#define createStaticQueue(NAME, ITEM, COUNT)                      \
    NAME = osMessageQueueNew((COUNT), sizeof(ITEM), &attr_##NAME);

// semaphores
#define defineStaticSemaphore(NAME)                               \
    osSemaphoreId_t NAME;                                         \
    static osSemaphoreAttr_t attr_##NAME = {                      \
        .name = #NAME,                                            \
        .cb_mem = NULL,                                           \
        .cb_size = 0                                              \
    };

#define createStaticSemaphore(NAME)                               \
    NAME = osSemaphoreNew(1, 1, &attr_##NAME);

#define getTick() osKernelGetTickCount()
#define getTickms() osKernelGetTickCount() // CMSIS handles ticks directly

#endif // __COMMON_FREERTOS_H__
