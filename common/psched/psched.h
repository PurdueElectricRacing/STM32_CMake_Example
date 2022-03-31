#ifndef _PSCHED_H_
#define _PSCHED_H_

// Includes
#if defined(STM32L432xx)
    #include "stm32l4xx.h"
    #include "system_stm32l4xx.h"
#elif defined(STM32L496xx)
    #include "stm32l4xx.h"
    #include "system_stm32l4xx.h"
#elif defined(STM32F4)
    #include "system_stm32f4xx.h"
#else
    #error "Please define a MCU arch"
#endif

#define toMicros(time) ((uint32_t) time * 1000)

// Structs, Enums, Types
enum {
    TASK,
    TASK_BG
};

typedef void (*func_ptr_t)(void);

typedef struct {
    uint16_t task_time;                 // Time spent in a task
    uint16_t bg_time;                   // Time spent in bg loop
    uint32_t task_entry_time;           // Tick time of task entry
    uint32_t bg_entry_time;             // Tick time of background entry
    float    cpu_use;                   // % use of task time
} cpu_t;

typedef struct
{
    uint8_t    skips;                   // Number of loop misses. Execution time was skipped. Should always be 0
    uint8_t    run_next;                // Triggers background to run next iteration
    uint8_t    task_count;              // Number of tasks
    uint8_t    bg_count;                // Number of background tasks
    uint8_t    running;                 // Marks scheduler as running
    uint8_t    preflight_required;      // Preflight in use
    uint8_t    preflight_complete;      // Preflight complete registration
    uint8_t    anim_complete;           // Preflight setup/inspection complete
    uint16_t   anim_min_time;           // Minimum runtime of preflight animation
    uint32_t   os_ticks;                // Current OS tick count
    uint32_t   of;                      // MCU operating frequency

    uint16_t   task_time[15];           // Timings of registered tasks
    uint16_t   anim_time;               // Timing of animation
    func_ptr_t task_pointer[15];        // Function pointers to tasks
    func_ptr_t bg_pointer[15];          // Function pointers to background tasks

    func_ptr_t anim;                    // Animation function
    func_ptr_t preflight;               // Preflight function

    cpu_t    core;
} sched_t;

extern sched_t sched;

// Prototypes
void taskCreate(func_ptr_t func, uint16_t task_time);
void taskCreateBackground(func_ptr_t func);
void taskDelete(uint8_t type, uint8_t task);
void configureAnim(func_ptr_t anim, func_ptr_t preflight, uint16_t anim_time, uint16_t anim_min_time);
void registerPreflightComplete(uint8_t status);
void schedInit(uint32_t freq);
void schedStart();
void schedPause();

#endif
