#ifndef __PLETTENBERG_H__
#define __PLETTENBERG_H__

#include "common/phal_L4/usart/usart.h"
#include "common/common_defs/common_defs.h"
#include "common/psched/psched.h"
#include "common/queue/queue.h"
#include "string.h"
#include "stm32l432xx.h"

#define MC_MAX_TX_LENGTH (25)
#define MC_MAX_RX_LENGTH (77 + MC_MAX_TX_LENGTH)

// Waits this long before updating to connected after an rx
#define MC_BOOT_TIME               (4000)
// Since first rx waits this long until the timeout is switched
// from the large timeout to small timeout (should be receiving
// echos of torque commands every 15ms)
#define MC_TIMEOUT_CONSTRAINT_TIME (1000 + MC_BOOT_TIME)

#define MC_RX_LARGE_TIMEOUT_MS 1500
#define MC_RX_SMALL_TIMEOUT_MS 30

#define MC_SERIAL_MODE    's'
#define MC_ANALOG_MODE    'p'
#define MC_FORWARD        'f'
#define MC_REVERSE        'r'
#define MC_BRAKE          'b'
#define MC_MAX_POWER      'm'
#define MC_INCREASE_ONE   '+'
#define MC_DECREASE_ONE   '-'
#define MC_INCREASE_TENTH 'g'
#define MC_DECREASE_TENTH 'l'

#define MC_ENTER_ADJUST_MODE 'a'
#define MC_EXIT_ADJUST_MODE  'e'
#define MC_WRITE_PARAMS      "wp"

#define MC_PAR_CURRENT_LIMIT "cl"
#define MC_PAR_MOT_TMP_LIMIT "mt"
#define MC_PAR_CTL_TMP_LIMIT "ct"
#define MC_PAR_UPDATE_PERIOD "ot"

#define MC_CURRENT_LIMIT (100)
#define MC_MOT_TMP_LIMIT (100)
#define MC_CTL_TMP_LIMIT (80)

#define CELL_MAX_V 4.2 //May be increased to 4.25 in the future
#define CELL_MIN_V 2.5

typedef enum
{
    MC_DISCONNECTED, // RX line is silent
    MC_INITIALIZING, // Signs of life, waiting for boot to complete
    MC_CONNECTED,    // Receiving messages as expected
    MC_ERROR         // :(
} motor_state_t;

typedef struct 
{
    motor_state_t motor_state;
    bool is_inverted;                       // Send 'f' versus 'r' for positive torque command
    uint16_t curr_power_x10;                // Last torque command percent output sent x10
    /* Tx */
    q_handle_t *tx_queue;                   // FIFO for tx commands to be sent via DMA
    /* Rx */
    uint32_t rx_timeout;                    // Dynamically set timeout to determine connection status
    uint32_t boot_start_time;               // Time of the first rx message received
    volatile uint32_t last_rx_time;         // Time of the last rx message received
    volatile char rx_buf[MC_MAX_RX_LENGTH]; // DMA rx circular buffer
    /* Parsed Values */
    bool  data_valid;                       // True if the following values are up to date and legit
    uint16_t voltage_x10;                      
    uint16_t current_x10;
    uint32_t rpm;
    uint8_t controller_temp;
    uint8_t motor_temp;
} motor_t;

/**
 * @brief Initializes the motor in serial mode
 */
void mc_init(motor_t *m, bool is_inverted, q_handle_t *tx_queue);
/**
 * @brief Positive power commands motor to move
 *        Negative power commands motor to regen
 */
void mc_set_power(float power, motor_t *m);
/**
 * @brief  Reads the data being sent from the motor controller
 *         and determines the connection status
 * @return false if the data was not succesfully parsed
 */
bool mc_periodic(motor_t *m);

#endif