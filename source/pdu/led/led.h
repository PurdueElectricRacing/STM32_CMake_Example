/**
 * @file led.h
 * @author Charles Tamer (ctamer@purdue.edu)
 * @brief
 * @version 0.1
 * @date 2024-1-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _LED_H
#define _LED_H

#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/psched/psched.h"
#include <stdbool.h>
#include <stdint.h>

// LED Driver Pins
#define LED_PUMP_1                  (0)
#define LED_PUMP_2                  (1)
#define LED_FAN_2                   (2)
#define LED_FAN_1                   (3)
#define LED_ABOX                    (4)
#define LED_DASH                    (5)
#define LED_MAIN                    (6)
#define LED_SDC                     (7)
#define LED_AUX_1                   (8)
#define LED_5V_CRIT                 (9)
#define LED_5V_NCRIT                (10)
#define LED_DAQ                     (11)
#define LED_5V_FAN                  (12)
#define LED_BLT                     (13)

enum LED_state {
    OFF = 0,
    ON,
    BLINK,
};

/**
 * @brief Set, clear, or toggle a specified LED pin
 * 
 * @param spi SPI hande
 * @param led LED driver pin
 * @param state Desired state of the LED : SET, CLEAR, or TOGGLE
 * @return true if data transfer is successful
 * @return false if data transfer fails
 */
bool LED_control(int led, enum LED_state state);

/**
 * @brief Periodic LED function that toggles LEDs that need to be toggled
 * 
 * @param
 * @return
 */
void LED_periodic();

#endif /* _LED_H */