/**
 * @file pedals.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Read, check, and send pedal measurements
 * @version 0.1
 * @date 2022-03-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef _PEDALS_H_
#define _PEDALS_H_

/* System Includes */
#include "common/psched/psched.h"
#include "common/common_defs/common_defs.h"
#include "common/faults/faults.h"

/* Module Includes */
#include "can_parse.h"
#include <stdint.h>

#define MAX_PEDAL_MEAS (4095)

#define APPS_IMPLAUS_MAX_DIFF (3000) //(410)  // T.4.2.4  (10% of 0x0FFF)
#define APPS_IMPLAUS_TIME_MS  (100)  // T.4.2.5
#define APPS_IMPLAUS_MIN      (50)   // Lowering for now, if below min, throttle = 0 anyway //(205)  // T.4.2.10 ( 5% of 0x0FFF)
#define APPS_IMPLAUS_MAX      (3891) // T.4.2.10 (95% of 0x0FFF)
#define BSE_IMPLAUS_MIN (50) // lowering for now //(205)  // ( 5% of 0x0FFF)
#define BSE_IMPLAUS_MAX (3891) // (95% of 0x0FFF)

#define APPS_BRAKE_THRESHOLD               (307)  // EV.5.7.1 (7.5% of 0x0FFF)
#define APPS_THROTTLE_FAULT_THRESHOLD      (1024) // EV.5.7.1 (25% of 0x0FFF)
#define APPS_THROTTLE_CLEARFAULT_THRESHOLD (205)  // EV.5.7.2 ( 5% of 0x0FFF)

#define BRAKE_PRESSURE_THRESHOLD (425)
#define FLT_TO_DISPLAY_INT_2_DEC (100U)
#define FLT_TO_PERCENTAGE (100U)

#define VREF 3.3F
#define RESISTOR_T1 3300
#define RESISTOR_T2 1000
#define MAX_RESISTANCE 2500
#define MAX_VALUE 4095

#define PROFILES_START_SECTOR    3
#define NUM_PROFILES             4
#define PROFILE_WRITE_SUCCESS 0
#define PROFILE_WRITE_FAIL -1

typedef struct
{
    bool     apps_faulted;              // wiring or 10% dev
    bool     apps_implaus_detected;
    uint32_t apps_implaus_start_time;
    bool     bse_faulted;               // wiring
    bool     bse_wiring_fail_detected;
    uint32_t bse_wiring_fail_start_time;
    bool     apps_brake_faulted;        // throttle and brake pressed together
} pedals_t;

extern pedals_t pedals;

typedef struct {
    uint16_t t1max;
    uint16_t t1min;
    uint16_t t2max;
    uint16_t t2min;
    uint16_t b1max;
    uint16_t b1min;
    uint16_t b2max;
    uint16_t b2min;
} pedal_calibration_t;

typedef struct {
    uint8_t id;
    uint8_t brake_travel_threshold;
    uint8_t throttle_travel_threshold;
    uint8_t reserved;
} driver_profile_t;

extern pedal_calibration_t pedal_calibration;
extern uint16_t filtered_pedals;
extern uint16_t thtl_limit;
extern driver_profile_t driver_profiles[4];

/* Function Prototypes */
void pedalsPeriodic(void);
int writeProfiles(void);
void readProfiles(void);

#endif