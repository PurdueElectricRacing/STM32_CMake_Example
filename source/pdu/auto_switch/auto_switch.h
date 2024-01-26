/**
 * @file auto_switch.h
 * @author Gavin Zyonse (gzyonse@purdue.edu)
 * @brief 
 * @version 1.0
 * @date 2023-11-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _AUTO_SWITCH_H_
#define _AUTO_SWITCH_H_

#include <stdint.h>
#include <source/pdu/main.h>

// Static variables
#define SHUNT_R 10000000
#define ADC_MAX 4095

// Voltage sense resistors
#define LV_24V_R1  47000 // Ohms
#define LV_24V_R2  3400  // Ohms
#define LV_5V_R1   4300  // Ohms
#define LV_5V_R2   3400  // Ohms
#define LV_3V3_R1  4300  // Ohms
#define LV_3V3_R2  10000 // Ohms

// HP Current sense resistors
#define HP_CS_R1 180 // Ohms
#define HP_CS_R2 330 // Ohms
#define HP_CS_R3 500 // Ohms

// Upstream Current sense
#define HP_CS_R_SENSE 0.002 // Ohms
#define CS_GAIN 100


// Enumeration
typedef enum {
    // High power switches
    SW_PUMP_1,
    SW_PUMP_2,
    SW_SDC,
    SW_AUX,
    // Low power switches
    SW_FAN_1,
    SW_FAN_2,
    SW_DASH,
    SW_ABOX,
    SW_MAIN,
    SW_BLT,
    // 5V switches
    SW_CRIT_5V,
    SW_NCRIT_5V,
    SW_DAQ,
    SW_FAN_5V,

    // Not actually switches
    CS_24V,
    CS_5V,

    // Number of switches (must be last)
    NUM_SWITCHES
} switches_t;

// Structures
typedef struct {
    uint16_t in_24v;
    uint16_t out_5v;
    uint16_t out_3v3;
} voltage_t;  // Voltage in mV

typedef struct {
    uint8_t fault_status[NUM_SWITCHES];
    uint16_t current[NUM_SWITCHES];  // Current in mA
    voltage_t voltage;
} auto_switch_t;

extern auto_switch_t auto_switch;

// Function definitions
uint8_t faultStatus();
void getFaults();
void getCurrent();
void getVoltage();
void enableSwitch();
uint16_t calcCurrent_HP(uint16_t);
uint16_t calcCurrent_LP(uint16_t);
void calcCurrent_Total();
uint16_t calcVoltage(uint16_t, int, int);

#endif