/**
 * @file car.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Master Car Control and Safety Checking
 * @version 0.1
 * @date 2022-03-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _CAR_H_
#define _CAR_H_

#include "can_parse.h"
#include "common/faults/faults.h"
#include "common/phal_F4_F7/gpio/gpio.h"
// #include "common/plettenberg/plettenberg.h"
#include "common/psched/psched.h"
#include "cooling.h"
#include "main.h"
#include "common/amk/amk.h"
#include <stdbool.h>

#define BUZZER_DURATION_MS 2500 // EV.10.5: 1-3s

#define ERROR_FALL_MS (5000)

/* BRAKE LIGHT CONFIG */
#define BRAKE_LIGHT_ON_THRESHOLD  (170)
#define BRAKE_LIGHT_OFF_THRESHOLD (70)

#define BRAKE_PRESSED_THRESHOLD (BRAKE_LIGHT_ON_THRESHOLD)


// Shock Pot Calibration
#define POT_TOTAL_RES 3000
#define POT_MAX_RES 3300
#define POT_MIN_RES 300

#define POT_VOLT_MAX_L 4.0f
#define POT_VOLT_MIN_L 4077.0f
#define POT_VOLT_MAX_R 4.0f
#define POT_VOLT_MIN_R 4090.0f
#define POT_MAX_DIST 75
#define POT_DIST_DROOP_L 57
#define POT_DIST_DROOP_R 54


//Defines to guess a BSPD fault
#define NUM_HIST_BSPD 16
typedef struct
{
    float torque_left;    // [-100.0, 100.0]
    float torque_right;
} torqueRequest_t;

typedef enum
{
    CAR_TORQUE_NONE,
    CAR_TORQUE_RAW,
    CAR_TORQUE_TV,
    CAR_TORQUE_THROT_MAP,
    CAR_TORQUE_DAQ
} torqueSource_t;

#define HV_LOW_PASS_SIZE (5)
#define HV_V_MC_CAL      (1000)        // V_actual / V_measured * 1000
#define HV_V_BAT_CAL     (1000)        // V_actual / V_measured * 1000
typedef struct
{
    bool pchg_complete;
    bool pchg_error;
    uint16_t  v_mc_filt;                     // volts x10
    uint16_t  v_bat_filt;                    // volts x10
    uint16_t  v_mc_buff[HV_LOW_PASS_SIZE];   // Units are 12-bit adc
    uint8_t   v_mc_buff_idx;
    uint16_t  v_bat_buff[HV_LOW_PASS_SIZE];  // Units are 12-bit adc
    uint8_t   v_bat_buff_idx;
} prechargeStat_t;

typedef struct
{
    car_state_t state;
    amk_motor_t motor_l;
    amk_motor_t motor_r;

    torqueRequest_t torque_r;
    torqueSource_t torque_src;
    bool regen_enabled;

    prechargeStat_t pchg;
    bool start_btn_debounced;
    bool sdc_close;
    bool brake_light;
    bool buzzer;
    uint32_t buzzer_start_ms;
} Car_t;
extern Car_t car;

extern uint8_t daq_buzzer;
extern uint8_t daq_brake;
extern uint8_t buzzer_brake_fault;
extern uint8_t daq_constant_tq;
extern uint8_t const_tq_val;

bool carInit();
void carHeartbeat();
void carPeriodic();
void parseMCDataPeriodic(void);
void calibrateSteeringAngle(uint8_t* success);
void monitorSDCPeriodic(void);
void updateSDCFaults(void);
void send_shockpots();

// SDC Node defines
#define SDC_MUX_HIGH_IDX 14
#define SDC_MAIN 0
#define SDC_C_STOP 1
#define SDC_INERTIA 2
#define SDC_BOTS 3
#define SDC_R_STOP 8
#define SDC_L_STOP 9
#define SDC_HVD 10
#define SDC_HUB 11
#define SDC_TSMS 12

typedef struct __attribute__((packed))
{
    uint8_t main_stat; //y0
    uint8_t c_stop_stat; //y1
    uint8_t inertia_stat; //y2
    uint8_t bots_stat; //y3
    uint8_t nc; //y4
    uint8_t bspd_stat; //y5
    uint8_t bms_stat; //y6
    uint8_t imd_stat; //y7
    uint8_t r_stop_stat; //y8
    uint8_t l_stop_stat; //y9
    uint8_t hvd_stat; //y10
    uint8_t r_hub_stat; //y11
    uint8_t tsms_stat; //y12
    uint8_t pchg_out_stat; //y13

} sdc_nodes_t;

extern sdc_nodes_t sdc_mux;


#endif
