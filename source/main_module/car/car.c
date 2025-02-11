#include "car.h"
#include "main.h"
#include "wheel_speeds.h"

Car_t car;
extern q_handle_t q_tx_usart_l, q_tx_usart_r;
extern usart_rx_buf_t huart_l_rx_buf, huart_r_rx_buf;
extern uint16_t num_failed_msgs_l, num_failed_msgs_r;
extern WheelSpeeds_t wheel_speeds;
// TODO: Just to remove errors for now
// usart_rx_buf_t huart_l_rx_buf, huart_r_rx_buf;
uint8_t daq_buzzer;
uint8_t daq_brake;
uint8_t daq_constant_tq;
uint8_t const_tq_val;
uint8_t buzzer_brake_fault;
sdc_nodes_t sdc_mux;

// Historical record of Brake stat and Current Sense to tell if BSPD has failed
int16_t hist_current[NUM_HIST_BSPD] = {0};
uint8_t hist_curr_idx;

uint8_t prchg_start;

/* Wheel Speed Config */
// WheelSpeed_t left_wheel =  {.tim=TIM2, .invert=true};
// WheelSpeed_t right_wheel = {.tim=TIM5, .invert=true};
// // TODO: test invert
// WheelSpeeds_t wheel_speeds = {.l=&left_wheel, .r=&right_wheel};

bool validatePrecharge();

bool carInit()
{
    /* Set initial states */
    car = (Car_t) {0}; // Everything to zero
    car.state = CAR_STATE_IDLE;
    car.torque_src = CAR_TORQUE_TV;
    car.regen_enabled = false;
    car.sdc_close = true; // We want to initialize SDC as "good"
    PHAL_writeGPIO(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, car.sdc_close);
    PHAL_writeGPIO(BRK_LIGHT_GPIO_Port, BRK_LIGHT_Pin, car.brake_light);
    PHAL_writeGPIO(BUZZER_GPIO_Port, BUZZER_Pin, car.buzzer);

    daq_buzzer = 0;
    daq_constant_tq = 0;
    const_tq_val = 0;
    hist_curr_idx = 0;


    PHAL_writeGPIO(SDC_MUX_S0_GPIO_Port, SDC_MUX_S0_Pin, 0);
    PHAL_writeGPIO(SDC_MUX_S1_GPIO_Port, SDC_MUX_S1_Pin, 0);
    PHAL_writeGPIO(SDC_MUX_S2_GPIO_Port, SDC_MUX_S2_Pin, 0);
    PHAL_writeGPIO(SDC_MUX_S3_GPIO_Port, SDC_MUX_S3_Pin, 0);
    wheelSpeedsInit();
}

void carHeartbeat()
{
    // SEND_MAIN_HB(car.state, car.pchg.pchg_complete);
    SEND_MAIN_HB_AMK(car.state, car.pchg.pchg_complete);
    SEND_REAR_MC_STATUS(car.motor_l.motor_state,
        car.motor_l.link_state, car.motor_l.last_link_error,
        car.motor_r.motor_state, car.motor_r.link_state,
        car.motor_r.last_link_error);
    static uint8_t n;
}

/**
 * @brief Main task for the car containing a finite state machine
 *        to determine when the car should be driveable
 */
uint32_t last_time = 0;
uint8_t curr = 0;
void carPeriodic()
{
    /* Set Default Outputs */
    // Set Outputs that are purely combinational (do not depend on previous events)
    // Default values will typically reflect those in the IDLE state
    car.torque_r.torque_left  = 0.0f;
    car.torque_r.torque_right = 0.0f;
    car.buzzer    = false;
    car.sdc_close = true;
    // if (!can_data.orion_currents_volts.stale)
    // {
    //     hist_current[hist_curr_idx++] = can_data.orion_currents_volts.pack_current;
    //     hist_curr_idx %= NUM_HIST_BSPD;
    // }
    // else
    // {
    //     hist_current[hist_curr_idx++] = 0;
    //     hist_curr_idx %= NUM_HIST_BSPD;
    // }
    // if (!checkFault(ID_TV_DISABLED_FAULT))
    // {
    //     car.torque_src = CAR_TORQUE_TV;
    // }
    // else
    // {
    //     car.torque_src = CAR_TORQUE_RAW;
    // }
    // TSMS/HVD Disconnecting is not an error, so go back to init state. However, we must keep fatal state latched
    if (checkFault(ID_TSMS_DISC_FAULT) || checkFault(ID_HVD_DISC_FAULT) && car.state != CAR_STATE_FATAL)
        car.state = CAR_STATE_IDLE;

    // /* Process Inputs */

    // // Start button debounce (only want rising edge)
    car.start_btn_debounced = can_data.start_button.start;
    can_data.start_button.start = false;



    /**
     * Brake Light Control
     * The on threshold is larger than the off threshold to
     * behave similar to a Shmitt-trigger, preventing blinking
     * during a transition
     */
    if (can_data.filt_throttle_brake.brake > BRAKE_LIGHT_ON_THRESHOLD)
    {
        if (!car.brake_light)
        {
            car.brake_light = true;
        }
    }
    else if (can_data.filt_throttle_brake.brake < BRAKE_LIGHT_OFF_THRESHOLD)
    {
        if (car.brake_light)
        {
            car.brake_light = false;
        }
    }

    if (checkFault(ID_RTD_EXIT_FAULT))
    {
        car.state = CAR_STATE_IDLE;
    }
    // An error fault has higher priority
    // than the RTD Exit Fault
    if (errorLatched())
    {
        car.state = CAR_STATE_ERROR;
    }
    // A fatal fault has higher priority
    // than an error fault
    if (fatalLatched())
    {
        car.state = CAR_STATE_FATAL;
    }

    /* State Dependent Operations */

    // EV.10.4 - Activation sequence
    // Tractive System Active - SDC closed, HV outside accumulator
    // Ready to drive - motors respond to APPS input
    //                  not possible unless:
    //                   - tractive system active
    //                   - brake pedal pressed and held
    //                   - start button press

    if (car.state == CAR_STATE_FATAL)
    {
        // SDC critical error has occured, open sdc
        // Currently latches into this state
        car.sdc_close = false;
        car.pchg.pchg_complete = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);
    }
    else if (car.state == CAR_STATE_ERROR)
    {
        // Error has occured, leave HV on but do not drive
        // Recover once error gone
        if (!errorLatched()) car.state = CAR_STATE_IDLE;
        car.pchg.pchg_complete = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);
        prchg_start = false;
    }
    else if (car.state == CAR_STATE_IDLE)
    {
        car.pchg.pchg_complete = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);
        prchg_start = false;
        if (sdc_mux.tsms_stat)
        {
            car.state = CAR_STATE_PRECHARGING;
        }
    }
    else if (car.state == CAR_STATE_PRECHARGING)
    {
        // float v_mc = ((adc_readings.v_mc / 4095.0f) * 3.3f);
        // float v_batt = ((adc_readings.v_bat / 4095.0f) * 3.3f);
        // float threshold = 0.92f * v_batt;

        // static uint32_t precharge_start_ms;
        // if (!prchg_start)
        // {
        //     precharge_start_ms = sched.os_ticks;
        //     prchg_start = 1;
        // }
        // setFault(ID_PRECHARGE_TIME_FAULT_FAULT, (sched.os_ticks - precharge_start_ms));
        if (/*v_mc >= threshold && */PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin))
        {
            car.pchg.pchg_complete = 1;
            car.state = CAR_STATE_ENERGIZED;
        }
        // else
        // {
        //     setFault(ID_PCHG_IMPLAUS_FAULT, 1);
        // }
    }
    else if (car.state == CAR_STATE_ENERGIZED)
    {
        prchg_start = false;
        car.pchg.pchg_complete = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);

        if (car.start_btn_debounced &&
           can_data.filt_throttle_brake.brake > BRAKE_PRESSED_THRESHOLD)
        {
            car.state = CAR_STATE_BUZZING;
            car.buzzer_start_ms = sched.os_ticks;
        }
    }
    else if (car.state == CAR_STATE_BUZZING)
    {
        car.pchg.pchg_complete = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);
        // EV.10.5 - Ready to drive sound
        // 1-3 seconds, unique from other sounds
        if (sched.os_ticks - car.buzzer_start_ms > BUZZER_DURATION_MS)
        {
            if (daq_constant_tq)
                car.state = CAR_STATE_CONSTANT_TORQUE;
            else
                car.state = CAR_STATE_READY2DRIVE;
        }
    }
    else if (car.state == CAR_STATE_READY2DRIVE)
    {
        car.pchg.pchg_complete = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);
        if (!car.pchg.pchg_complete)
        {
            car.state = CAR_STATE_IDLE;
        }

        // Check if requesting to exit ready2drive
        if (car.start_btn_debounced)
        {
            car.state = CAR_STATE_IDLE;
        }
        else
        {
            float t_req_pedal = 0;
            float t_req_pedal_l = 0;
            float t_req_pedal_r = 0;
            float t_req_equal_l = 0;
            float t_req_equal_r = 0;
            if (!can_data.filt_throttle_brake.stale)
                t_req_pedal = (float) CLAMP(can_data.filt_throttle_brake.throttle, 0, 4095);
            if (!can_data.throttle_vcu.stale)
                t_req_pedal_l = (float) CLAMP(can_data.throttle_vcu.vcu_k_rl, 0, 4095);
            if (!can_data.throttle_vcu.stale)
                t_req_pedal_r = (float) CLAMP(can_data.throttle_vcu.vcu_k_rr, 0, 4095);
            if (!can_data.throttle_vcu_equal.stale)
                t_req_equal_l = (float) CLAMP(can_data.throttle_vcu_equal.equal_k_rl, 0, 4095);
            if (!can_data.throttle_vcu_equal.stale)
                t_req_equal_r = (float) CLAMP(can_data.throttle_vcu_equal.equal_k_rr, 0, 4095);

            t_req_pedal = t_req_pedal * 100.0f / 4095.0f;
            t_req_pedal_l = t_req_pedal_l * 100.0f / 4095.0f;
            t_req_pedal_r = t_req_pedal_r * 100.0f / 4095.0f;
            t_req_equal_l = t_req_equal_l * 100.0f / 4095.0f;
            t_req_equal_r = t_req_equal_r * 100.0f / 4095.0f;
            // if (t_req_pedal > 10.0f)
            //     t_req_pedal = 10.0f;


            // TODO: ensure APPS checks sets throttle to 0 if enough braking
            // t_req = t_req < 100 ? 0 : ((t_req - 100) / (4095 - 100) * 4095);
            // uint16_t adjusted_throttle = (can_data.raw_throttle_brake.throttle < 100) ? 0 : (can_data.raw_throttle_brake.throttle - 100) * 4095 / (4095 - 100);

            torqueRequest_t temp_t_req;
            switch (car.torque_src)
            {
                case CAR_TORQUE_RAW:
                    temp_t_req.torque_left  = t_req_pedal;
                    temp_t_req.torque_right = t_req_pedal;
                    break;
                case CAR_TORQUE_TV:
                    if ((wheel_speeds.left_rad_s_x100 == 0 || wheel_speeds.right_rad_s_x100 == 0) && can_data.orion_currents_volts.pack_current > 10)
                    {
                        setFault(ID_REMAP_UNRELIABLE_FAULT, 1);
                    }
                    else
                    {
                        setFault(ID_REMAP_UNRELIABLE_FAULT, 0);
                    }
                    if (checkFault(ID_TV_ENABLED_FAULT))
                    {
                        temp_t_req.torque_left  = t_req_pedal_l;
                        temp_t_req.torque_right = t_req_pedal_r;
                        // EV.4.2.3 - Torque algorithm
                        // Any algorithm or electronic control unit that can adjust the
                        // requested wheel torque may only lower the total driver
                        // requested torque and must not increase it
                        if (temp_t_req.torque_left > t_req_equal_l)
                        {
                            temp_t_req.torque_left = t_req_equal_l;
                        }
                        if (temp_t_req.torque_right > t_req_equal_r)
                        {
                            temp_t_req.torque_right = t_req_equal_r;
                        }
                    }
                    else if (!checkFault(ID_REMAP_UNRELIABLE_FAULT) || (checkFault(ID_MM_ENABLED_FAULT)))
                    {
                        temp_t_req.torque_left  = t_req_equal_l;
                        temp_t_req.torque_right = t_req_equal_r;
                    }
                    else
                    {
                        temp_t_req.torque_left = t_req_pedal;
                        temp_t_req.torque_right = t_req_pedal;
                    }
                    if (t_req_pedal == 0)
                    {
                        temp_t_req.torque_left = 0;
                        temp_t_req.torque_right = 0;
                    }
                    break;
                case CAR_TORQUE_THROT_MAP:
                    temp_t_req.torque_left  = t_req_equal_l;
                    temp_t_req.torque_right = t_req_equal_r;
                    // EV.4.2.3 - Torque algorithm
                    // Any algorithm or electronic control unit that can adjust the
                    // requested wheel torque may only lower the total driver
                    // requested torque and must not increase it
                    if (t_req_pedal == 0)
                    {
                        temp_t_req.torque_left = 0;
                        temp_t_req.torque_right = 0;
                    }
                    // if (temp_t_req.torque_left > t_req_pedal_l)
                    // {
                    //     temp_t_req.torque_left = t_req_pedal_l;
                    // }
                    // if (temp_t_req.torque_right > t_req_pedal_r)
                    // {
                    //     temp_t_req.torque_right = t_req_pedal_r;
                    // }
                    break;
                case CAR_TORQUE_DAQ:
                    break;
                case CAR_TORQUE_NONE:
                    break;
                default:
                    temp_t_req.torque_left  = 0;
                    temp_t_req.torque_right = 0;
                break;
            }

            // Enforce range
            temp_t_req.torque_left =  CLAMP(temp_t_req.torque_left,  -100.0f, 100.0f);
            temp_t_req.torque_right = CLAMP(temp_t_req.torque_right, -100.0f, 100.0f);

            // Disable Regenerative Braking
            if (!car.regen_enabled)
            {
                if (temp_t_req.torque_left  < 0) temp_t_req.torque_left  = 0.0f;
                if (temp_t_req.torque_right < 0) temp_t_req.torque_right = 0.0f;
            }

            car.torque_r = temp_t_req;
        }
    }
    else if (car.state == CAR_STATE_CONSTANT_TORQUE)
    {
        static bool throttle_pressed;
        static bool brake_pressed;
        PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);
        // Check if requesting to exit ready2drive
        // if (car.start_btn_debounced)
        // {
        //     car.state = CAR_STATE_IDLE;
        // }
        // else
        // {
            float t_req_pedal = 0;

            if (!can_data.filt_throttle_brake.stale)
                t_req_pedal = (float) CLAMP(can_data.filt_throttle_brake.throttle, 0, 4095);

            torqueRequest_t temp_t_req;
            // Register an INTENTIONAL press of throttle and hold this state untill commanded otherwise
            if (throttle_pressed || (!throttle_pressed && t_req_pedal > 1024))
            {
                brake_pressed = 0;
                temp_t_req.torque_left = (const_tq_val * 1.0f);
                temp_t_req.torque_right = (const_tq_val * 1.0f);
                throttle_pressed = 1;
            }
            if ((can_data.filt_throttle_brake.brake > 50) || brake_pressed)
            {
                throttle_pressed = 0;
                brake_pressed = 1;
                temp_t_req.torque_left = 0.0f;
                temp_t_req.torque_right = 0.0f;
            }

            // Enforce range
            temp_t_req.torque_left =  CLAMP(temp_t_req.torque_left,  -100.0f, 100.0f);
            temp_t_req.torque_right = CLAMP(temp_t_req.torque_right, -100.0f, 100.0f);

            // Disable Regenerative Braking
            if (!car.regen_enabled)
            {
                if (temp_t_req.torque_left  < 0) temp_t_req.torque_left  = 0.0f;
                if (temp_t_req.torque_right < 0) temp_t_req.torque_right = 0.0f;
            }

            car.torque_r = temp_t_req;
        // }
    }

    /* Update System Outputs */
    car.buzzer = car.state == CAR_STATE_BUZZING || daq_buzzer;
    buzzer_brake_fault = PHAL_readGPIO(BRK_BUZZER_STAT_GPIO_Port, BRK_BUZZER_STAT_Pin);
    PHAL_writeGPIO(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, car.sdc_close);
    PHAL_writeGPIO(BRK_LIGHT_GPIO_Port, BRK_LIGHT_Pin, car.brake_light | daq_brake);
    PHAL_writeGPIO(BUZZER_GPIO_Port, BUZZER_Pin, car.buzzer);
    mcSetPower(car.torque_r.torque_left,  &car.motor_l);
    mcSetPower(car.torque_r.torque_right, &car.motor_r);
 }


/**
 * @brief Parses motor controller and sensor
 *        info into can messages, updates
 *        motor controller connection status
 */
void parseMCDataPeriodic(void)
{
    uint16_t shock_l, shock_r;

    /* Update Motor Controller Data Structures */
    mcPeriodic(&car.motor_l);
    mcPeriodic(&car.motor_r);

    // setFault(ID_LEFT_MC_CONN_FAULT, car.pchg.pchg_complete &&
    //             car.motor_l.motor_state != MC_CONNECTED);
    // setFault(ID_RIGHT_MC_CONN_FAULT, car.pchg.pchg_complete &&
    //             car.motor_r.motor_state != MC_CONNECTED);
    // Only send once both controllers have updated data
    // if (motor_right.data_stale ||
    //     motor_left.data_stale) return;

    // TODO: fill with faults

    // Extract raw shocks from DMA buffer
    // shock_l = raw_shock_pots.pot_left;
    // shock_r = raw_shock_pots.pot_right;
    // Scale from raw 12bit adc to mm * 10 of linear pot travel
    // shock_l = (POT_VOLT_MIN_DIST_MM * 10 - ((uint32_t) shock_l) * (POT_VOLT_MIN_DIST_MM - POT_VOLT_MAX_DIST_MM) * 10 / 4095);
    // shock_r = (POT_VOLT_MIN_DIST_MM * 10 - ((uint32_t) shock_r) * (POT_VOLT_MIN_DIST_MM - POT_VOLT_MAX_DIST_MM) * 10 / 4095);

    // SEND_REAR_WHEEL_DATA(wheel_speeds.left_kph_x100, wheel_speeds.right_kph_x100,
    //                      shock_l, shock_r);
    // uint16_t l_speed = (wheel_speeds.l->rad_s / (2*PI));
    // uint16_t r_speed = (wheel_speeds.l->rad_s / (2*PI));
    wheelSpeedsPeriodic();
    SEND_REAR_WHEEL_SPEEDS(car.motor_l.rpm, car.motor_r.rpm,
                                    wheel_speeds.left_rad_s_x100,
                                    wheel_speeds.right_rad_s_x100);
    static uint32_t last_curr_t;
    if (sched.os_ticks - last_curr_t >= 100)
    {
        SEND_REAR_MOTOR_CURRENTS_VOLTS(
                                    (uint16_t) car.motor_l.current_x10,
                                    (uint16_t) car.motor_r.current_x10,
                                    (uint16_t) car.motor_r.voltage_x10);
        last_curr_t = sched.os_ticks;
    }
    // TODO: possibly move into cooling
    static uint32_t last_tmp_t;
    if (sched.os_ticks - last_tmp_t >= 500)
    {
        // SEND_REAR_MOTOR_TEMPS(
        //                         (uint8_t) car.motor_l.motor_temp,
        //                         (uint8_t) car.motor_r.motor_temp,
        //                         (uint8_t) car.motor_l.controller_temp,
        //                         (uint8_t) car.motor_r.controller_temp);
        SEND_NUM_MC_SKIPS(num_failed_msgs_r, num_failed_msgs_l);
        last_tmp_t = sched.os_ticks;
    }
}

void send_shockpots()
{
    uint16_t shock_l = adc_readings.shock_l;
    uint16_t shock_r = adc_readings.shock_r;
    int16_t shock_l_parsed;
    int16_t shock_r_parsed;
    // Will scale linearly from 0 - 3744. so 75 - (percent of 3744 * 75)
    shock_l_parsed = -1 * ((POT_MAX_DIST - (int16_t)((shock_l / (POT_VOLT_MIN_L - POT_VOLT_MAX_L)) * POT_MAX_DIST)) - POT_DIST_DROOP_L);
    shock_r_parsed = -1 * ((POT_MAX_DIST - (int16_t)((shock_r / (POT_VOLT_MIN_R - POT_VOLT_MAX_R)) * POT_MAX_DIST)) - POT_DIST_DROOP_R);

    SEND_SHOCK_REAR(shock_l_parsed, shock_r_parsed);
}

/**
 * @brief  Checks faults that should prevent
 *         the car from driving, but are okay
 *         to leave the sdc closed
 *
 * @return true  Faults exist
 * @return false No faults have existed for set time
 */
uint32_t last_error_time = 0;
bool error_rose = 0;
bool checkErrorFaults()
{
    uint8_t is_error = 0;
    uint8_t prchg_stat;
    static uint16_t prchg_time;

    /* Heart Beat Stale */
    // is_error += can_data.dashboard_hb.stale;
    // is_error += can_data.front_driveline_hb.stale;
    // TODO: is_error += can_data.rear_driveline_hb.stale;
    // TODO: is_error += can_data.precharge_hb.stale;

    prchg_stat = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);

    // if (!prchg_stat) {
    //     ++prchg_time;
    // } else {
    //     prchg_set = 1;
    //     prchg_time = 0;
    // }

    // if (prchg_time > (500 / 15)) {
    //     --prchg_time;
    //     ++is_error;
    //     prchg_set = 0;
    // }

    /* Precharge */
    // is_error += !PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);

    /* Dashboard */
    // TODO: is_error += can_data.raw_throttle_brake.stale;

    /* Driveline */
    // Front
    // is_error += can_data.front_driveline_hb.front_left_motor  !=
    //             FRONT_LEFT_MOTOR_CONNECTED;
    // is_error += can_data.front_driveline_hb.front_right_motor !=
    //             FRONT_RIGHT_MOTOR_CONNECTED;

    // Rear
    // TODO: revert
    /*
    is_error += can_data.rear_driveline_hb.rear_left_motor    !=
                REAR_LEFT_MOTOR_CONNECTED;
    is_error += can_data.rear_driveline_hb.rear_right_motor   !=
                REAR_RIGHT_MOTOR_CONNECTED;
                */

    /* Temperature */
    // TODO: if (!DT_ALWAYS_COOL)  is_error += cooling.dt_temp_error;
    // TODO: if (!BAT_ALWAYS_COOL) is_error += cooling.bat_temp_error;

    if (is_error && !error_rose)
    {
        error_rose = true;
        last_error_time = sched.os_ticks;
    }

    if (!is_error && error_rose &&
        sched.os_ticks - last_error_time > ERROR_FALL_MS)
    {
        error_rose = false;
    }

    return is_error;
}

/**
 * @brief  Checks faults that should open the SDC
 * @return true  Faults exist
 * @return false No faults have existed for set time
 */
bool checkFatalFaults()
{
    uint8_t is_error = 0;

    // TODO: is_error += !PHAL_readGPIO(LIPO_BAT_STAT_GPIO_Port, LIPO_BAT_STAT_Pin);

    is_error += (can_data.max_cell_temp.max_temp > 500) ? 1 : 0;

    return is_error;
}

/**
 * @brief Resets the steering angle sensor calibration
 *        Call once after assembly with wheel centered
 *        Device: Bosch F02U.V02.894-01
 */
void calibrateSteeringAngle(uint8_t *success)
{
    // To zero the sensor after assembly:
    // Reset calibration with CCW = 5h
    // Start a new calibration with CCW = 3h
    // The sensor can then be used immediately
    SEND_LWS_CONFIG(0x05, 0, 0); // reset cal
    SEND_LWS_CONFIG(0x03, 0, 0); // start new
    *success = 1;
}

/**
 * @brief Checks if the precharge circuit is working
 *        correctly based on the V_MV and V_Bat ADC
 *        measurements and PRCHG Complete Signal
 *
 * @return true Precharge Working Properly
 */
bool validatePrecharge()
{
    uint32_t tmp_mc, tmp_bat;
    tmp_mc = tmp_bat = 0;

    // Measure inputs
    car.pchg.v_mc_buff[car.pchg.v_mc_buff_idx++]   = adc_readings.v_mc;
    car.pchg.v_bat_buff[car.pchg.v_bat_buff_idx++] = adc_readings.v_bat;
    car.pchg.pchg_complete = PHAL_readGPIO(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin);

    // Update buffers
    car.pchg.v_mc_buff_idx  %= HV_LOW_PASS_SIZE;
    car.pchg.v_bat_buff_idx %= HV_LOW_PASS_SIZE;

    // Sum buffers
    for (uint8_t i = 0; i < HV_LOW_PASS_SIZE; ++i)
    {
        tmp_mc  += car.pchg.v_mc_buff[i];
        tmp_bat += car.pchg.v_bat_buff[i];
    }
    // Take average
    tmp_mc  /= HV_LOW_PASS_SIZE;
    tmp_bat /= HV_LOW_PASS_SIZE;

    // V_outx10 = adc_raw * adc_v_ref / adc_max (Tiffomy outputs volts / 100)
    car.pchg.v_mc_filt  = (tmp_mc  * ((ADC_REF_mV * HV_V_MC_CAL)  / 1000UL)) / 0xFFFUL;
    car.pchg.v_bat_filt = (tmp_bat * ((ADC_REF_mV * HV_V_BAT_CAL) / 1000UL)) / 0xFFFUL;

    // Validate
    if ((car.pchg.pchg_complete && car.pchg.v_mc_filt < ((((uint32_t) car.pchg.v_bat_filt) * 80) / 100)) ||
        (!car.pchg.pchg_complete && car.pchg.v_mc_filt >= ((((uint32_t) car.pchg.v_bat_filt * 95) / 100))))
    {
        car.pchg.pchg_error = true;
    }
    else
    {
        car.pchg.pchg_error = false;
    }

    return car.pchg.pchg_error;
}

void updateSDCFaults()
{
    if(!can_data.orion_currents_volts.stale)
    {
        hist_current[hist_curr_idx] = can_data.orion_currents_volts.pack_current;
        hist_curr_idx++;
    }
    uint8_t brake_fail = 0;
    // Loop through all SDC nodes (no need for precharge complete)
    for (uint8_t i = 0; i < (SDC_MUX_HIGH_IDX - 1); i++)
    {
        switch(i)
        {
            case (SDC_C_STOP):
                if (!sdc_mux.c_stop_stat && sdc_mux.inertia_stat)
                {
                    // setFault(ID_COCKPIT_ESTOP_FAULT, 1);
                }
                else
                {
                    setFault(ID_COCKPIT_ESTOP_FAULT, 0);
                }
                break;
            case (SDC_INERTIA):
                if (!sdc_mux.inertia_stat && sdc_mux.bots_stat)
                {
                    // setFault(ID_INERTIA_FAIL_FAULT, 1);
                }
                else
                {
                    setFault(ID_INERTIA_FAIL_FAULT, 0);
                }
                break;
            case (SDC_BOTS):
                //If bots is down, we need to check whether BOTS was tripped or BSPD was tripped
                // if (!sdc_mux.bots_stat && !checkFault(ID_IMD_FAULT) && (PHAL_readGPIO(BMS_STAT_GPIO_Port, BMS_STAT_Pin) || can_data.orion_currents_volts.stale))
                // {
                //     int32_t total_current = 0;
                //     for (int16_t i = 0; i < NUM_HIST_BSPD; i++)
                //     {
                //         total_current += hist_current[i];
                //     }
                //     if (total_current > 5000)
                //     {
                //         setFault(ID_BSPD_LATCHED_FAULT, 1);
                //     }
                //     else if (brake_fail)
                //         setFault(ID_BSPD_LATCHED_FAULT, 1);
                //     else
                //         setFault(ID_BOTS_FAIL_FAULT, 1);
                // }
                // else if (checkFault(ID_BOTS_FAIL_FAULT) && sdc_mux.bots_stat)
                // {
                //     setFault(ID_BOTS_FAIL_FAULT, 0);
                // }
                break;
            case (SDC_L_STOP):
                if (!sdc_mux.l_stop_stat && sdc_mux.r_stop_stat)
                {
                    // setFault(ID_LEFT_ESTOP_FAULT, 1);
                }
                else
                {
                    setFault(ID_LEFT_ESTOP_FAULT, 0);
                }
                break;
            case (SDC_R_STOP):
                if (!sdc_mux.r_stop_stat && sdc_mux.main_stat)
                {
                    // setFault(ID_RIGHT_ESTOP_FAULT, 1);
                }
                else
                {
                    setFault(ID_RIGHT_ESTOP_FAULT, 0);
                }
                break;
            case (SDC_HVD):
                if (!sdc_mux.hvd_stat && sdc_mux.l_stop_stat)
                {
                    // setFault(ID_HVD_DISC_FAULT, 1);
                }
                else
                {
                    setFault(ID_HVD_DISC_FAULT, 0);
                }
                break;
            case (SDC_HUB):
                if (!sdc_mux.r_hub_stat && sdc_mux.hvd_stat)
                {
                    // setFault(ID_HUB_DISC_FAULT, 1);
                }
                else
                {
                    setFault(ID_HUB_DISC_FAULT, 0);
                }
                break;
            case (SDC_TSMS):
                if (!sdc_mux.tsms_stat && sdc_mux.r_hub_stat)
                {
                    // setFault(ID_TSMS_DISC_FAULT, 1);
                }
                else
                {
                    setFault(ID_TSMS_DISC_FAULT, 0);
                }
                break;
        }
    }
}

/**
 * @brief Update Status of SDC Mux and Send on CAN
 *
 */
void monitorSDCPeriodic()
{
    static uint8_t index = 0;
    // static sdc_nodes_t sdc_nodes_raw;
    // bool *nodes = (bool *) &sdc_nodes_raw;

    uint8_t stat =  (uint8_t) PHAL_readGPIO(SDC_MUX_DATA_GPIO_Port, SDC_MUX_DATA_Pin);
    // uint8_t main_stat; //y0
    // uint8_t c_stop_stat; //y1
    // uint8_t inertia_stat; //y2
    // uint8_t bots_stat; //y3
    // uint8_t nc; //y4
    // uint8_t bspd_stat; //y5
    // uint8_t bms_stat; //y6
    // uint8_t imd_stat; //y7
    // uint8_t r_stop_stat; //y8
    // uint8_t l_stop_stat; //y9
    // uint8_t hvd_stat; //y10
    // uint8_t r_hub_stat; //y11
    // uint8_t tsms_stat; //y12
    // uint8_t pchg_out_stat; //y13
    // *(nodes+index++) = stat;
    switch(index)
    {
        case 0:
            sdc_mux.main_stat = stat;
            break;
        case 1:
            sdc_mux.c_stop_stat = stat;
            break;
        case 2:
            sdc_mux.inertia_stat = stat;
            break;
        case 3:
            sdc_mux.bots_stat = stat;
            break;
        case 4:
            break;
        case 5:
            sdc_mux.bspd_stat = stat;
            break;
        case 6:
            sdc_mux.bms_stat = stat;
            break;
        case 7:
            sdc_mux.imd_stat = stat;
            break;
        case 8:
            sdc_mux.r_stop_stat = stat;
            break;
        case 9:
            sdc_mux.l_stop_stat = stat;
            break;
        case 10:
            sdc_mux.hvd_stat = stat;
            break;
        case 11:
            sdc_mux.r_hub_stat = stat;
            break;
        case 12:
            sdc_mux.tsms_stat = stat;
            break;
        case 13:
            sdc_mux.pchg_out_stat = stat;
            break;
        default:
            break;
    }
    index++;
    if (index == SDC_MUX_HIGH_IDX)
    {
        index = 0;
        // sdc_mux = sdc_nodes_raw;
        SEND_SDC_STATUS(sdc_mux.imd_stat, sdc_mux.bms_stat, sdc_mux.bspd_stat, sdc_mux.bots_stat,
                sdc_mux.inertia_stat, sdc_mux.c_stop_stat, sdc_mux.main_stat, sdc_mux.r_stop_stat, sdc_mux.l_stop_stat,
                sdc_mux.hvd_stat, sdc_mux.r_hub_stat, sdc_mux.tsms_stat, sdc_mux.pchg_out_stat);
    }

    PHAL_writeGPIO(SDC_MUX_S0_GPIO_Port, SDC_MUX_S0_Pin, (index & 0x01));
    PHAL_writeGPIO(SDC_MUX_S1_GPIO_Port, SDC_MUX_S1_Pin, (index & 0x02));
    PHAL_writeGPIO(SDC_MUX_S2_GPIO_Port, SDC_MUX_S2_Pin, (index & 0x04));
    PHAL_writeGPIO(SDC_MUX_S3_GPIO_Port, SDC_MUX_S3_Pin, (index & 0x08));
}
