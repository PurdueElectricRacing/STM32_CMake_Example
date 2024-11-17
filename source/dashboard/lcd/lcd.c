#include "lcd.h"

#include "can_parse.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "nextion.h"
#include "pedals.h"
#include "common/faults/faults.h"
#include "common_defs.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

volatile page_t curr_page;              // Current page displayed on the LCD
volatile page_t prev_page;              // Previous page displayed on the LCD
uint16_t cur_fault_buf_ndx;             // Current index in the fault buffer
volatile uint16_t fault_buf[5] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};   // Buffer of displayed faults
bool sendFirsthalf;                     // Flag for sending data to data page
char *errorText;                        // Pointer to data to display for the Error, Warning, and Critical Fault codes
extern uint16_t filtered_pedals;        // Global from pedals module for throttle display
extern q_handle_t q_tx_can;             // Global queue for CAN tx
extern q_handle_t q_fault_history;      // Global queue from fault library for fault history
volatile settings_t settings;           // Data for the settings page
volatile tv_settings_t tv_settings;     // Data for the tvsettings page
volatile driver_config_t driver_config; // Data for the driver page
volatile fault_config_t fault_config;   // Data for the faults page
race_page_t race_page_data;             // Data for the race page
extern lcd_t lcd_data;
uint8_t fault_time_displayed;           // Amount of units of time that the fault has been shown to the driver


// update helper prototypes
void update_driver_page();
void update_cooling_page();
void update_tv_page();
void update_faults_page();
// move up helper prototypes
void move_up_tv();
void move_up_cooling();
void move_up_driver();
// move down helper prototypes
void move_down_tv();
void move_down_cooling();
void move_down_driver();
// select helper prototypes
void select_race();
void select_driver();
void select_tv();
void select_cooling();
// update helper prototypes
void update_race_page_group1();
void update_race_page_group2();
void update_race_page();
void updateSDCStatus(uint8_t status, char *element);
void setFaultIndicator(uint16_t fault, char *element);
// string helper prototypes
void append_char(char *str, char ch, size_t max_len); // Append a character to a string
char *int_to_char(int16_t val, char *val_to_send);  // Convert integer value to character for the nextion interface


// Call initially to ensure the LCD is initialized to the proper value -
// should be replaced with the struct prev page stuff eventually
int zeroEncoder(volatile int8_t* start_pos) {
    // ! uncomment this when encoder is implemented
    // Collect initial raw reading from encoder
    // uint8_t raw_enc_a = PHAL_readGPIO(ENC_A_GPIO_Port, ENC_A_Pin);
    // uint8_t raw_enc_b = PHAL_readGPIO(ENC_B_GPIO_Port, ENC_B_Pin);
    // uint8_t raw_res = (raw_enc_b | (raw_enc_a << 1));
    // *start_pos = raw_res;
    lcd_data.encoder_position = 0;

    // Set page (leave preflight)
    updatePage();
    return 1;
}

// Initialize the LCD screen
// Preflight will be shown on power on, then reset to RACE
void initLCD() {
    curr_page = PAGE_RACE;
    prev_page = PAGE_PREFLIGHT;
    errorText = 0;
    settings = (settings_t) {0, 0, 0, 0, 0, 0, 0, 0};
    sendFirsthalf = true;
    tv_settings = (tv_settings_t) {true, 0, 12, 100, 40 };
    fault_config_t fault_config = {FAULT1, FAULT1}; // TODO
}

void updatePage() {
    // Only update the encoder if we are on a "selectable" page
    if ((curr_page != PAGE_ERROR) && (curr_page != PAGE_WARNING) && (curr_page != PAGE_FATAL))
    {
        curr_page = lcd_data.encoder_position;
        fault_time_displayed = 0;
    }

    // If we do not detect a page update (most notably detect if encoder did not move), do nothing
    if (curr_page == prev_page) {
        return;
    }

    // Parse the page that was passed into the function
    switch (curr_page) {
        case PAGE_LOGGING:
            prev_page = PAGE_LOGGING;
            set_page(LOGGING_STRING);
            break;
        case PAGE_DRIVER:
            prev_page = PAGE_DRIVER;
            set_page(DRIVER_STRING);
            update_driver_page();
            break;
        case PAGE_SDCINFO:
            prev_page = PAGE_SDCINFO;
            set_page(SDCINFO_STRING);
            break;
        case PAGE_TVSETTINGS:
            prev_page = PAGE_TVSETTINGS;
            set_page(TVSETTINGS_STRING);
            update_tv_page();
            break;
        case PAGE_ERROR:
            set_page(ERR_STRING);
            set_text(ERR_TXT, NXT_TEXT, errorText);
            break;
        case PAGE_WARNING:
            set_page(WARN_STRING);
            set_text(ERR_TXT, NXT_TEXT, errorText);
            break;
        case PAGE_FATAL:
            set_page(FATAL_STRING);
            set_text(ERR_TXT, NXT_TEXT, errorText);
            break;
        case PAGE_RACE:
            prev_page = PAGE_RACE;
            set_page(RACE_STRING);
            break;
        case PAGE_DATA:
            prev_page = PAGE_DATA;
            set_page(DATA_STRING);
            break;
        case PAGE_SETTINGS:
            prev_page = PAGE_SETTINGS;
            set_page(SETTINGS_STRING);
            update_cooling_page();
            break;
        case PAGE_FAULTS:
            prev_page = PAGE_FAULTS;
            set_page(FAULT_STRING);
            update_faults_page();
            break;
    }
}

void moveUp() {
    switch (curr_page) {
        case PAGE_LOGGING:
            //TODO
            break;
        case PAGE_DRIVER:
            move_up_driver();
            break;
        case PAGE_TVSETTINGS:
            move_up_tv();
            break;
        case PAGE_SETTINGS:
            move_up_cooling();
            break;
        case PAGE_FAULTS:
            // TODO support new faults page
            break;
    }
}

void moveDown() {
    switch (curr_page) {
        case PAGE_LOGGING:
            //TODO
            break;
        case PAGE_DRIVER:
            move_down_driver();
            break;
        case PAGE_TVSETTINGS:
            move_down_tv();
            break;
        case PAGE_SETTINGS:
            move_down_cooling();
            break;
        case PAGE_FAULTS:
            // TODO support new faults page
            break;
    }
}

void selectItem() {

    // User has selected to clear the current fault screen
    if ((curr_page == PAGE_ERROR) || (curr_page == PAGE_FATAL) || (curr_page == PAGE_WARNING))
    {
        // Go back to where we were before
        curr_page = prev_page;
        // so select item doesnt't break
        prev_page = PAGE_PREFLIGHT;
        fault_time_displayed = 0;
        updatePage();
        return;
    }

    switch (curr_page) {
        case PAGE_LOGGING:
            SEND_DASHBOARD_START_LOGGING(1);
            break;
        case PAGE_DRIVER:
            select_driver();
            break;
        case PAGE_TVSETTINGS:
            select_tv();
            break;
        case PAGE_SETTINGS:
            select_cooling();
            break;
        case PAGE_RACE:
            select_race();
            break;
        case PAGE_FAULTS:
            // TODO support new faults page
            break;
    }
}

void updateDataPages() {
    if (curr_page == PAGE_RACE) {
        update_race_page();
        return;
    }

    if (curr_page == PAGE_DATA) {
        set_value(POW_LIM_BAR, NXT_VALUE, 0);
        set_value(THROT_BAR, NXT_VALUE, (int) ((filtered_pedals / 4095.0) * 100));
    }

    return;
}

void sendTVParameters() {
    SEND_DASHBOARD_TV_PARAMETERS(tv_settings.tv_enable_selected, tv_settings.tv_deadband_val, tv_settings.tv_intensity_val, tv_settings.tv_p_val);
}


bool isFaultAlreadyInBuffer(uint16_t fault) {
    for (int j = 0; j < 5; j++) {
        if (fault_buf[j] == fault) {
            return true;
        }
    }
    return false;
}

bool insertFaultIntoBuffer(uint16_t fault) {
    for (uint8_t k = 0; k < 5; k++) {
        if (fault_buf[cur_fault_buf_ndx] != 0xFFFF) {
            if (!checkFault(fault_buf[cur_fault_buf_ndx]) ||
                (faultArray[fault].priority > faultArray[fault_buf[cur_fault_buf_ndx]].priority)) {
                fault_buf[cur_fault_buf_ndx] = fault;
                cur_fault_buf_ndx = (cur_fault_buf_ndx + 1) % 5;
                return true;
            }
        } else {
            fault_buf[cur_fault_buf_ndx] = fault;
            cur_fault_buf_ndx = (cur_fault_buf_ndx + 1) % 5;
            return true;
        }
        cur_fault_buf_ndx = (cur_fault_buf_ndx + 1) % 5;
    }
    return false;
}

bool processFaults() {
    bool pageUpdateRequired = false;

    // Process up to 5 faults each time for now
    for (int i = 0; i < 5; i++) {
        uint16_t next_to_check = 0xFFFF;
        bool faultWasInserted = false;

        if (!qReceive(&q_fault_history, &next_to_check)) {
            // Break out if issue or the queue is empty
            break;
        }

        if (isFaultAlreadyInBuffer(next_to_check)) {
            continue;
        }

        faultWasInserted = insertFaultIntoBuffer(next_to_check);
        if (!faultWasInserted) {
            qSendToBack(&q_fault_history, &next_to_check);
        } else {
            pageUpdateRequired = true;
        }
    }

    return pageUpdateRequired;
}

void updateFaultDisplay() {
    if (!(curr_page == PAGE_ERROR || curr_page == PAGE_WARNING || curr_page == PAGE_FATAL)) {
        fault_time_displayed = 0;
        return;
    }

    if (++fault_time_displayed > 8) {
        curr_page = prev_page;
        prev_page = PAGE_PREFLIGHT;
        updatePage();
        return;
    }

    // No new fault to display
    if (qIsEmpty(&q_fault_history) && (most_recent_latched == 0xFFFF)) {
        return;
    }

    bool pageUpdateRequired = processFaults();

    // Set the alert page to show based on most_recent_latched
    if (most_recent_latched != 0xFFFF) {
        curr_page = faultArray[most_recent_latched].priority + 9;
        errorText = faultArray[most_recent_latched].screen_MSG;
        pageUpdateRequired = true;
    }

    // Update page if required
    if (pageUpdateRequired) {
        updatePage();
    }

    // Await next fault
    most_recent_latched = 0xFFFF;
}

void updateFaultPageIndicators() {
    if (curr_page != PAGE_FAULTS) {
        return;
    }

    setFaultIndicator(fault_buf[0], FAULT_1_TXT);
    setFaultIndicator(fault_buf[1], FAULT_2_TXT);
    setFaultIndicator(fault_buf[2], FAULT_3_TXT);
    setFaultIndicator(fault_buf[3], FAULT_4_TXT);
    setFaultIndicator(fault_buf[4], FAULT_5_TXT);
}

void updateSDCDashboard() {
    static uint8_t update_group = 0U;
    if (curr_page != PAGE_SDCINFO) {
        return;
    }

    // cycle through the update groups (5 elements each)
    update_group++;
    switch (update_group) {
        case 1:
            updateSDCStatus(can_data.precharge_hb.IMD, SDC_IMD_STAT_TXT); // IMD from ABOX
            updateSDCStatus(can_data.precharge_hb.BMS, SDC_BMS_STAT_TXT);
            updateSDCStatus(!checkFault(ID_BSPD_LATCHED_FAULT), SDC_BSPD_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.BOTS, SDC_BOTS_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.inertia, SDC_INER_STAT_TXT);
            break;
        case 2:
            updateSDCStatus(can_data.sdc_status.c_estop, SDC_CSTP_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.main, SDC_MAIN_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.r_estop, SDC_RSTP_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.l_estop, SDC_LSTP_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.HVD, SDC_HVD_STAT_TXT);
            break;
        case 3:
            updateSDCStatus(can_data.sdc_status.hub, SDC_RHUB_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.TSMS, SDC_TSMS_STAT_TXT);
            updateSDCStatus(can_data.sdc_status.pchg_out, SDC_PCHG_STAT_TXT);
            //todo set first trip from latest change in the sdc
            update_group = 0U;
            break;
        default:
            update_group = 0U;
            break;
    }
}

// ! Helper function definitions

void update_driver_page() {
    driver_config.curr_hover = DRIVER_DEFAULT_SELECT;
    set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
    set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_BG);

    if (driver_config.curr_select == DRIVER_DEFAULT_SELECT)
    {
        set_value(DRIVER_DEFAULT_OP, NXT_VALUE, 1);
    }
    else
    {
        set_value(DRIVER_DEFAULT_OP, NXT_VALUE, 0);
    }

    if (driver_config.curr_select == DRIVER_TYLER_SELECT)
    {
        set_value(DRIVER_TYLER_OP, NXT_VALUE, 1);
    }
    else
    {
        set_value(DRIVER_TYLER_OP, NXT_VALUE, 0);
    }

    if (driver_config.curr_select == DRIVER_RUHAAN_SELECT)
    {
        set_value(DRIVER_RUHAAN_OP, NXT_VALUE, 1);
    }
    else
    {
        set_value(DRIVER_RUHAAN_OP, NXT_VALUE, 0);
    }

    if (driver_config.curr_select == DRIVER_LUKE_SELECT)
    {
        set_value(DRIVER_LUKE_OP, NXT_VALUE, 1);
    }
    else
    {
        set_value(DRIVER_LUKE_OP, NXT_VALUE, 0);
    }
}

void update_cooling_page() {
    // Parsed value represents:
    char parsed_value[3] = "\0";
    settings.curr_hover = DT_FAN_HOVER;                                     // Set hover
    set_value(DT_FAN_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);         // Set t2 with settings hover
    set_value(DT_FAN_BAR, NXT_VALUE, settings.d_fan_val);                   // Set progress bar for j0
    set_value(DT_FAN_VAL, NXT_FONT_COLOR, SETTINGS_BAR_BG);                         // Set color for t8 (background of bar?)
    set_text(DT_FAN_VAL, NXT_TEXT, int_to_char(settings.d_fan_val, parsed_value));  // Set fan value for t8
    bzero(parsed_value, 3);                                                         // Clear our char buffer

    // Set drivetrain pump selector color
    if (settings.d_pump_selected) {
        set_value(DT_PUMP_OP, NXT_FONT_COLOR, SETTINGS_UV_SELECT);
    }
    else {
        set_value(DT_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
    }

    // Set drivetrain pump selector status
    set_value(DT_PUMP_OP, NXT_VALUE, settings.d_pump_selected);

    // Set Battery fan c3 (Pump 1?)
    // todo: Why is this here?
    if (settings.b_fan2_selected) {
        set_value(B_FAN2_OP, NXT_FONT_COLOR, SETTINGS_UV_SELECT);
    }
    else {
        set_value(B_FAN2_OP, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
    }

    // Set value for c3 battery pump 1
    set_value(B_FAN2_OP, NXT_VALUE, settings.b_fan2_selected);
    if (settings.b_pump_selected) {
        set_value(B_PUMP_OP, NXT_FONT_COLOR, SETTINGS_UV_SELECT);
    }
    else {
        set_value(B_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
    }

    // Set Battery Pump 2 value
    set_value(B_PUMP_OP, NXT_VALUE, settings.b_pump_selected);

    // Set battery fan bar, text, color
    set_value(B_FAN1_BAR, NXT_VALUE, settings.b_fan_val);
    set_text(B_FAN1_VAL, NXT_TEXT, int_to_char(settings.b_fan_val, parsed_value));
    bzero(parsed_value, 3);
    set_value(B_FAN1_VAL, NXT_FONT_COLOR, SETTINGS_BAR_BG);
}

void update_tv_page() {
    // Parsed value represents:
    char parsed_value[3] = "\0";

    // Establish hover position
    tv_settings.curr_hover = TV_INTENSITY_HOVER;

    // Set background colors
    set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
    set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
    set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);

    // Set displayed data
    set_value(TV_INTENSITY_FLT, NXT_VALUE, tv_settings.tv_intensity_val);
    set_value(TV_PROPORTION_FLT, NXT_VALUE, tv_settings.tv_p_val);
    set_text(TV_DEAD_TXT, NXT_TEXT, int_to_char(tv_settings.tv_deadband_val, parsed_value));
    bzero(parsed_value, 3);
    set_value(TV_ENABLE_OP, NXT_VALUE, tv_settings.tv_enable_selected);
}

void update_faults_page() {
    if (fault_buf[0] == 0xFFFF)
    {
        set_text(FAULT_1_TXT, NXT_TEXT, FAULT_NONE_STRING);
    }
    else
    {
        set_text(FAULT_1_TXT, NXT_TEXT, faultArray[fault_buf[0]].screen_MSG);
    }
    if (fault_buf[1] == 0xFFFF)
    {
        set_text(FAULT_2_TXT, NXT_TEXT, FAULT_NONE_STRING);
    }
    else
    {
        set_text(FAULT_2_TXT, NXT_TEXT, faultArray[fault_buf[1]].screen_MSG);
    }

    if (fault_buf[2] == 0xFFFF)
    {
        set_text(FAULT_3_TXT, NXT_TEXT, FAULT_NONE_STRING);
    }
    else
    {
        set_text(FAULT_3_TXT, NXT_TEXT, faultArray[fault_buf[2]].screen_MSG);
    }

    if (fault_buf[3] == 0xFFFF)
    {
        set_text(FAULT_4_TXT, NXT_TEXT, FAULT_NONE_STRING);
    }
    else
    {
        set_text(FAULT_4_TXT, NXT_TEXT, faultArray[fault_buf[3]].screen_MSG);
    }

    if (fault_buf[4] == 0xFFFF)
    {
        set_text(FAULT_5_TXT, NXT_TEXT, FAULT_NONE_STRING);
    }
    else
    {
        set_text(FAULT_5_TXT, NXT_TEXT, faultArray[fault_buf[4]].screen_MSG);
    }
}

void move_up_tv() {
    char parsed_value[3] = "\0";
    // If Intensity is selected
    if (tv_settings.curr_hover == TV_INTENSITY_SELECTED)
    {
        // Increase the intensity value
        tv_settings.tv_intensity_val = (tv_settings.tv_intensity_val + 5) % 1000;

        // Update the page items
        set_value(TV_INTENSITY_FLT, NXT_VALUE, tv_settings.tv_intensity_val);
    }
    else if (tv_settings.curr_hover == TV_INTENSITY_HOVER)
    {
        // Wrap around to enable
        tv_settings.curr_hover = TV_ENABLE_HOVER;

        // Update the background
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
    }
    else if (tv_settings.curr_hover == TV_P_SELECTED)
    {
        // Increase the p value
        tv_settings.tv_p_val = (tv_settings.tv_p_val + 5) % 1000;

        // Update the page items
        set_value(TV_PROPORTION_FLT, NXT_VALUE, tv_settings.tv_p_val);

    }
    else if (tv_settings.curr_hover == TV_P_HOVER)
    {
        // Scroll up to Intensity
        tv_settings.curr_hover = TV_INTENSITY_HOVER;

        // Update the background
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_DEADBAND_SELECTED)
    {
        // Increase the deadband value
        tv_settings.tv_deadband_val = (tv_settings.tv_deadband_val + 1) % 30;

        // Update the page items
        set_text(TV_DEAD_TXT, NXT_TEXT, int_to_char(tv_settings.tv_deadband_val, parsed_value));
        bzero(parsed_value, 3);

    }
    else if (tv_settings.curr_hover == TV_DEADBAND_HOVER)
    {
        // Scroll up to P
        tv_settings.curr_hover = TV_P_HOVER;

        // Update the background
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_ENABLE_HOVER)
    {
        // Scroll up to deadband
        tv_settings.curr_hover = TV_DEADBAND_HOVER;
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else
    {
        // ?
    }
}

void move_up_cooling() {
    char parsed_value[3] = "\0";
    switch (settings.curr_hover) {
        case DT_FAN_HOVER:
            set_value(DT_FAN_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(B_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = PUMP_HOVER;
            break;
        case DT_PUMP_HOVER:
            set_value(DT_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(DT_FAN_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = DT_FAN_HOVER;
            break;
        case FAN1_HOVER:
            set_value(B_FAN1_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(DT_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = DT_PUMP_HOVER;
            break;
        case FAN2_HOVER:
            set_value(B_FAN2_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(B_FAN1_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = FAN1_HOVER;
            break;
        case PUMP_HOVER:
            set_value(B_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(B_FAN2_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = FAN2_HOVER;
            break;
        case DT_FAN_SELECT:
            settings.d_fan_val /= 10;
            settings.d_fan_val *= 10;
            settings.d_fan_val = (settings.d_fan_val == 100) ? 0 : settings.d_fan_val + 10;
            set_value(DT_FAN_BAR, NXT_VALUE, settings.d_fan_val);
            set_text(DT_FAN_VAL, NXT_TEXT, int_to_char(settings.d_fan_val, parsed_value));
            bzero(parsed_value, 3);
            set_value(DT_FAN_VAL, NXT_FONT_COLOR, BLACK);
            break;
        case FAN1_SELECT:
            settings.b_fan_val /= 10;
            settings.b_fan_val *= 10;
            settings.b_fan_val = (settings.b_fan_val == 100) ? 0 : settings.b_fan_val + 10;
            set_value(B_FAN1_BAR, NXT_VALUE, settings.b_fan_val);
            set_text(B_FAN1_VAL, NXT_TEXT, int_to_char(settings.b_fan_val, parsed_value));
            bzero(parsed_value, 3);
            set_value(B_FAN1_VAL, NXT_FONT_COLOR, BLACK);
            break;
    }
}

void move_up_driver() {
    if (driver_config.curr_hover == DRIVER_DEFAULT_SELECT)
    {
        // Wrap around to enable
        driver_config.curr_hover = DRIVER_LUKE_SELECT;
        driver_config.curr_select = DRIVER_LUKE_SELECT;
        // Update the background
        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
    }
    else if (driver_config.curr_hover == DRIVER_TYLER_SELECT)
    {
        driver_config.curr_hover = DRIVER_DEFAULT_SELECT;
        driver_config.curr_select = DRIVER_DEFAULT_SELECT;

        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (driver_config.curr_hover == DRIVER_RUHAAN_SELECT)
    {
        driver_config.curr_hover = DRIVER_TYLER_SELECT;
        driver_config.curr_select = DRIVER_TYLER_SELECT;
        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (driver_config.curr_hover == DRIVER_LUKE_SELECT)
    {
        driver_config.curr_hover = DRIVER_RUHAAN_SELECT;
        driver_config.curr_select = DRIVER_RUHAAN_SELECT;
        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    }
}

void move_down_tv() {
    char parsed_value[3] = "\0";
    if (tv_settings.curr_hover == TV_INTENSITY_SELECTED)
    {
        // Decrease the intensity value
        if (tv_settings.tv_intensity_val == 0)
        {
            tv_settings.tv_intensity_val = 100;
        }
        else
        {
            tv_settings.tv_intensity_val-= 5;
        }

        // Update the page item
        set_value(TV_INTENSITY_FLT, NXT_VALUE, tv_settings.tv_intensity_val);
    }
    else if (tv_settings.curr_hover == TV_INTENSITY_HOVER)
    {
        // Scroll down to P
        tv_settings.curr_hover = TV_P_HOVER;

        // Update the background
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_P_SELECTED)
    {
        // Decrease the P value
        if (tv_settings.tv_p_val == 0)
        {
            tv_settings.tv_p_val = 100;
        }
        else
        {
            tv_settings.tv_p_val-= 5;
        }

        // Update the page items
        set_value(TV_PROPORTION_FLT, NXT_VALUE, tv_settings.tv_p_val);

    }
    else if (tv_settings.curr_hover == TV_P_HOVER)
    {
        // Scroll down to deadband
        tv_settings.curr_hover = TV_DEADBAND_HOVER;

        // Update the background
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_DEADBAND_SELECTED)
    {
        // Decrease the deadband value
        if (tv_settings.tv_deadband_val == 0)
        {
            tv_settings.tv_deadband_val = 30;
        }
        else
        {
            tv_settings.tv_deadband_val--;
        }

        // Update the page items
        set_text(TV_DEAD_TXT, NXT_TEXT, int_to_char(tv_settings.tv_deadband_val, parsed_value));
        bzero(parsed_value, 3);

    }
    else if (tv_settings.curr_hover == TV_DEADBAND_HOVER)
    {
        // Scroll down to enable
        tv_settings.curr_hover = TV_ENABLE_HOVER;

        // Update the background
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
    }
    else if (tv_settings.curr_hover == TV_ENABLE_HOVER)
    {
        // Scroll down to intensity
        tv_settings.curr_hover = TV_INTENSITY_HOVER;
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else
    {
        // ?
    }
}

void move_down_cooling() {
    char parsed_value[3] = "\0";
    switch (settings.curr_hover) {
        case DT_FAN_HOVER:
            set_value(DT_FAN_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(DT_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = DT_PUMP_HOVER;
            break;
        case DT_PUMP_HOVER:
            set_value(DT_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(B_FAN1_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = FAN1_HOVER;
            break;
        case FAN1_HOVER:
            set_value(B_FAN1_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(B_FAN2_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = FAN2_HOVER;
            break;
        case FAN2_HOVER:
            set_value(B_FAN2_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(B_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = PUMP_HOVER;
            break;
        case PUMP_HOVER:
            set_value(B_PUMP_TXT, NXT_BACKGROUND_COLOR, SETTINGS_BG);
            set_value(DT_FAN_TXT, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
            settings.curr_hover = DT_FAN_HOVER;
            break;
        case DT_FAN_SELECT:
            settings.d_fan_val /= 10;
            settings.d_fan_val *= 10;
            settings.d_fan_val = (settings.d_fan_val == 0) ? 100 : settings.d_fan_val - 10;
            set_value(DT_FAN_BAR, NXT_VALUE, settings.d_fan_val);
            set_text(DT_FAN_VAL, NXT_TEXT, int_to_char(settings.d_fan_val, parsed_value));
            bzero(parsed_value, 3);
            set_value(DT_FAN_VAL, NXT_FONT_COLOR, BLACK);
            break;
        case FAN1_SELECT:
            settings.b_fan_val /= 10;
            settings.b_fan_val *= 10;
            settings.b_fan_val = (settings.b_fan_val == 0) ? 100 : settings.b_fan_val - 10;
            set_value(B_FAN1_BAR, NXT_VALUE, settings.b_fan_val);
            set_text(B_FAN1_VAL, NXT_TEXT, int_to_char(settings.b_fan_val, parsed_value));
            bzero(parsed_value, 3);
            set_value(B_FAN1_VAL, NXT_FONT_COLOR, BLACK);
            break;
    }
}

void move_down_driver() {
    if (driver_config.curr_hover == DRIVER_DEFAULT_SELECT)
    {
        driver_config.curr_hover = DRIVER_TYLER_SELECT;
        driver_config.curr_select = DRIVER_TYLER_SELECT;
        // Update the background
        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (driver_config.curr_hover == DRIVER_TYLER_SELECT)
    {
        driver_config.curr_hover = DRIVER_RUHAAN_SELECT;
        driver_config.curr_select = DRIVER_RUHAAN_SELECT;
        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (driver_config.curr_hover == DRIVER_RUHAAN_SELECT)
    {
        driver_config.curr_hover = DRIVER_LUKE_SELECT;
        driver_config.curr_select = DRIVER_LUKE_SELECT;
        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
    }
    else if (driver_config.curr_hover == DRIVER_LUKE_SELECT)
    {
        driver_config.curr_hover = DRIVER_DEFAULT_SELECT;
        driver_config.curr_select = DRIVER_DEFAULT_SELECT;
        set_value(DRIVER_DEFAULT_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(DRIVER_TYLER_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_RUHAAN_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(DRIVER_LUKE_TXT, NXT_BACKGROUND_COLOR, TV_BG);
    }
}

void select_race() {
    tv_settings.tv_enable_selected = (tv_settings.tv_enable_selected == 0);
    set_value(RACE_TV_ON, NXT_VALUE, tv_settings.tv_enable_selected);
}

void select_tv() {
    // So if we hit select on an already selected item, unselect it (switch to hover)
    if (tv_settings.curr_hover == TV_INTENSITY_HOVER)
    {
        tv_settings.curr_hover = TV_INTENSITY_SELECTED;
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, ORANGE);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
        // todo Rot encoder state should let us scroll through value options
        // for now just use buttons for move up and move down
    }
    else if (tv_settings.curr_hover == TV_INTENSITY_SELECTED)
    {
        // "submit" -> CAN payload will update automatically? decide
        // Think about edge case when the user leaves the page? Can they without unselecting -> no. What if fault?
        tv_settings.curr_hover = TV_INTENSITY_HOVER;
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
        // rot encoder state goes back to page move instead of value move
    }
    else if (tv_settings.curr_hover == TV_P_HOVER)
    {
        tv_settings.curr_hover = TV_P_SELECTED;
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, ORANGE);
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_P_SELECTED)
    {
        tv_settings.curr_hover = TV_P_HOVER;
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_DEADBAND_HOVER)
    {
        tv_settings.curr_hover = TV_DEADBAND_SELECTED;
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, ORANGE);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_DEADBAND_SELECTED)
    {
        tv_settings.curr_hover = TV_DEADBAND_HOVER;
        set_value(TV_PROPORTION_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_INTENSITY_FLT, NXT_BACKGROUND_COLOR, TV_BG);
        set_value(TV_DEAD_TXT, NXT_BACKGROUND_COLOR, TV_HOVER_BG);
        set_value(TV_ENABLE_OP, NXT_BACKGROUND_COLOR, TV_BG);
    }
    else if (tv_settings.curr_hover == TV_ENABLE_HOVER)
    {
        // Don't change the curr_hover

        // Toggle the option
        tv_settings.tv_enable_selected = (tv_settings.tv_enable_selected == 0);

        // Set the option
        set_value(TV_ENABLE_OP, NXT_VALUE, tv_settings.tv_enable_selected);

        // Update CAN as necessary
    }
    else
    {
        // ?
    }
}

void select_cooling() {
    switch (settings.curr_hover) {
    case DT_FAN_HOVER:
        // settings.d_fan_selected = !settings.d_fan_selected;
        // set_value(DT_FAN_BAR, NXT_VALUE, settings.d_fan_selected);
        settings.curr_hover = DT_FAN_SELECT;
        set_value(DT_FAN_TXT, NXT_VALUE, SETTINGS_BG);
        set_value(DT_FAN_BAR, NXT_BACKGROUND_COLOR, WHITE);
        set_value(DT_FAN_BAR, NXT_FONT_COLOR, BLACK);
        return;
    case DT_PUMP_HOVER:
        settings.d_pump_selected = !settings.d_pump_selected;
        if (settings.d_pump_selected) {
            set_value(DT_PUMP_OP, NXT_FONT_COLOR, SETTINGS_UV_SELECT);
            set_value(DT_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_BG);
        }
        else {
            set_value(DT_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
        }
        set_value(DT_PUMP_OP, NXT_VALUE, settings.d_pump_selected);
        break;
    case FAN1_HOVER:
        // settings.b_fan1_selected = !settings.b_fan1_selected;
        // set_value(B_FAN1_BAR, NXT_VALUE, settings.b_fan1_selected);
        settings.curr_hover = FAN1_SELECT;
        set_value(B_FAN1_TXT, NXT_VALUE, SETTINGS_BG);
        set_value(B_FAN1_BAR, NXT_BACKGROUND_COLOR, WHITE);
        set_value(B_FAN1_BAR, NXT_FONT_COLOR, BLACK);
        break;
    case FAN2_HOVER:
        settings.b_fan2_selected = !settings.b_fan2_selected;
        if (settings.b_fan2_selected) {
            set_value(B_FAN2_OP, NXT_FONT_COLOR, SETTINGS_UV_SELECT);
            set_value(B_FAN2_OP, NXT_BACKGROUND_COLOR, SETTINGS_BG);
        }
        else {
            set_value(B_FAN2_OP, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
        }
        set_value(B_FAN2_OP, NXT_VALUE, settings.b_fan2_selected);
        break;
    case PUMP_HOVER:
        settings.b_pump_selected = !settings.b_pump_selected;
        if (settings.b_pump_selected) {
        set_value(B_PUMP_OP, NXT_FONT_COLOR, SETTINGS_UV_SELECT);
        set_value(B_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_BG);
        }
        else {
            set_value(B_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_HOVER_BG);
        }
        set_value(B_PUMP_OP, NXT_VALUE, settings.b_pump_selected);
        break;
    case DT_FAN_SELECT:
        settings.curr_hover = DT_FAN_HOVER;
        set_value(DT_FAN_TXT, NXT_VALUE, SETTINGS_HOVER_BG);
        set_value(DT_FAN_BAR, NXT_BACKGROUND_COLOR, SETTINGS_BAR_BG);
        set_value(DT_FAN_BAR, NXT_FONT_COLOR, SETTINGS_BAR_FG);
        set_value(DT_FAN_VAL, NXT_FONT_COLOR, SETTINGS_BAR_BG);
        break;
    case FAN1_SELECT:
        settings.curr_hover = FAN1_HOVER;
        set_value(B_FAN1_TXT, NXT_VALUE, SETTINGS_HOVER_BG);
        set_value(B_FAN1_BAR, NXT_BACKGROUND_COLOR, SETTINGS_BAR_BG);
        set_value(B_FAN1_BAR, NXT_FONT_COLOR, SETTINGS_BAR_FG);
        set_value(B_FAN1_VAL, NXT_FONT_COLOR, SETTINGS_BAR_BG);
        break;
    }
    SEND_COOLING_DRIVER_REQUEST(settings.d_pump_selected, settings.d_fan_val, settings.b_fan2_selected, settings.b_pump_selected, settings.b_fan_val);
}

void select_driver() {
    switch(driver_config.curr_hover)
    {
        case DRIVER_DEFAULT_SELECT:
            set_value(DRIVER_DEFAULT_OP, NXT_VALUE, 1);
            set_value(DRIVER_TYLER_OP, NXT_VALUE, 0);
            set_value(DRIVER_RUHAAN_OP, NXT_VALUE, 0);
            set_value(DRIVER_LUKE_OP, NXT_VALUE, 0);
            break;
        case DRIVER_TYLER_SELECT:
            set_value(DRIVER_DEFAULT_OP, NXT_VALUE, 0);
            set_value(DRIVER_TYLER_OP, NXT_VALUE, 1);
            set_value(DRIVER_RUHAAN_OP, NXT_VALUE, 0);
            set_value(DRIVER_LUKE_OP, NXT_VALUE, 0);
            break;
        case DRIVER_RUHAAN_SELECT:
            set_value(DRIVER_DEFAULT_OP, NXT_VALUE, 0);
            set_value(DRIVER_TYLER_OP, NXT_VALUE, 0);
            set_value(DRIVER_RUHAAN_OP, NXT_VALUE, 1);
            set_value(DRIVER_LUKE_OP, NXT_VALUE, 0);
            break;
        case DRIVER_LUKE_SELECT:
            set_value(DRIVER_DEFAULT_OP, NXT_VALUE, 0);
            set_value(DRIVER_TYLER_OP, NXT_VALUE, 0);
            set_value(DRIVER_RUHAAN_OP, NXT_VALUE, 0);
            set_value(DRIVER_LUKE_OP, NXT_VALUE, 1);
            break;
    }
}

void update_race_page_group1() {
    char temps_buf[5] = "\0"; // TODO adjust buf to proper size (adjust for digits needed)

    if (can_data.rear_motor_temps.stale) {
        set_text(MOT_TEMP, NXT_TEXT, "S");
    }
    else {
        uint8_t motor_temp = MAX(can_data.rear_motor_temps.left_mot_temp, can_data.rear_motor_temps.right_mot_temp);
        int_to_char(motor_temp, temps_buf);
        append_char(temps_buf, 'C', sizeof(temps_buf));

        set_text(MOT_TEMP, NXT_TEXT, temps_buf);
        bzero(temps_buf, sizeof(temps_buf));
    }

    if (can_data.max_cell_temp.stale) {
        set_text(BATT_TEMP, NXT_TEXT, "S");
    }
    else {
        uint16_t batt_temp = can_data.max_cell_temp.max_temp / 10;
        int_to_char(batt_temp, temps_buf);
        append_char(temps_buf, 'C', sizeof(temps_buf));

        set_text(BATT_TEMP, NXT_TEXT, temps_buf);
        bzero(temps_buf, sizeof(temps_buf));
    }

    // TODO update MC_TEMP
}

void update_race_page_group2() {
    if (can_data.main_hb.stale) {
        set_text(CAR_STAT, NXT_TEXT, "S");
        set_value(CAR_STAT, NXT_BACKGROUND_COLOR, BLACK);
    }
    else {
        switch(can_data.main_hb.car_state) {
            case CAR_STATE_PRECHARGING:
                set_value(CAR_STAT, NXT_FONT_COLOR, ORANGE);
                set_text(CAR_STAT, NXT_TEXT, "PRCHG");
                break;
            case CAR_STATE_ENERGIZED:
                set_value(CAR_STAT, NXT_FONT_COLOR, ORANGE);
                set_text(CAR_STAT, NXT_TEXT, "ENER");
                break;
            case CAR_STATE_IDLE:
                set_value(CAR_STAT, NXT_FONT_COLOR, INFO_GRAY);
                set_text(CAR_STAT, NXT_TEXT, "INIT");
                break;
            case CAR_STATE_READY2DRIVE:
                set_value(CAR_STAT, NXT_FONT_COLOR, RACE_GREEN);
                set_text(CAR_STAT, NXT_TEXT, "ON");
                break;
            case CAR_STATE_ERROR:
                set_value(CAR_STAT, NXT_FONT_COLOR, YELLOW);
                set_text(CAR_STAT, NXT_TEXT, "ERR");
                break;
            case CAR_STATE_FATAL:
                set_value(CAR_STAT, NXT_FONT_COLOR, RED);
                set_text(CAR_STAT, NXT_TEXT, "FATAL");
                break;
        }
    }


    // Update the voltage and current
    char batt_buf[5] = "\0"; // 3 digits + 1 unit + \0
    if (can_data.orion_currents_volts.stale) {
        set_text(BATT_VOLT, NXT_TEXT, "S");
        set_text(BATT_CURR, NXT_TEXT, "S");
    }
    else {
        uint16_t voltage = (can_data.orion_currents_volts.pack_voltage / 10);
        int_to_char(voltage, batt_buf);
        append_char(batt_buf, 'V', sizeof(batt_buf));
        set_text(BATT_VOLT, NXT_TEXT, batt_buf);
        bzero(batt_buf, sizeof(batt_buf));

        uint16_t current = (can_data.orion_currents_volts.pack_current / 10);
        int_to_char(current, batt_buf);
        append_char(batt_buf, 'V', sizeof(batt_buf));
        set_text(BATT_CURR, NXT_TEXT, batt_buf);
        bzero(batt_buf, sizeof(batt_buf));
    }
}

void update_race_page() {
    static uint8_t update_group = 0U;
    if (curr_page != PAGE_RACE) {
        return;
    }

    set_value(BRK_BAR, NXT_VALUE, 0); // TODO BRK BAR
    set_value(THROT_BAR, NXT_VALUE, (int) ((filtered_pedals / 4095.0) * 100));

    // update the speed
    if (can_data.rear_wheel_speeds.stale) {
        set_text(SPEED, NXT_TEXT, "S");
    }
    else {
        // Vehicle Speed [m/s] = Wheel Speed [RPM] * 16 [in] * PI * 0.0254 / 60
        char speed_buf[3] = "\0";
        // set_text(SPEED, NXT_TEXT, int_to_char((uint16_t)((float)MAX(can_data.rear_wheel_speeds.left_speed_sensor, can_data.rear_wheel_speeds.right_speed_sensor) * 0.01 * 0.4474), parsed_value));
        uint16_t speed = ((float)can_data.gps_speed.gps_speed * 0.02237); // TODO macro this magic number
        set_text(SPEED, NXT_TEXT, int_to_char(speed, speed_buf));
        bzero(speed_buf, sizeof(speed_buf));
    }

    //cycle the update groups
    update_group++;
    switch (update_group) {
        case 1:
            update_race_page_group1();
            break;
        case 2:
            update_race_page_group2();
            update_group = 0U;
            break;
        default:
            update_group = 0U;
            break;
    }
}

void coolant_out_CALLBACK(CanParsedData_t* msg_data_a) {
    char parsed_value[3] = "\0";
    if (curr_page != PAGE_SETTINGS) {
        settings.d_pump_selected = msg_data_a->coolant_out.dt_pump;
        settings.b_fan2_selected = msg_data_a->coolant_out.bat_pump;
        settings.b_pump_selected = msg_data_a->coolant_out.bat_pump_aux;
        return;
    }

    if (settings.curr_hover != DT_FAN_SELECT) {
        settings.d_fan_val = msg_data_a->coolant_out.dt_fan;
        set_value(DT_FAN_BAR, NXT_VALUE, settings.d_fan_val);
        set_value(DT_FAN_VAL, NXT_FONT_COLOR, BLACK);
        set_text(DT_FAN_VAL, NXT_TEXT, int_to_char(settings.d_fan_val, parsed_value));
        bzero(parsed_value, 3);
    }

    if (settings.curr_hover != FAN1_SELECT) {
        settings.b_fan_val = msg_data_a->coolant_out.bat_fan;
        set_value(B_FAN1_BAR, NXT_VALUE, settings.b_fan_val);
        set_value(B_FAN1_VAL, NXT_FONT_COLOR, settings.b_fan_val);
        set_text(B_FAN1_VAL, NXT_TEXT, int_to_char(settings.b_fan_val, parsed_value));
        bzero(parsed_value, 3);
    }

    set_value(DT_PUMP_OP, NXT_FONT_COLOR, BLACK);
    set_value(B_FAN2_OP, NXT_FONT_COLOR, BLACK);
    set_value(B_PUMP_OP, NXT_FONT_COLOR, BLACK);
    set_value(DT_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_BG);
    set_value(B_FAN2_OP, NXT_BACKGROUND_COLOR, SETTINGS_BG);
    set_value(B_PUMP_OP, NXT_BACKGROUND_COLOR, SETTINGS_BG);
    settings.d_pump_selected = msg_data_a->coolant_out.dt_pump;
    settings.b_fan2_selected = msg_data_a->coolant_out.bat_pump;
    settings.b_pump_selected = msg_data_a->coolant_out.bat_pump_aux;
    set_value(DT_PUMP_OP, NXT_VALUE, settings.d_pump_selected);
    set_value(B_FAN2_OP, NXT_VALUE, settings.b_fan2_selected);
    set_value(B_PUMP_OP, NXT_VALUE, settings.b_pump_selected);

}

void append_char(char *str, char ch, size_t max_len) {
    size_t len = 0;
    while (*str != '\0' && len < max_len - 1) {
        str++;
        len++;
    }
    if (len < max_len - 1) {
        *str++ = ch;
        *str = '\0';
    };
}

char *int_to_char(int16_t val, char *val_to_send) {
    char *orig_ptr = val_to_send;
    if (val < 10) {
        if (val < 0) {
            *val_to_send++ = (char)('-');
            val *= -1;
        }
        *val_to_send = (char)(val + 48);
        return orig_ptr;
    }
    else if (val < 100) {
        *val_to_send++ = val / 10 + 48;
        *val_to_send = val % 10 + 48;
        return orig_ptr;
    }
    else {
        *val_to_send++ = val / 100 + 48;
        *val_to_send++ = val % 100 / 10 + 48;
        *val_to_send = val % 10 + 48;
        return orig_ptr;
    }
}

void setFaultIndicator(uint16_t fault, char *element) {
    if (fault == 0xFFFF) {
        set_value(element, NXT_FONT_COLOR, WHITE);
    } else if (checkFault(fault)) {
        set_value(element, NXT_FONT_COLOR, RED);
    }
    set_value(element, NXT_FONT_COLOR, RACE_GREEN);
}

void updateSDCStatus(uint8_t status, char *element) {
    if (status)
    {
        set_value(element, NXT_BACKGROUND_COLOR, GREEN);
    }
    else
    {
        set_value(element, NXT_BACKGROUND_COLOR, RED);
    }
}
