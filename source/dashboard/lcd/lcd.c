#include "lcd.h"

#include "can_parse.h"
#include "nextion.h"
#include "pedals.h"
#include "common/faults/faults.h"
#include "common_defs.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "menu_system.h"
#include "main.h"

volatile page_t curr_page;              // Current page displayed on the LCD
volatile page_t prev_page;              // Previous page displayed on the LCD
uint16_t cur_fault_buf_ndx;             // Current index in the fault buffer
volatile uint16_t fault_buf[5] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};   // Buffer of displayed faults
char *errorText;                        // Pointer to data to display for the Error, Warning, and Critical Fault codes
extern uint16_t filtered_pedals;        // Global from pedals module for throttle display
extern q_handle_t q_tx_can;             // Global queue for CAN tx
extern q_handle_t q_fault_history;      // Global queue from fault library for fault history
extern lcd_t lcd_data;
uint8_t fault_time_displayed;           // Amount of units of time that the fault has been shown to the driver
extern driver_profile_t driver_profiles[4];


// Driver Page Functions
void update_driver_page();
void move_up_driver();
void move_down_driver();
void select_driver();

// Profile Page Functions
void update_profile_page();
void move_up_profile();
void move_down_profile();
void select_profile();

// Cooling Page Functions
void update_cooling_page();
void move_up_cooling();
void move_down_cooling();
void select_cooling();

// TV Page Functions
void update_tv_page();
void move_up_tv();
void move_down_tv();
void select_tv();

// Faults Page Functions
void update_faults_page();
void move_up_faults();
void move_down_faults();
void select_fault();
void fault_button_callback();

// Race Page Functions
void update_race_telemetry();
void update_race_page();
void select_race();

void select_error_page();

void update_logging_page();
void select_logging();

// Utility Functions
void updateSDCStatus(uint8_t status, char *element);
void setFaultIndicator(uint16_t fault, char *element);

// Page handlers array stored in flash
const page_handler_t page_handlers[] = { // Order must match page_t enum
    [PAGE_RACE]      = {update_race_page, NULL, NULL, select_race}, // No move handlers
    [PAGE_COOLING]   = {update_cooling_page, move_up_cooling, move_down_cooling, select_cooling},
    [PAGE_TVSETTINGS]= {update_tv_page, move_up_tv, move_down_tv, select_tv},
    [PAGE_FAULTS]    = {update_faults_page, move_up_faults, move_down_faults, select_fault},
    [PAGE_SDCINFO]   = {NULL, NULL, NULL, NULL},  // SDCINFO is passive
    [PAGE_DRIVER]    = {update_driver_page, move_up_driver, move_down_driver, select_driver},
    [PAGE_PROFILES]  = {update_profile_page, move_up_profile, move_down_profile, select_profile},
    [PAGE_LOGGING]   = {update_logging_page, NULL, NULL, select_logging},
    [PAGE_APPS]      = {NULL, NULL, NULL, NULL}, // Apps is passive
    [PAGE_PREFLIGHT] = {NULL, NULL, NULL, NULL}, // Preflight is passive
    [PAGE_WARNING]   = {NULL, NULL, NULL, select_error_page}, // Error pages share a select handler
    [PAGE_ERROR]     = {NULL, NULL, NULL, select_error_page},  
    [PAGE_FATAL]     = {NULL, NULL, NULL, select_error_page}
};

menu_element_t race_elements[] = {
    {
        .type = ELEMENT_OPTION,
        .object_name = RACE_TV_ON,
        .current_value = 0,
        .on_change = sendTVParameters
    }
};

menu_page_t race_page = {
    .elements = race_elements,
    .num_elements = sizeof(race_elements) / sizeof(race_elements[0]),
    .current_index = 0,
    .is_element_selected = false
};

menu_element_t cooling_elements[] = {
    {
        .type = ELEMENT_VAL,
        .object_name = DT_FAN_VAL,
        .current_value = 0,
        .min_value = 0,
        .max_value = 100,
        .increment = 25,
    },
    {
        .type = ELEMENT_OPTION,
        .object_name = DT_PUMP_OP,
        .current_value = 0,
        .on_change = sendCoolingParameters
    },
    {
        .type = ELEMENT_VAL,
        .object_name = B_FAN_VAL,
        .current_value = 0,
        .min_value = 0,
        .max_value = 100,
        .increment = 25,
    },
    {
        .type = ELEMENT_OPTION,
        .object_name = B_PUMP_OP,
        .current_value = 0,
        .on_change = sendCoolingParameters
    }
};

menu_page_t cooling_page = {
    .elements = cooling_elements,
    .num_elements = sizeof(cooling_elements) / sizeof(cooling_elements[0]),
    .current_index = 0,
    .is_element_selected = false
};

// TV Settings page menu elements
menu_element_t tv_elements[] = {
    {
        .type = ELEMENT_FLT,
        .object_name = TV_INTENSITY_FLT,
        .current_value = 0,
        .min_value = 0,
        .max_value = 100, // decimal shifted left by 1
        .increment = 5,
        .on_change = NULL // TV Params Sent Periodically
    },
    {
        .type = ELEMENT_FLT,
        .object_name = TV_PROPORTION_FLT,
        .current_value = 40,
        .min_value = 0,
        .max_value = 100, // decimal shifted left by 1
        .increment = 5,
        .on_change = NULL // TV Params Sent Periodically
    },
    {
        .type = ELEMENT_VAL,
        .object_name = TV_DEAD_TXT,
        .current_value = 12,
        .min_value = 0,
        .max_value = 30,
        .increment = 1,
        .on_change = NULL // TV Params Sent Periodically
    },
    {
        .type = ELEMENT_OPTION,
        .object_name = TV_ENABLE_OP,
        .current_value = 0,
        .on_change = NULL // TV Params Sent Periodically
    }
};

menu_page_t tv_page = {
    .elements = tv_elements,
    .num_elements = sizeof(tv_elements) / sizeof(tv_elements[0]),
    .current_index = 0,
    .is_element_selected = false
};

menu_element_t faults_elements[] = {
    {
        .type = ELEMENT_BUTTON,
        .object_name = FAULT1_BUTTON,
        .on_change = fault_button_callback // clear fault
    },
    {
        .type = ELEMENT_BUTTON,
        .object_name = FAULT2_BUTTON,
        .on_change = fault_button_callback // clear fault
    },
    {
        .type = ELEMENT_BUTTON,
        .object_name = FAULT3_BUTTON,
        .on_change = fault_button_callback // clear fault
    },
    {
        .type = ELEMENT_BUTTON,
        .object_name = FAULT4_BUTTON,
        .on_change = fault_button_callback // clear fault
    },
    {
        .type = ELEMENT_BUTTON,
        .object_name = FAULT5_BUTTON,
        .on_change = fault_button_callback // clear fault
    },
    {
        .type = ELEMENT_BUTTON,
        .object_name = CLEAR_BUTTON,
        .on_change = fault_button_callback // clear all faults
    }
};

menu_page_t faults_page = {
    .elements = faults_elements,
    .num_elements = sizeof(faults_elements) / sizeof(faults_elements[0]),
    .current_index = 0,
    .is_element_selected = false
};

menu_element_t driver_elements[] = {
    {
        .type = ELEMENT_LIST,
        .object_name = DRIVER1_LIST,
        .current_value = 1 // Default to driver 1
    },
    {
        .type = ELEMENT_LIST,
        .object_name = DRIVER2_LIST,
        .current_value = 0
    },
    {
        .type = ELEMENT_LIST,
        .object_name = DRIVER3_LIST,
        .current_value = 0
    },
    {
        .type = ELEMENT_LIST,
        .object_name = DRIVER4_LIST,
        .current_value = 0
    }
};

menu_page_t driver_page = {
    .elements = driver_elements,
    .num_elements = sizeof(driver_elements) / sizeof(driver_elements[0]),
    .current_index = 0,
    .is_element_selected = false
};

// Profile page menu elements
menu_element_t profile_elements[] = {
    {
        .type = ELEMENT_VAL,
        .object_name = PROFILE_BRAKE_FLT,
        .current_value = 0,
        .min_value = 0,
        .max_value = 20,
        .increment = 5,
    },
    {
        .type = ELEMENT_VAL,
        .object_name = PROFILE_THROTTLE_FLT,
        .current_value = 0,
        .min_value = 0,
        .max_value = 20,
        .increment = 5,
    },
    {
        .type = ELEMENT_BUTTON,
        .object_name = PROFILE_SAVE_BUTTON,
        .on_change = NULL // todo replace with save function
    }
};

menu_page_t profile_page = {
    .elements = profile_elements,
    .num_elements = sizeof(profile_elements) / sizeof(profile_elements[0]),
    .current_index = 0,
    .is_element_selected = false,
    .saved = true,
};

menu_element_t logging_elements[] = {
    {
        .type = ELEMENT_OPTION,
        .object_name = LOG_OP,
        .current_value = 0,
        .on_change = sendLoggingParameters
    }
};

menu_page_t logging_page = {
    .elements = logging_elements,
    .num_elements = sizeof(logging_elements) / sizeof(logging_elements[0]),
    .current_index = 0,
    .is_element_selected = false
};

// Initialize the LCD screen
// Preflight will be shown on power on, then reset to RACE
void initLCD() {
    curr_page = PAGE_RACE;
    prev_page = PAGE_PREFLIGHT;
    errorText = 0;
    set_baud(115200);
    set_brightness(100);

    readProfiles();
    profile_page.saved = true;

    // Set page (leave preflight)
    updatePage();
}

void updatePage() {
    // Only update the encoder if we are on a "selectable" page
    bool is_error_page = (curr_page == PAGE_ERROR) || (curr_page == PAGE_WARNING) || (curr_page == PAGE_FATAL);
    
    if (!is_error_page) {
        curr_page = lcd_data.encoder_position;
        fault_time_displayed = 0;
    }

    // If we do not detect a page update, do nothing
    if (curr_page == prev_page) {
        return;
    }

    // Only update prev_page for non-error pages
    if (!is_error_page) {
        prev_page = curr_page;
    }

    // Set the page on display
    switch (curr_page) {
        case PAGE_RACE: set_page(RACE_STRING); break;
        case PAGE_COOLING: set_page(COOLING_STRING); break;
        case PAGE_TVSETTINGS: set_page(TVSETTINGS_STRING); break;
        case PAGE_FAULTS: set_page(FAULT_STRING); break;
        case PAGE_SDCINFO: set_page(SDCINFO_STRING); break;
        case PAGE_DRIVER: set_page(DRIVER_STRING); break;
        case PAGE_PROFILES: set_page(DRIVER_CONFIG_STRING); break;
        case PAGE_LOGGING: set_page(LOGGING_STRING); break;
        case PAGE_APPS: set_page(APPS_STRING); break;
        case PAGE_WARNING:
            set_page(WARN_STRING);
            set_text(ERR_TXT, errorText);
            return;
        case PAGE_ERROR:
            set_page(ERR_STRING);
            set_text(ERR_TXT, errorText);
            return;
        case PAGE_FATAL:
            set_page(FATAL_STRING);
            set_text(ERR_TXT, errorText);
            return;
    }

    // Call update handler if available
    if (page_handlers[curr_page].update != NULL) {
        page_handlers[curr_page].update();
    }
}

void moveUp() {
    if (page_handlers[curr_page].move_up != NULL) {
        page_handlers[curr_page].move_up();
    }
}

void moveDown() {
    if (page_handlers[curr_page].move_down != NULL) {
        page_handlers[curr_page].move_down();
    }
}

void selectItem() {
    if (page_handlers[curr_page].select != NULL) {
        page_handlers[curr_page].select();
    }
}

void update_apps_telemetry() {
    if (curr_page != PAGE_APPS) {
        return;
    }

    set_value(BRK_BAR, 0); // todo brake bar
    set_value(THROT_BAR, (int) ((filtered_pedals / 4095.0) * 100));

    set_textf(APPS_BRAKE1_VAL, "%d",raw_adc_values.b1);
    set_textf(APPS_BRAKE2_VAL, "%d",raw_adc_values.b2);
    set_textf(APPS_THROTTLE1_VAL, "%d",raw_adc_values.t1);
    set_textf(APPS_THROTTLE2_VAL, "%d",raw_adc_values.t2);

    uint16_t brake_diff = ABS(raw_adc_values.b1 - raw_adc_values.b2);
    uint16_t brake_dev = (brake_diff * 1000) / 4095.0;
    set_value(APPS_BRAKE_DEV_VAL, brake_dev);

    uint16_t throttle_diff = ABS(raw_adc_values.t1 - raw_adc_values.t2);
    uint16_t throttle_dev = (throttle_diff * 1000) / 4095.0;
    set_value(APPS_THROTTLE_DEV_VAL, throttle_dev);

    if (checkFault(ID_IMPLAUS_DETECTED_FAULT)) {
        set_text(APPS_STATUS, "IMP Detected");
        set_font_color(APPS_STATUS, RED);
    } else {
        set_text(APPS_STATUS, "CLEAR");
        set_font_color(APPS_STATUS, GREEN);
    }
}

void updateTelemetryPages() {
    if (curr_page == PAGE_RACE) {
        update_race_telemetry();
    } else {
        update_apps_telemetry();
    }
}

void sendTVParameters() {
    SEND_DASHBOARD_TV_PARAMETERS(tv_elements[3].current_value, tv_elements[2].current_value, tv_elements[0].current_value, tv_elements[1].current_value);
}

void sendCoolingParameters() {
    SEND_COOLING_DRIVER_REQUEST(cooling_elements[1].current_value, cooling_elements[0].current_value, 0, cooling_elements[3].current_value, cooling_elements[2].current_value);
}

void sendLoggingParameters() {
    SEND_DASHBOARD_START_LOGGING(logging_elements[0].current_value);
}

void updateFaultDisplay() {
    if ((curr_page == PAGE_ERROR || (curr_page == PAGE_WARNING) || (curr_page == PAGE_FATAL)))
    {
        if (++fault_time_displayed > 8)
        {
            curr_page = prev_page;
            prev_page = PAGE_PREFLIGHT;
            updatePage();
        }
    }
    else
    {
        fault_time_displayed = 0;
    }

    // No new fault to display
    if (qIsEmpty(&q_fault_history) && (most_recent_latched == 0xFFFF))
    {
        return;
    }

    // Track if we alrady have this fault in the display buffer
    bool faultAlreadyInBuffer = false;
    bool pageUpdateRequired = false;
    bool faultWasInserted = false;

    // Process up to 5 faults each time for now
    for (int i = 0; i < 5; i++)
    {
        faultAlreadyInBuffer = false;
        uint16_t next_to_check = 0xFFFF;
        faultWasInserted = false;

        if (qReceive(&q_fault_history, &next_to_check))
        {
            // Iterate through fault buffer for existance of fault already
            for (int j = 0; j < 5; j++)
            {
                // This should be based off of the queue item not anything else
                if (fault_buf[j] == next_to_check)
                {
                    faultAlreadyInBuffer = true;
                    break;
                }
            }

            // New fault to add to the display, if room
            if (false == faultAlreadyInBuffer)
            {
                // try all the slots for inserting the fault
                for (uint8_t k = 0; k < 5; k++)
                {
                    // If fault is currently not in our fault buffer, replace it if the current fault is cleared,
                    //  or if the new fault has higher priority
                    if (fault_buf[cur_fault_buf_ndx] != 0xFFFF)
                    {
                        if ((checkFault(fault_buf[cur_fault_buf_ndx]) == false ) ||
                        (faultArray[next_to_check].priority > faultArray[fault_buf[cur_fault_buf_ndx]].priority))
                        {
                            fault_buf[cur_fault_buf_ndx] = next_to_check;
                            faultWasInserted = true;
                            pageUpdateRequired = true;
                            break;
                        }
                    }
                    else
                    {
                        // Empty slot just insert
                        fault_buf[cur_fault_buf_ndx] = next_to_check;
                        faultWasInserted = true;
                        pageUpdateRequired = true;
                        break;
                    }
                    cur_fault_buf_ndx = (cur_fault_buf_ndx + 1) % 5;
                }

                // Put back in the queue if it wasn't processed
                if (false == faultWasInserted)
                {
                    qSendToBack(&q_fault_history, &next_to_check);
                }

            }
        }
        else
        {
            // Break out if issue or the queue is empty
            break;
        }

    }

    // Set the alert page to show based on most_recent_latched
    if ((most_recent_latched != 0xFFFF))
    {
        curr_page = faultArray[most_recent_latched].priority + 9;
        errorText = faultArray[most_recent_latched].screen_MSG;
        pageUpdateRequired = true;
    }

    // Update page if required
    if (pageUpdateRequired)
    {
        updatePage();
    }

    // Await next fault
    most_recent_latched = 0xFFFF;
}

void updateFaultPageIndicators() {
    if (curr_page != PAGE_FAULTS) {
        return;
    }

    setFaultIndicator(fault_buf[0], FAULT1_TXT);
    setFaultIndicator(fault_buf[1], FAULT2_TXT);
    setFaultIndicator(fault_buf[2], FAULT3_TXT);
    setFaultIndicator(fault_buf[3], FAULT4_TXT);
    setFaultIndicator(fault_buf[4], FAULT5_TXT);
}

void updateSDCDashboard() {
    static uint8_t update_group = 0U;
    if (curr_page != PAGE_SDCINFO) {
        return;
    }

    // cycle through the update groups
    update_group ^= 1;
    if (update_group) {
        updateSDCStatus(can_data.precharge_hb.IMD, SDC_IMD_STAT_TXT); // IMD from ABOX
        updateSDCStatus(can_data.precharge_hb.BMS, SDC_BMS_STAT_TXT);
        updateSDCStatus(!checkFault(ID_BSPD_LATCHED_FAULT), SDC_BSPD_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.BOTS, SDC_BOTS_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.inertia, SDC_INER_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.c_estop, SDC_CSTP_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.main, SDC_MAIN_STAT_TXT);
    } else {
        updateSDCStatus(can_data.sdc_status.r_estop, SDC_RSTP_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.l_estop, SDC_LSTP_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.HVD, SDC_HVD_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.hub, SDC_RHUB_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.TSMS, SDC_TSMS_STAT_TXT);
        updateSDCStatus(can_data.sdc_status.pchg_out, SDC_PCHG_STAT_TXT);
        //todo set first trip from latest change in the sdc
    }
}

// ! Helper function definitions

void select_error_page() {
    fault_time_displayed = 0;   // Reset fault timer first
    curr_page = prev_page;      // Return to previous page 
    prev_page = PAGE_PREFLIGHT; // so select item doesnt't break
    updatePage();               // Important: Update the page before returning
    return;
}

void update_driver_page() {
    menu_refresh_page(&driver_page);
}

void move_up_driver() {
    menu_move_up(&driver_page);
}

void move_down_driver() {
    menu_move_down(&driver_page);
}

void select_driver() {
    menu_select(&driver_page);
}

void update_profile_page() { // todo this function is kinda jank
    // Update displayed driver name
    int driver_index = menu_list_get_selected(&driver_page);
    if (driver_index < 0) {
        return;
    }
    
    switch (driver_index) { 
        case 0: set_text(PROFILE_CURRENT_TXT, DRIVER1_NAME); break;
        case 1: set_text(PROFILE_CURRENT_TXT, DRIVER2_NAME); break;
        case 2: set_text(PROFILE_CURRENT_TXT, DRIVER3_NAME); break;
        case 3: set_text(PROFILE_CURRENT_TXT, DRIVER4_NAME); break;
    }
    
    profile_elements[0].current_value = driver_profiles[driver_index].brake_travel_threshold;
    profile_elements[1].current_value = driver_profiles[driver_index].throttle_travel_threshold;

    // Update display and styling
    menu_refresh_page(&profile_page);
}

void move_up_profile() {
    menu_move_up(&profile_page);
    
    // Update save status indicator
    if (!profile_page.is_element_selected) {
        set_font_color(PROFILE_STATUS_TXT, profile_page.saved ? GREEN : RED);
        set_text(PROFILE_STATUS_TXT, profile_page.saved ? "SAVED" : "UNSAVED");
    }
}

void move_down_profile() {
    menu_move_down(&profile_page);
    
    // Update save status indicator
    if (!profile_page.is_element_selected) {
        set_font_color(PROFILE_STATUS_TXT, profile_page.saved ? GREEN : RED);
        set_text(PROFILE_STATUS_TXT, profile_page.saved ? "SAVED" : "UNSAVED");
    }
}

void select_profile() {
    if (profile_page.current_index == 2) { // Save button
        int driver_index = menu_list_get_selected(&driver_page);
        // Save profile values
        driver_profiles[driver_index].brake_travel_threshold = profile_elements[0].current_value;
        driver_profiles[driver_index].throttle_travel_threshold = profile_elements[1].current_value;

        if (PROFILE_WRITE_SUCCESS != writeProfiles()) {
            profile_page.saved = false;
            set_font_color(PROFILE_STATUS_TXT, RED);
            set_text(PROFILE_STATUS_TXT, "FAILED");
        } else {
            profile_page.saved = true;
            set_font_color(PROFILE_STATUS_TXT, GREEN);
            set_text(PROFILE_STATUS_TXT, "SAVED");
        }
        return;
    }

    // Handle other elements using menu system
    menu_select(&profile_page);
    
    // Mark as unsaved when values change
    if (profile_page.is_element_selected) {
        profile_page.saved = false;
        set_font_color(PROFILE_STATUS_TXT, RED);
        set_text(PROFILE_STATUS_TXT, "UNSAVED");
    }
}

void update_cooling_page() {
    menu_refresh_page(&cooling_page);
    set_value(DT_FAN_BAR, cooling_elements[0].current_value);
    set_value(B_FAN_BAR, cooling_elements[2].current_value);
}

void move_up_cooling() {
    menu_move_up(&cooling_page);

    // Passively update the bar values
    if (cooling_page.is_element_selected) {
        set_value(DT_FAN_BAR, cooling_elements[0].current_value);
        set_value(B_FAN_BAR, cooling_elements[2].current_value);
    }
}

void move_down_cooling() {
    menu_move_down(&cooling_page);

    // Passively update the bar values
    if (cooling_page.is_element_selected) {
        set_value(DT_FAN_BAR, cooling_elements[0].current_value);
        set_value(B_FAN_BAR, cooling_elements[2].current_value);
    }
}

void select_cooling() {
    menu_select(&cooling_page);
}

// Updates the status of cooling elements from the PDU 
void coolant_out_CALLBACK(CanParsedData_t* msg_data_a) {
    cooling_elements[0].current_value = msg_data_a->coolant_out.dt_fan;
    cooling_elements[1].current_value = msg_data_a->coolant_out.dt_pump;
    cooling_elements[2].current_value = msg_data_a->coolant_out.bat_fan;
    cooling_elements[3].current_value = msg_data_a->coolant_out.bat_pump;

    if (curr_page != PAGE_COOLING) {
        return;
    }

    update_cooling_page();
}

void update_tv_page() {
    menu_refresh_page(&tv_page);
}

void move_up_tv() {
    menu_move_up(&tv_page);
}

void move_down_tv() {
    menu_move_down(&tv_page);
}

void select_tv() {
    menu_select(&tv_page);
    race_elements[0].current_value = tv_elements[3].current_value; // Sync TV settings
}

void update_faults_page() {
    if (fault_buf[0] == 0xFFFF) {
        set_text(FAULT1_TXT, FAULT_NONE_STRING);
    } else {
        set_text(FAULT1_TXT, faultArray[fault_buf[0]].screen_MSG);
    }

    if (fault_buf[1] == 0xFFFF) {
        set_text(FAULT2_TXT, FAULT_NONE_STRING);
    } else {
        set_text(FAULT2_TXT, faultArray[fault_buf[1]].screen_MSG);
    }

    if (fault_buf[2] == 0xFFFF) {
        set_text(FAULT3_TXT, FAULT_NONE_STRING);
    } else {
        set_text(FAULT3_TXT, faultArray[fault_buf[2]].screen_MSG);
    }

    if (fault_buf[3] == 0xFFFF) {
        set_text(FAULT4_TXT, FAULT_NONE_STRING);
    } else {
        set_text(FAULT4_TXT, faultArray[fault_buf[3]].screen_MSG);
    }

    if (fault_buf[4] == 0xFFFF) {
        set_text(FAULT5_TXT, FAULT_NONE_STRING);
    } else {
        set_text(FAULT5_TXT, faultArray[fault_buf[4]].screen_MSG);
    }

    menu_refresh_page(&faults_page);
}

void move_up_faults() {
    menu_move_up(&faults_page);
}

void move_down_faults() {
    menu_move_down(&faults_page);
}

void select_fault() {
    menu_select(&faults_page);
}

void clear_fault(int index) {
    if (index < 0 || index > 4) {
        return;
    }

    if (fault_buf[index] == 0xFFFF) {
        return;
    }

    if (checkFault(fault_buf[index])) {  // Check if fault is not latched
        return;
    }

    // Shift the elements to the left
    for (int i = index; i < 4; i++) {
        fault_buf[i] = fault_buf[i + 1];
    }
    fault_buf[4] = 0xFFFF;
}

void fault_button_callback() {
    int hover_index = faults_page.current_index;
    if (hover_index == 5) {
        for (int i = 4; i >= 0; i--) {
            clear_fault(i);
        }
    } else {
        clear_fault(hover_index);
    }
}

void update_race_page() {
    menu_refresh_page(&race_page);
}

void update_race_telemetry() {
    if (curr_page != PAGE_RACE) {
        return;
    }

    set_value(BRK_BAR, 0); // TODO BRK BAR
    set_value(THROT_BAR, (int) ((filtered_pedals / 4095.0) * 100));

    // update the speed
    if (can_data.rear_wheel_speeds.stale) {
        set_text(SPEED, "S");
    } else {
        // Vehicle Speed [m/s] = Wheel Speed [RPM] * 16 [in] * PI * 0.0254 / 60
        // set_text(SPEED, NXT_TEXT, int_to_char((uint16_t)((float)MAX(can_data.rear_wheel_speeds.left_speed_sensor, can_data.rear_wheel_speeds.right_speed_sensor) * 0.01 * 0.4474), parsed_value));
        uint16_t speed = ((float)can_data.gps_speed.gps_speed * 0.02237); // TODO macro this magic number
        set_textf(SPEED, "%d", speed);
    }

    // Update the voltage and current
    if (can_data.orion_currents_volts.stale) {
        set_text(BATT_VOLT, "S");
        set_text(BATT_CURR, "S");
    } else {
        uint16_t voltage = (can_data.orion_currents_volts.pack_voltage / 10);
        set_textf(BATT_VOLT, "%dV", voltage);

        uint16_t current = (can_data.orion_currents_volts.pack_current / 10);
        set_textf(BATT_CURR, "%dA", current);  // Note: Changed 'V' to 'A' for current
    }

    // Update the motor temperature
    if (can_data.rear_motor_temps.stale) {
        set_text(MOT_TEMP, "S");
    } else {
        uint8_t motor_temp = MAX(can_data.rear_motor_temps.left_mot_temp, can_data.rear_motor_temps.right_mot_temp);
        set_textf(MOT_TEMP, "%dC", motor_temp);
    }

    // TODO update motor controller temp

    // Update the battery temperature
    if (can_data.max_cell_temp.stale) {
        set_text(BATT_TEMP, "S");
    } else {
        uint16_t batt_temp = can_data.max_cell_temp.max_temp / 10;
        set_textf(BATT_TEMP, "%dC", batt_temp);
    }

    // Update the state of charge
    if (can_data.main_hb.stale) {
        set_text(CAR_STAT, "S");
        set_font_color(CAR_STAT, WHITE);
    } else {
        switch(can_data.main_hb.car_state) {
            case CAR_STATE_PRECHARGING:
                set_font_color(CAR_STAT, ORANGE);
                set_text(CAR_STAT, "PRECHARGE");
                break;
            case CAR_STATE_ENERGIZED:
                set_font_color(CAR_STAT, ORANGE);
                set_text(CAR_STAT, "ENERGIZED");
                break;
            case CAR_STATE_IDLE:
                set_font_color(CAR_STAT, INFO_GRAY);
                set_text(CAR_STAT, "IDLE");
                break;
            case CAR_STATE_READY2DRIVE:
                set_font_color(CAR_STAT, GREEN);
                set_text(CAR_STAT, "READY");
                break;
            case CAR_STATE_ERROR:
                set_font_color(CAR_STAT, YELLOW);
                set_text(CAR_STAT, "ERROR");
                break;
            case CAR_STATE_FATAL:
                set_font_color(CAR_STAT, RED);
                set_text(CAR_STAT, "FATAL");
                break;
        }
    }
}

void select_race() {
    menu_select(&race_page);
    tv_elements[3].current_value = race_elements[0].current_value; // Sync TV settings
}

void update_logging_page() {
    menu_refresh_page(&logging_page);

    if (logging_elements[0].current_value == 1) {
        set_text(LOGGING_STATUS_TXT, "DAQ ON");
        set_font_color(LOGGING_STATUS_TXT, GREEN);
    } else {
        set_text(LOGGING_STATUS_TXT, "DAQ OFF");
        set_font_color(LOGGING_STATUS_TXT, RED);
    }
}

void select_logging() {
    menu_select(&logging_page);

    if (logging_elements[0].current_value == 1) {
        set_text(LOGGING_STATUS_TXT, "DAQ ON");
        set_font_color(LOGGING_STATUS_TXT, GREEN);
    } else {
        set_text(LOGGING_STATUS_TXT, "DAQ OFF");
        set_font_color(LOGGING_STATUS_TXT, RED);
    }
}

void setFaultIndicator(uint16_t fault, char *element) {
    if (fault == 0xFFFF) {
        set_font_color(element, WHITE);
        return;
    }
    
    if (checkFault(fault)) {
        set_font_color(element, RED);
    } else {
        set_font_color(element, GREEN);
    }
}

void updateSDCStatus(uint8_t status, char *element) {
    if (status)
    {
        set_background(element, GREEN);
    }
    else
    {
        set_background(element, RED);
    }
}
