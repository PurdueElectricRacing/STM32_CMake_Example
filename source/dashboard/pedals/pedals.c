#include "pedals.h"
#include "common/phal_F4_F7/flash/flash.h"
#include "main.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "can_parse.h"
#include "lcd.h"
#include <stdint.h>

pedals_t pedals = {0};
uint16_t thtl_limit = 4096;

pedal_calibration_t pedal_calibration = {.t1max=1640,.t1min=1000, // WARNING: DAQ VARIABLE
                                         .t2max=2760,.t2min=2000, // IF EEPROM ENABLED,
                                         .b1max=1490,.b1min=450, // VALUE WILL CHANGE
                                         .b2max=1490,.b2min=450}; // 1400, 400

uint16_t t1_buff[10] = {0};
uint16_t t2_buff[10] = {0};
uint16_t b1_buff[10] = {0};
uint16_t b2_buff[10] = {0};
uint8_t t1_idx = 0;
uint8_t t2_idx = 0;
uint8_t b1_idx = 0;
uint8_t b2_idx = 0;

uint16_t filtered_pedals;

driver_pedal_profile_t driver_pedal_profiles[4] = {
    {0, 10,10,0},
    {1, 10,10,0},
    {2, 10,10,0},
    {3, 10,10,0}
};

extern q_handle_t q_tx_can;

void pedalsPeriodic(void)
{
    // Get current values (don't want them changing mid-calculation)
    uint16_t t1 = raw_adc_values.t1;
    uint16_t t2 = raw_adc_values.t2;
    uint16_t b1 = raw_adc_values.b1;
    uint16_t b2 = raw_adc_values.b2;

    // Brake bias
    float brake_bias = 0;
    if (b1 + b2)
    {
        brake_bias = ((float)b1 / (b1 + b2));
    }

    setFault(ID_APPS_WIRING_T1_FAULT, t1);
    setFault(ID_APPS_WIRING_T2_FAULT, t2);

    setFault(ID_BSE_FAULT, PHAL_readGPIO(BRK_FAIL_TAP_GPIO_Port, BRK_FAIL_TAP_Pin));

    float t1_volts = (VREF / 0xFFFU) * t1;
    float t2_volts = (VREF / 0XFFFU) * t2;

    t1 = (t1_volts * RESISTOR_T1) / (VREF - t1_volts);
    t2 = (t2_volts * RESISTOR_T2) / (VREF - t2_volts);

    // Scale values based on min and max
    t1 = CLAMP(t1, pedal_calibration.t1min, pedal_calibration.t1max);
    t2 = CLAMP(t2, pedal_calibration.t2min, pedal_calibration.t2max);
    b1 = CLAMP(b1, pedal_calibration.b1min, pedal_calibration.b1max);
    b2 = CLAMP(b2, pedal_calibration.b2min, pedal_calibration.b2max);

    t1 = (uint16_t) ((((uint32_t) (t1 - pedal_calibration.t1min)) * MAX_PEDAL_MEAS) /
                     (pedal_calibration.t1max - pedal_calibration.t1min));
    t2 = (uint16_t) ((((uint32_t) (t2 - pedal_calibration.t2min)) * MAX_PEDAL_MEAS) /
                     (pedal_calibration.t2max - pedal_calibration.t2min));
    b1 = (uint16_t) ((((uint32_t) (b1 - pedal_calibration.b1min)) * MAX_PEDAL_MEAS) /
                     (pedal_calibration.b1max - pedal_calibration.b1min));
    b2 = (uint16_t) ((((uint32_t) (b2 - pedal_calibration.b2min)) * MAX_PEDAL_MEAS) /
                     (pedal_calibration.b2max - pedal_calibration.b2min));

    // Both set at the same time
    if ((b1 >= APPS_BRAKE_THRESHOLD &&
        t1 >= APPS_THROTTLE_FAULT_THRESHOLD) || (checkFault(ID_APPS_BRAKE_FAULT) && t1 >= APPS_THROTTLE_CLEARFAULT_THRESHOLD))
    {
        // set warning fault and treq could be 0
        t2 = 0;
        t1 = 0;
        setFault(ID_APPS_BRAKE_FAULT, true);
    }
    else if (t1 <= APPS_THROTTLE_CLEARFAULT_THRESHOLD)
    {
        setFault(ID_APPS_BRAKE_FAULT, false);
    }

    filtered_pedals = t1;

    SEND_RAW_THROTTLE_BRAKE(raw_adc_values.t1,
                            raw_adc_values.t2, raw_adc_values.b1,
                            raw_adc_values.b2, 0);
    SEND_FILT_THROTTLE_BRAKE(t1, b1);
}

static const uint32_t* PROFILE_FLASH_START = (uint32_t*)ADDR_FLASH_SECTOR_3;
static volatile uint32_t* profile_current_address;

// TODO move to main
int writePedalProfiles() { // TODO switch to EEPROM
    profile_current_address = (volatile uint32_t*)PROFILE_FLASH_START;

    if (FLASH_OK != PHAL_flashErasePage(PROFILES_START_SECTOR)) { // !! Fix the crash here
        return PROFILE_WRITE_FAIL;
    }

    for (uint8_t i = 0; i < NUM_PROFILES; ++i) {
        if (FLASH_OK != PHAL_flashWriteU32((uint32_t)profile_current_address, 
                                         *(uint32_t*)&driver_pedal_profiles[i])) {
            return PROFILE_WRITE_FAIL;
        }
        profile_current_address++;
    }

    return PROFILE_WRITE_SUCCESS;
}

void readPedalProfiles() {
    uint32_t read_address = ADDR_FLASH_SECTOR_11;

    for (uint8_t i = 0; i < NUM_PROFILES; ++i) {
        uint32_t *data = (uint32_t *)&driver_pedal_profiles[i];
        *data = *((uint32_t *)read_address);

        read_address += sizeof(driver_pedal_profile_t);
    }
}