
#if 0
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rtc/rtc.h"
#include "common/uds/uds.h"

#include "main.h"
#include "daq_hub.h"
#include "daq_sd.h"

#define DAQ_BL_CMD_NTP_DATE   0x30
#define DAQ_BL_CMD_NTP_TIME   0x31
#define DAQ_BL_CMD_NTP_GET    0x32

#define DAQ_BL_CMD_HANDSHAKE  0x40
#define DAQ_BL_CMD_LOG_ENABLE 0x41
#define DAQ_BL_CMD_LOG_STATUS 0x42
#define DAQ_BL_CMD_LED_DISCO  0x43

#define DAQ_BL_CMD_LIST_DIR   0x50

static RTC_timestamp_t start_time =
{
    .date = {.month_bcd=RTC_MONTH_FEBRUARY,
             .weekday=RTC_WEEKDAY_TUESDAY,
             .day_bcd=0x27,
             .year_bcd=0x24},
    .time = {.hours_bcd=0x18,
             .minutes_bcd=0x27,
             .seconds_bcd=0x00,
             .time_format=RTC_FORMAT_24_HOUR},
};

static void uds_process_cmd_ntp_get(void)
{
    RTC_timestamp_t time;
    if (PHAL_getTimeRTC(&time))
        debug_printf("DAQ time: 20%02d-%02d-%02d %02d:%02d:%02d\n",
            time.date.year_bcd, time.date.month_bcd,
            time.date.day_bcd,  time.time.hours_bcd,
            time.time.minutes_bcd, time.time.seconds_bcd);
}

/**
 * Process DAQ-specific UDS commands
 */
void uds_handle_sub_command_callback(uint8_t cmd, uint64_t data)
{
    switch (cmd)
    {
        case UDS_CMD_SYS_RST:
            daq_shutdown_hook(); // NVIC reset / bootloader if loaded (override sys)
            NVIC_SystemReset(); // Don't want to wait for power off if entered for bootloader mode
            break;

        case DAQ_BL_CMD_HANDSHAKE:
            PHAL_toggleGPIO(ERROR_LED_PORT, ERROR_LED_PIN);
            break;

        case DAQ_BL_CMD_NTP_DATE: // send date first
            start_time.date.day_bcd = data & 0xff;
            start_time.date.weekday = (data >> 8) & 0xf;
            start_time.date.month_bcd = (data >> 12) & 0xff;
            start_time.date.year_bcd = (data >> 20) & 0xff;
            break;
        case DAQ_BL_CMD_NTP_TIME: // then time
            start_time.time.seconds_bcd = data & 0xff;
            start_time.time.minutes_bcd = (data >> 8) & 0xff;
            start_time.time.hours_bcd = (data >> 16) & 0xff;
            PHAL_configureRTC(&start_time, true); // now sync
            break;
        case DAQ_BL_CMD_NTP_GET:
            uds_process_cmd_ntp_get();
            break;

        case DAQ_BL_CMD_LOG_ENABLE:
            dh.log_enable_uds = !!data;
            break;
        case DAQ_BL_CMD_LOG_STATUS:
            if (get_log_enable())
                debug_printf("Logging enabled\n");
            else
                debug_printf("Logging disabled\n");
            break;

        case DAQ_BL_CMD_LED_DISCO:
            //PHAL_writeGPIO(GPIO1_PORT, GPIO1_PIN, (frame.data >> 0) & 1);
            //PHAL_writeGPIO(GPIO2_PORT, GPIO2_PIN, (frame.data >> 1) & 1);
            PHAL_writeGPIO(ERROR_LED_PORT, ERROR_LED_PIN, (data >> 2) & 1);
            PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, (data >> 3) & 1);
            PHAL_writeGPIO(SD_ACTIVITY_LED_PORT, SD_ACTIVITY_LED_PIN, (data >> 4) & 1);
            PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, (data >> 5) & 1);
            break;
    }
}
#endif
