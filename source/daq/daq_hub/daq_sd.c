
#include "common/phal_F4_F7/rtc/rtc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "daq_sd.h"
#include "daq_hub.h"
#include "ff.h"
#include "main.h"

extern ThreadWrapper threadWrapperName(sd_create_file_periodic);
extern ThreadWrapper threadWrapperName(sd_write_periodic);
extern ThreadWrapper threadWrapperName(ftp_server_run);

/**
 * @brief Returns if logging has been enabled.
 *        Sources for enabling are on-board switch
 *        and CAN message request from either
 *        dashboard or via wireless request.
 *        Logging will not be enabled unless
 *        the switch is flipped to on.
 *
 * @return Enabled
 */
bool get_log_enable(void)
{
    // TODO: combine with CAN message from dash
    // return dh.log_enable_sw || dh.log_enable_tcp;
    // TODO: switch doesnt work fix resistor value just return true for now
    //return dh.log_enable_uds;
    return dh.log_enable_sw;
}

static void sd_handle_error(sd_error_t err, FRESULT res)
{
    ++dh.sd_error_ct;
    dh.sd_last_err = err;
    dh.sd_last_err_res = res;
    dh.sd_last_error_time = getTick();
    PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 1);
    debug_printf("sd err: %d res: %d\n", err, res);
}

bool daq_request_sd_mount(void)
{
    return dh.sd_state == SD_STATE_MOUNTED || dh.sd_state == SD_STATE_FILE_CREATED;
}

static FRESULT sd_open_new_file(void)
{
    FRESULT ret;
    RTC_timestamp_t time;
    static uint32_t log_num;

    // Record creation time
    // TODO track handle periods
    dh.log_start_ms = getTick();
    dh.last_write_ms = getTick();

    char f_name[30];

    if (PHAL_getTimeRTC(&time))
    {
        // Create file name from RTC
        sprintf(f_name, "log-20%02d-%02d-%02d--%02d-%02d-%02d.log",
                time.date.year_bcd, time.date.month_bcd,
                time.date.day_bcd,  time.time.hours_bcd,
                time.time.minutes_bcd, time.time.seconds_bcd);
    }
    else
    {
        sprintf(f_name, "log-%04d.log", log_num);
    }
    ret = f_open(&dh.log_fp, f_name, FA_OPEN_APPEND | FA_READ | FA_WRITE);
    if (ret == FR_OK) {
        debug_printf("delta: %d: created file %02d: %s\n", getTick() - dh.last_file_tick, log_num, f_name);
        log_num++;
        dh.last_file_tick = getTick();
    }

    return ret;
}

void sd_create_file_periodic(void)
{
    FRESULT res;

    if (dh.sd_state == SD_FAIL || dh.sd_state == SD_SHUTDOWN) return; // try again

    if (dh.sd_state == SD_STATE_IDLE || dh.sd_state == SD_STATE_MOUNTED)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    else if (dh.sd_state == SD_STATE_FILE_CREATED)
    {
        mDelay(SD_NEW_FILE_PERIOD_MS); // create new file at regular intervals, but not for the first file
        if ((res = f_close(&dh.log_fp)) != FR_OK) sd_handle_error(SD_ERROR_FCLOSE, res);
    }

    res = sd_open_new_file();
    if (res == FR_OK)
    {
        dh.last_file_tick = getTick();
        dh.sd_state = SD_STATE_FILE_CREATED;
        dh.sd_last_err = SD_ERROR_NONE; // back to okay
        PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 0);
    }
    else
    {
        sd_handle_error(SD_ERROR_FOPEN, res);
    }

    xTaskNotify(getTaskHandle(sd_write_periodic), 0, eNoAction);
    // TODO close file on log disable
}

void sd_update_connection_state(void)
{
    FRESULT res;

#if 0
    // debounce
    // TODO determine hierarchy with TCP enable switch
    dh.log_enable_sw = PHAL_readGPIO(LOG_ENABLE_PORT, LOG_ENABLE_PIN);
    if (!dh.log_enable_sw)
    {
        PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 1);
    }
    else
    {
        PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 0);
    }
#endif

    switch (dh.sd_state)
    {
        case SD_STATE_IDLE:
            if (SD_Detect() == SD_PRESENT)
            {
                res = f_mount(&dh.fat_fs, "", 1);
                if (res == FR_OK)
                {
                    bActivateTail(&b_rx_can, RX_TAIL_SD);
                    dh.sd_state = SD_STATE_MOUNTED;
                    PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 1);
                    debug_printf("SD UP!\n");
                    xTaskNotify(getTaskHandle(sd_create_file_periodic), 0, eNoAction);
                    //xTaskNotify(getTaskHandle(ftp_server_run), (1<<0), eSetBits);  // set 0th bit HIGH
                }
                else
                {
                    sd_handle_error(SD_ERROR_MOUNT, res);
                    PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 0);
                }
            }
            else
            {
                PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 0);
            }
        break;
        case SD_STATE_MOUNTED:
        case SD_STATE_FILE_CREATED:
            break;
        case SD_FAIL:
            // Intentional fall-through
        case SD_SHUTDOWN:
        default:
            f_close(&dh.log_fp);   // Close file
            f_mount(0, "", 1);     // Unmount drive
            SD_DeInit();           // Shutdown SDIO peripheral
            dh.sd_state = SD_STATE_IDLE;
            bDeactivateTail(&b_rx_can, RX_TAIL_SD);
            PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 0);
        break;
    }
}

static inline void sd_file_sync(void)
{
    FRESULT res = f_sync(&dh.log_fp);
    if (res != FR_OK)
    {
        sd_handle_error(SD_ERROR_SYNC, res);
    }
}

void sd_write_periodic(void)
{
    timestamped_frame_t *buf;
    uint32_t consecutive_items;
    UINT bytes_written;
    FRESULT res;

    if (dh.sd_state != SD_STATE_FILE_CREATED)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    if (dh.sd_state == SD_STATE_FILE_CREATED)
    {
        // Use the total item count, not contiguous for the threshold
        if (bGetItemCount(&b_rx_can, RX_TAIL_SD) >= SD_MAX_WRITE_COUNT)
        {
            if ((bGetTailForRead(&b_rx_can, RX_TAIL_SD, (void**) &buf, &consecutive_items) == 0))
            {
                if (consecutive_items > SD_MAX_WRITE_COUNT) consecutive_items = SD_MAX_WRITE_COUNT; // limit
                // Write time :D
                PHAL_writeGPIO(SD_ACTIVITY_LED_PORT, SD_ACTIVITY_LED_PIN, 1);
                res = f_write(&dh.log_fp, buf, consecutive_items * sizeof(*buf), &bytes_written);
                if (res != FR_OK)
                {
                    sd_handle_error(SD_ERROR_WRITE, res);
                }
                else
                {
                    bCommitRead(&b_rx_can, RX_TAIL_SD, bytes_written / sizeof(*buf));
                    sd_file_sync(); // fsync takes only 4 ticks and ensures sure cache is flushed on close
                }
                PHAL_writeGPIO(SD_ACTIVITY_LED_PORT, SD_ACTIVITY_LED_PIN, 0);
            }
            else
            {
                daq_catch_error();
            }
        }
    }
}
