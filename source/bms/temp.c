#include "temp.h"

int checkTempMaster(uint8_t addr)
{
    uint8_t buff;

    // Write cfg
    PHAL_I2C_gen_start(addr, 1, PHAL_I2C_MODE_TX);
    PHAL_I2C_write(addr);
    PHAL_I2C_gen_stop();

    // Read cfg
    PHAL_I2C_gen_start(addr, 1, PHAL_I2C_MODE_TX);
    PHAL_I2C_read(&buff);
    PHAL_I2C_gen_stop();

    // If read data is bad, we're not master
    if (buff != addr)
    {
        return 0;
    }

    // Reset to pull temps next read
    PHAL_I2C_gen_start(addr, 1, PHAL_I2C_MODE_TX);
    PHAL_I2C_write(0);
    PHAL_I2C_gen_stop();

    bms.temp_master = 1;

    return 1;
}

void tempTask(void)
{
    uint8_t i;
    uint8_t buff[TEMP_MAX * 2];

    // Get values from device 1
    PHAL_I2C_gen_start(TEMP_ID1, bms.temp_count, PHAL_I2C_MODE_RX);
    PHAL_I2C_read_multi(buff, bms.temp_count);
    PHAL_I2C_gen_stop();

    for (i = 0; i < bms.temp_count; i++)
    {
        bms.cells.chan_temps_raw[i] = (buff[i * 2] << 8) | buff[(i * 2) + 1];
    }

    // Get values from device 2
    PHAL_I2C_gen_start(TEMP_ID2, bms.temp_count, PHAL_I2C_MODE_RX);
    PHAL_I2C_read_multi(buff, bms.temp_count);
    PHAL_I2C_gen_stop();

    for (i = 0; i < bms.temp_count; i++)
    {
        bms.cells.chan_temps_raw[i] = (buff[i * 2] << 8) | buff[(i * 2) + 1];
    }
}

void procTemps(void)
{
    uint8_t i, error_temp, error_dt;
    float   dt_new;
    float   ddt;
    float   voltage;
    double  temperature;

    static uint16_t temp_last[TEMP_MAX];
    static float    dt[TEMP_MAX];

    error_temp = error_dt = 0;

    for (i = 0; i < bms.temp_count; i++)
    {
        voltage = VOLTAGE_REF * ((float) bms.cells.chan_temps_raw[i]) / 0xFFFF;
        voltage = (voltage * THERM_RESIST) / (VOLTAGE_TOP - voltage);
        temperature = B_VALUE / log(voltage / R_INF_3977) - KELVIN_2_CELSIUS;

        bms.cells.chan_temps_conv[i] = temperature * 10;

        if (bms.cells.chan_temps_conv[i] >= TEMP_MAX_C)
        {
            error_temp = 1;
        }

        dt_new = (bms.cells.chan_temps_conv[i] - temp_last[i]) / 0.015f;
        temp_last[i] = bms.cells.chan_temps_conv[i];
        ddt = (dt_new - dt[i]) / 0.015f;

        if (dt_new > DT_CRIT)
        {
            error_temp = 1;
        }
    }

    if (error_temp)
    {
        bms.error |= (1U << 4);
    }
    else
    {
        bms.error &= ~(1U << 4);
    }
}