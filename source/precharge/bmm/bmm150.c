/**
 * @file bmi.c
 * @author Adam Busch (busch8@purdue.edu)
 * @brief
 * @version 0.1
 * @date 2022-02-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "bmm150.h"
#include "bsxlite_interface.h"
#include "common/phal_L4/spi/spi.h"
#include "common_defs.h"

static inline void BMI088_selectGyro(BMI088_Handle_t *bmi);
static inline void BMI088_selectAccel(BMI088_Handle_t *bmi);

bool BMM150_init(BMM150_Handle_t *bmm)
{
    bmm->mag_ready = false;

    /* Mag initilization  */
    BMM150_selectMag(bmm);
    if (PHAL_SPI_readByte(bmm->spi, BMM150_MAG_CHIP_ID_ADDR, true) != BMM150_MAG_CHIP_ID)
        return false;

    PHAL_SPI_writeByte(bmm->spi, BMM150_MAG_RANGE_ADDR, bmm->mag_datarate);

    // Perform self tests for sensor
    BMM150_magSelfTestStart(bmi);
    while (!BMM150_magSelfTestComplete(bmi))
        ;

    if (!BMI088_magSelfTestPass(bmi))
        return false;

    return true;
}

// unsure which one to use
int8_t bmm150_init(struct bmm150_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    /* Power up the sensor from suspend to sleep mode */
    rslt = set_power_control_bit(BMM150_POWER_CNTRL_ENABLE, dev);

    if (rslt == BMM150_OK)
    {
        /* Start-up time delay of 3ms */
        dev->delay_us(BMM150_START_UP_TIME, dev->intf_ptr);

        /* Chip ID of the sensor is read */
        rslt = bmm150_get_regs(BMM150_REG_CHIP_ID, &chip_id, 1, dev);

        /* Proceed if everything is fine until now */
        if (rslt == BMM150_OK)
        {
            /* Check for chip id validity */
            if (chip_id == BMM150_CHIP_ID)
            {
                dev->chip_id = chip_id;

                /* Function to update trim values */
                rslt = read_trim_registers(dev);
            }
        }
    }

    return rslt;
}

void BMI088_powerOnAccel(BMI088_Handle_t *bmi)
{
    BMI088_selectAccel(bmi);
    PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_PWR_CONF_ADDR, 0);
    PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_PWR_CTRL_ADDR, BMI088_ACC_PWR_CTRL_NORMAL);
    return;
}

bool BMI088_initAccel(BMI088_Handle_t *bmi)
{
    BMI088_selectAccel(bmi);

    /* Wait a long time befoer you call this function (50ms) */
    PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_PWR_CONF_ADDR, 0);
    PHAL_SPI_readByte(bmi->spi, BMI088_ACC_CHIP_ID_ADDR, false);

    PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_RANGE_ADDR, bmi->accel_range) &&
        PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_CONFIG_ADDR, (bmi->accel_bwp << 4) | bmi->accel_odr);

    uint8_t read_back = PHAL_SPI_readByte(bmi->spi, BMI088_ACC_CHIP_ID_ADDR, false);

    bmi->accel_ready = true;
    return true;
}

bool BMI088_gyroSelfTestStart(BMI088_Handle_t *bmi)
{
    BMI088_selectGyro(bmi);
    PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, 0x01U);
    return true;
}

bool BMI088_gyroSelfTestComplete(BMI088_Handle_t *bmi)
{
    BMI088_selectGyro(bmi);
    uint8_t self_test_res = PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, true);
    return (self_test_res & 0b10) == 0b10;
}

bool BMI088_gyroSelfTestPass(BMI088_Handle_t *bmi)
{
    BMI088_selectGyro(bmi);
    uint8_t test_result = PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, true);
    if (test_result & 0b10)
    {
        // Self test completed
        return (test_result & 0b10100) == 0b10000;
    }
    // Self test was not yet run
    return false;
}

bool BMI088_readAccel(BMI088_Handle_t *bmi, vector_3d_t *v)
{
    static uint8_t spi_rx_buff[16] = {0};
    static uint8_t spi_tx_buff[16] = {0};

    BMI088_selectAccel(bmi);
    while (PHAL_SPI_busy())
        ;

    spi_tx_buff[0] = (1 << 7) | BMI088_ACC_RATE_X_LSB_ADDR;
    PHAL_SPI_transfer(bmi->spi, spi_tx_buff, 8, spi_rx_buff);
    while (PHAL_SPI_busy())
        ;
    int16_t raw_ax, raw_ay, raw_az;
    raw_ax = (((int16_t)spi_rx_buff[2 + 1]) << 8) | spi_rx_buff[1 + 1];
    raw_ay = (((int16_t)spi_rx_buff[4 + 1]) << 8) | spi_rx_buff[3 + 1];
    raw_az = (((int16_t)spi_rx_buff[6 + 1]) << 8) | spi_rx_buff[5 + 1];

    // Conversion taken from datasheet pg 22.

    v->x = (float)(raw_ax << (bmi->accel_range + 1)) / 32768.0f * G_TO_M_S * 1.5f;
    v->y = (float)(raw_ay << (bmi->accel_range + 1)) / 32768.0f * G_TO_M_S * 1.5f;
    v->z = (float)(raw_az << (bmi->accel_range + 1)) / 32768.0f * G_TO_M_S * 1.5f;

    return true;
}

static inline void BMI088_selectMag(BMI088_Handle_t *bmi)
{
    bmm->spi->nss_gpio_port = bmm->mag_csb_gpio_port;
    bmi->spi->nss_gpio_pin = bmm->mag_csb_pin;
}