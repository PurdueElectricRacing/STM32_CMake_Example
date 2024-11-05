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

#include "bmi088.h"
#include "bsxlite_interface.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common_defs.h"
#include "main.h"
#include "common/psched/psched.h"


static inline void BMI088_selectGyro(BMI088_Handle_t *bmi);
static inline void BMI088_selectAccel(BMI088_Handle_t *bmi);

bool BMI088_init(BMI088_Handle_t *bmi)
{
    bmi->accel_ready = false;
    /* Gyro initilization  */
    BMI088_selectGyro(bmi);
    if (PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_CHIP_ID_ADDR, true) != BMI088_GYRO_CHIP_ID)
        return false;

    PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_BANDWIDTH_ADDR, bmi->gyro_datarate);
    PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_RANGE_ADDR, bmi->gyro_range);
    PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);

    // Perform self tests for sensor
    BMI088_gyroSelfTestStart(bmi);
    while (!BMI088_gyroSelfTestComplete(bmi))
        ;

    if (!BMI088_gyroSelfTestPass(bmi))
        return false;

    return true;
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

    /* Wait a long time before you call this function (50ms) */
    PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_PWR_CONF_ADDR, 0);
    PHAL_SPI_readByte(bmi->spi, BMI088_ACC_CHIP_ID_ADDR, false);

    PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_RANGE_ADDR, bmi->accel_range) &&
        PHAL_SPI_writeByte(bmi->spi, BMI088_ACC_CONFIG_ADDR, (bmi->accel_bwp << 4) | bmi->accel_odr);

    uint8_t read_back = PHAL_SPI_readByte(bmi->spi, BMI088_ACC_CHIP_ID_ADDR, false);

    bmi->accel_ready = true;
    return true;
}

bool BMI088_gyroOK(BMI088_Handle_t *bmi)
{
    BMI088_gyroSelfTestStart(bmi);
    while (!BMI088_gyroSelfTestComplete(bmi))
        ;

    if (!BMI088_gyroSelfTestPass(bmi))
        return false;

    return true;
}

bool BMI088_gyroSelfTestStart(BMI088_Handle_t *bmi)
{
    BMI088_selectGyro(bmi);
    PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, 0x01U);
    PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);
    return true;
}

bool BMI088_gyroSelfTestComplete(BMI088_Handle_t *bmi)
{
    BMI088_selectGyro(bmi);
    uint8_t self_test_res = PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, true);
    PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);
    return (self_test_res & 0b10) == 0b10;
}

bool BMI088_gyroSelfTestPass(BMI088_Handle_t *bmi)
{
    BMI088_selectGyro(bmi);
    uint8_t test_result = PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, true);
    if (test_result & 0b10)
    {
        // Self test completed
        PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);
        return (test_result & 0b10100) == 0b10000;
    }
    // Self test was not yet run
    PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);
    return false;
}

bool BMI088_readGyro(BMI088_Handle_t *bmi, vector_3d_t *v)
{
    static uint8_t spi_rx_buff[16] = {0};
    static uint8_t spi_tx_buff[16] = {0};

    BMI088_selectGyro(bmi);
    while (PHAL_SPI_busy(bmi->spi))
        ;

    spi_tx_buff[0] = (1 << 7) | BMI088_GYRO_RATE_X_LSB_ADDR;
    PHAL_SPI_transfer_noDMA(bmi->spi, spi_tx_buff, 1, 7, spi_rx_buff);
    // while (PHAL_SPI_busy(bmi->spi))
    //     ;
    int16_t raw_x, raw_y, raw_z;
    raw_x = (((int16_t)spi_rx_buff[2]) << 8) | spi_rx_buff[1];
    raw_y = (((int16_t)spi_rx_buff[4]) << 8) | spi_rx_buff[3];
    raw_z = (((int16_t)spi_rx_buff[6]) << 8) | spi_rx_buff[5];

    int16_t max_raw = MAX(MAX(raw_x, raw_y), raw_z);
    bool range_up = bmi->gyro_dynamic_range && (ABS(max_raw) >= 32000); // int16_t range is -32,768 to +32,767
    bool range_down = bmi->gyro_dynamic_range && (ABS(max_raw) <= 1000);

    // Convert raw values into physical values based on range
    // Decimal is fixed in the first place
    float scale = 0.0;
    switch (bmi->gyro_range)
    {
    case (GYRO_RANGE_2000):
        scale = (16.384 / DEG_TO_RAD);
        break;
    case (GYRO_RANGE_1000):
        scale = (32.768 / DEG_TO_RAD);
        break;
    case (GYRO_RANGE_500):
        scale = (65.536 / DEG_TO_RAD);
        if (range_down)
        {
            bmi->gyro_range = GYRO_RANGE_250;
            PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_RANGE_ADDR, bmi->gyro_range);
        }
        break;
    case (GYRO_RANGE_250):
        scale = (131.072 / DEG_TO_RAD);

        if (range_up)
        {
            bmi->gyro_range = GYRO_RANGE_500;
            PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_RANGE_ADDR, bmi->gyro_range);
        }
        else if (range_down)
        {
            bmi->gyro_range = GYRO_RANGE_125;
            PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_RANGE_ADDR, bmi->gyro_range);
        }
        break;
    case (GYRO_RANGE_125):
        scale = (262.144 / DEG_TO_RAD);

        if (range_up)
        {
            bmi->gyro_range = GYRO_RANGE_250;
            PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_RANGE_ADDR, bmi->gyro_range);
        }
        break;
    default:
        scale = 1.0; // prevent div by zero
    }

    v->x = raw_x / scale;
    v->y = raw_y / scale;
    v->z = raw_z / scale;
    PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);
    return true;
}

bool BMI088_readAccel(BMI088_Handle_t *bmi, vector_3d_t *v)
{
    static uint8_t spi_rx_buff[16] = {0};
    static uint8_t spi_tx_buff[16] = {0};

    BMI088_selectAccel(bmi);
    while (PHAL_SPI_busy(bmi->spi))
        ;

    spi_tx_buff[0] = (1 << 7) | BMI088_ACC_RATE_X_LSB_ADDR;
    PHAL_SPI_transfer_noDMA(bmi->spi, spi_tx_buff, 1, 7, spi_rx_buff);
    // while (PHAL_SPI_busy(bmi->spi))
    //     ;
    int16_t raw_ax, raw_ay, raw_az;
    raw_ax = (((int16_t)spi_rx_buff[2 + 1]) << 8) | spi_rx_buff[1 + 1];
    raw_ay = (((int16_t)spi_rx_buff[4 + 1]) << 8) | spi_rx_buff[3 + 1];
    raw_az = (((int16_t)spi_rx_buff[6 + 1]) << 8) | spi_rx_buff[5 + 1];

    // Conversion taken from datasheet pg 22.

    // NOTE - changed x and y to correspond to data sheet
    v->x = (float)(raw_ax << (bmi->accel_range + 1)) / 32768.0f * G_TO_M_S * 1.5f;
    v->y = (float)(raw_ay << (bmi->accel_range + 1)) / 32768.0f * G_TO_M_S * 1.5f;
    v->z = (float)(raw_az << (bmi->accel_range + 1)) / 32768.0f * G_TO_M_S * 1.5f;
    // asm("nop");
    PHAL_writeGPIO(SPI1_CSB_ACCEL_PORT, SPI1_CSB_ACCEL_PIN, 1);
    return true;
}

static inline void BMI088_selectGyro(BMI088_Handle_t *bmi)
{
    PHAL_writeGPIO(SPI1_CSB_ACCEL_PORT, SPI1_CSB_ACCEL_PIN, 1);
    bmi->spi->nss_gpio_port = bmi->gyro_csb_gpio_port;
    bmi->spi->nss_gpio_pin = bmi->gyro_csb_pin;
}

static inline void BMI088_selectAccel(BMI088_Handle_t *bmi)
{
    PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);
    bmi->spi->nss_gpio_port = bmi->accel_csb_gpio_port;
    bmi->spi->nss_gpio_pin = bmi->accel_csb_pin;
}

uint8_t BMI088_checkGyroHealth(BMI088_Handle_t *bmi)
{
    uint8_t self_test_register_result = PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, true);
    // self_test_register_result &= 0b00010000;
    return (self_test_register_result == 0b00010000);
}