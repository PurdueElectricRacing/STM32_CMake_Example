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
#include "common/phal_L4/spi/spi.h"

static uint8_t spi_rx_buff[16] = {0}; 
static uint8_t spi_tx_buff[16] = {0};

static inline void BMI088_selectGyro(BMI088_Handle_t* bmi);
static inline void BMI088_selectAcel(BMI088_Handle_t* bmi);


bool BMI088_init(BMI088_Handle_t* bmi)
{
    BMI088_selectGyro(bmi);
    if (PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_CHIP_ID_ADDR, false) != BMI088_GYRO_CHIP_ID)
        return false;
    
    PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_BANDWIDTH_ADDR, bmi->gyro_datarate);
    PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_RANGE_ADDR, bmi->gyro_range);

    // Perform self tests for sensor
    BMI088_gyroSelfTestStart(bmi);
    while (!BMI088_gyroSelfTestComplete(bmi))
        ;
    
    return BMI088_gyroSelfTestPass(bmi);
}

bool BMI088_gyroSelfTestStart(BMI088_Handle_t* bmi)
{
    BMI088_selectGyro(bmi);
    PHAL_SPI_writeByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, 0x01U);
}

bool BMI088_gyroSelfTestComplete(BMI088_Handle_t* bmi)
{
    BMI088_selectGyro(bmi);
    uint8_t self_test_res = PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, false);
    return (self_test_res & 0b10) == 0b10;
}

bool BMI088_gyroSelfTestPass(BMI088_Handle_t* bmi)
{
    BMI088_selectGyro(bmi);
    uint8_t test_result = PHAL_SPI_readByte(bmi->spi, BMI088_GYRO_SELFTEST_ADDR, false);
    if (test_result & 0b10)
    {
        // Self test completed
        return (test_result & 0b10100) == 0b10000;
    }
    // Self test was not yet run
    return false;
}

bool BMI088_readGyro(BMI088_Handle_t* bmi, int16_t* x, int16_t* y, int16_t* z)
{
    BMI088_selectGyro(bmi);
    while (PHAL_SPI_busy())
        ;
    
    spi_tx_buff[0] = (1 << 7) | BMI088_GYRO_RATE_X_LSB_ADDR;
    PHAL_SPI_transfer(bmi->spi, spi_tx_buff, 7, spi_rx_buff);
    while (PHAL_SPI_busy())
        ;
    int16_t raw_x, raw_y, raw_z;
    raw_x =  (((int16_t) spi_rx_buff[2]) << 8) | spi_rx_buff[1];
    raw_y =  (((int16_t) spi_rx_buff[4]) << 8) | spi_rx_buff[3];
    raw_z =  (((int16_t) spi_rx_buff[6]) << 8) | spi_rx_buff[5];

    // Convert raw values into physical values based on range
    // Decimal is fixed in the first place
    float scale = 0.0;
    switch(bmi->gyro_range)
    {
        case (GYRO_RANGE_2000):
            scale = (16.384 / 10);
            break;
        case (GYRO_RANGE_1000):
            scale = (32.768 / 10);
            break;
        case (GYRO_RANGE_500):
            scale = (65.536 / 10);
            break;
        case (GYRO_RANGE_250):
            scale = (131.072 / 10);
            break;
        case (GYRO_RANGE_125):
            scale = (262.144 / 10);
            break;
    }

    *x = (int16_t) (raw_x / scale);
    *y = (int16_t) (raw_y / scale);
    *z = (int16_t) (raw_z / scale);
    return true;
}

static inline void BMI088_selectGyro(BMI088_Handle_t* bmi)
{
    bmi->spi->nss_gpio_port = bmi->gyro_csb_gpio_port;
    bmi->spi->nss_gpio_pin = bmi->gyro_csb_pin;
}

static inline void BMI088_selectAcel(BMI088_Handle_t* bmi)
{
    bmi->spi->nss_gpio_port = bmi->acel_csb_gpio_port;
    bmi->spi->nss_gpio_pin = bmi->acel_csb_pin;
}