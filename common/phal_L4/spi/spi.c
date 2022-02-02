/**
 * @file spi.c
 * @author Adam Busch (busch8@purdue.edu)
 * @brief PER Serial Peripheral Interface Device driver for STM32L4
 * @version 0.1
 * @date 2022-01-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "common/phal_L4/spi/spi.h"


extern uint32_t APB2ClockRateHz;
static volatile SPI_InitConfig_t* active_transfer = NULL;

bool PHAL_SPI_init(SPI_InitConfig_t* cfg)
{
    // Enable RCC Clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Setup for Master, positive polarity
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
    SPI1->CR2 &= ~(SPI_CR2_NSSP | SPI_CR2_SSOE);

    // Data Size
    SPI1->CR2 &= ~(SPI_CR2_DS_Msk);
    SPI1->CR2 |= (CLAMP(cfg->data_len, 4, 16) - 1) << SPI_CR2_DS_Pos;

    // RX Fifo full on 8 bits
    SPI1->CR2 |= SPI_CR2_FRXTH; 
    // SPI1->CR1 |= SPI_CR1_LSBFIRST;

    // Data Rate
    // Divisor is a power of 2, find the closest power of 2 limited to log2(256)
    uint32_t f_div = LOG2_DOWN(APB2ClockRateHz / cfg->data_rate) - 1;
    f_div = CLAMP(f_div, 0, 0b111);
    SPI1->CR1 &= ~SPI_CR1_BR_Msk;
    SPI1->CR1 |= f_div << SPI_CR1_BR_Pos;

    // Setup DMA channels
    if (cfg->rx_dma_cfg && !PHAL_initDMA(cfg->rx_dma_cfg))
        return false;

    if (cfg->tx_dma_cfg && !PHAL_initDMA(cfg->tx_dma_cfg))
        return false;

    PHAL_writeGPIO(cfg->nss_gpio_bank, cfg->nss_gpio_pin, 1);
    
    cfg->_busy = false;
}

bool PHAL_SPI_transfer(SPI_InitConfig_t* spi, const uint8_t* out_data, const uint32_t data_len, const uint8_t* in_data)
{
    /*
    Each DMA channel is enabled if a data buffer is provided.

    Only the Tx DMA channel has transfer complete interrupts enabled, as the same data length is used for both channels
    */

    if (active_transfer || spi->_busy)
        return false;
    
    active_transfer = spi;
    
    if(spi->nss_sw)
        PHAL_writeGPIO(spi->nss_gpio_bank, spi->nss_gpio_pin, 0);

    spi->_busy = true;
    
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    PHAL_DMA_setTxferLength(spi->tx_dma_cfg, data_len);
    PHAL_DMA_setMemAddress(spi->rx_dma_cfg, (uint32_t) out_data);

    SPI1->CR2 |= SPI_CR2_RXDMAEN;
    PHAL_DMA_setTxferLength(spi->rx_dma_cfg, data_len);
    PHAL_DMA_setMemAddress(spi->rx_dma_cfg, (uint32_t) in_data);

    PHAL_startTxfer(spi->rx_dma_cfg);
    
    // Enable the DMA IRQ
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    
    // Start transaction
    SPI1->CR1 |= SPI_CR1_SPE;

    // STM32 HAL Libraries start TX Dma transaction last
    PHAL_startTxfer(spi->tx_dma_cfg);

    return true;
}

bool PHAL_SPI_busy()
{
    if (active_transfer)
        return active_transfer->_busy;
    return false;
}


void DMA1_Channel3_IRQHandler()
{
    bool is_tc_interrupt = DMA1->ISR & DMA_ISR_TCIF3;
    // Ack interrupt
    DMA1->IFCR |= DMA_IFCR_CTCIF3_Msk;
    if (is_tc_interrupt) 
    {
        
    }
}

/**
 * @brief SPI1 Rx DMA Transfer Complete interrupt
 * 
 */
void DMA1_Channel2_IRQHandler()
{
    bool is_tc_interrupt = DMA1->ISR & DMA_ISR_TCIF2;
    // Ack interrupt
    DMA1->IFCR |= DMA_IFCR_CTCIF2_Msk;

    if (is_tc_interrupt)
    {
        if (active_transfer->nss_sw)
            PHAL_writeGPIO(active_transfer->nss_gpio_bank, active_transfer->nss_gpio_pin, 1);

        // Disable DMA channels
        PHAL_stopTxfer(active_transfer->rx_dma_cfg);
        PHAL_stopTxfer(active_transfer->tx_dma_cfg);
        
        // Disable SPI peripheral and DMA requests
        SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
        SPI1->CR1 &= ~SPI_CR1_SPE;

        active_transfer->_busy = false;
        active_transfer = NULL;
    }
    
}
