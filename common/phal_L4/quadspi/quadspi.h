#ifndef _PHAL_QUADSPI_H
#define _PHAL_QUADSPI_H

#include "stm32l4xx.h"
#include "common/phal_L4/dma/dma.h"
#include <stdbool.h>

#define QUADSPI_FIFO_SIZE_BYTES (16)


typedef enum {
    QUADSPI_INDIRECT_WRITE_MODE = 0b00,
    QUADSPI_INDIRECT_READ_MODE  = 0b01,
    QUADSPI_AUTOMATIC_POLL_MODE = 0b10,
} QUADSPI_FunctionMode_t;

typedef enum {
    QUADSPI_SKIP_SECTION= 0b00, /* Skip over this phase of transfer */
    QUADSPI_SINGLE_LINE = 0b01, /* Use single QSPI line for transfer */
    QUADSPI_DUAL_LINE   = 0b10, /* Use two QSPI lines for transfer */
    QUADSPI_QUAD_LINE   = 0b11, /* Use four QSPI lines for transfer */
} QUADSPI_LineWidth_t;

typedef enum {
    QUADSPI_8_BIT  = 0b00,
    QUADSPI_16_BIT = 0b01,
    QUADSPI_24_BIT = 0b10,
    QUADSPI_32_BIT = 0b11,
} QUADSPI_FieldSize_t;

typedef struct{
    QUADSPI_FunctionMode_t mode;            /* Functional mode selection */
    
    QUADSPI_LineWidth_t instruction_lines;  /* Number of SPI lines to use for instruction transfer */
    bool single_instruction;                /* Only send instructions once, might not be supported by device. */

    QUADSPI_LineWidth_t address_lines;      /* Number of SPI lines to use for address transfer */
    QUADSPI_FieldSize_t address_size;       /* Number of bytes to use for data transfer */
    
    QUADSPI_LineWidth_t alternate_lines;    /* Number of SPI lines to use for alternate data transfer */
    QUADSPI_FieldSize_t alternate_size;     /* Number of bytes to use for alternate data transfer */

    QUADSPI_LineWidth_t data_lines;         /* Number of SPI lines to use for data transfer */
    
    uint8_t dummy_cycles;                   /* Number of dummy cycles to insert after address */
    uint8_t fifo_threshold;                 /* Number of bytes to initiate the FIFO Threshold flag */

    dma_init_t* dma_cfg;

    bool _busy;                             /* DMA transaction busy flag */
} QUADSPI_Config_t;

/**
 * @brief Initilize quadspi peripheral clocks and speed to defaults
 * 
 * @return true  Successfuly initilized QUADSPI peripheral
 * @return false 
 */
bool PHAL_qspiInit();

/**
 * @brief Configure QUADSPI peripheral for SPI Flash transactions
 *        Will setup flash size, dummy cycles, instruction width, and data width.
 * 
 * @return true 
 * @return false 
 */
bool PHAL_qspiConfigure(QUADSPI_Config_t* config);

void PHAL_qspiSetFunctionMode(QUADSPI_FunctionMode_t new_mode);
void PHAL_qspiSetDataWidth(QUADSPI_LineWidth_t new_width);
void PHAL_qspiSetInstructionWidth(QUADSPI_LineWidth_t new_width);
void PHAL_qspiSetAddressWidth(QUADSPI_LineWidth_t new_width);
void PHAL_qspiSetAddressSize(QUADSPI_FieldSize_t new_size);

bool PHAL_qspiWrite(uint8_t instruction, uint32_t address, uint8_t* tx_data, uint32_t tx_length);
bool PHAL_qspiWrite_DMA(uint8_t instruction, uint32_t address, uint8_t* tx_data, uint16_t length);
bool PHAL_qspiRead(uint8_t instruction, uint32_t address, uint8_t* rx_data, uint32_t rx_length);

#define QUADSPI_DMA1_CONFIG(tx_addr_, priority_, read_memory_)        \
    {.periph_addr=(uint32_t) &(SPI1->DR), .mem_addr=(uint32_t) (tx_addr_),      \
     .tx_size=1, .increment=false, .circular=false,            \
     .dir=read_memory_, .mem_inc=true, .periph_inc=false, .mem_to_mem=false, \
     .priority=(priority_), .mem_size=0b00, .periph_size=0b00,        \
     .tx_isr_en=true, .dma_chan_request=0b0101, .channel_idx=5,    \
     .periph=DMA1, .channel=DMA1_Channel5, .request=DMA1_CSELR}

#endif