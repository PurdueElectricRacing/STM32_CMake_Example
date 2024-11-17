

#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/can/can.h"

// DAQ uart test F4
#include "common/log/log.h"
#include <string.h>
#include "f4_testing_common.h"
#include "source/daq/gs_usb.h"
#include "common/queue/queue.h"
#include "pds.h"
#include "source/daq/buffer/buffer.h"

#ifdef F4_TESTING_DAQ_DISCO_PDS_CAN // Main MCU 24 CAN PDS testing

// CAN Receive Buffer Configuration
#define RX_BUFF_ITEM_COUNT 2000

#if 0
// USART6
dma_init_t usart_tx_dma_config = USART6_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART6_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t lte_usart_config = {
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART6,
   .wake_addr = false,
   .usart_active_num = USART6_ACTIVE_IDX,
   .tx_dma_cfg = &usart_tx_dma_config, // &usart_tx_dma_config
   .rx_dma_cfg = &usart_rx_dma_config, // usart_rx_dma_config
};
#else
dma_init_t usart_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t lte_usart_config = {
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART2,
   .wake_addr = false,
   .usart_active_num = USART2_ACTIVE_IDX,
   .tx_dma_cfg = &usart_tx_dma_config, // &usart_tx_dma_config
   .rx_dma_cfg = &usart_rx_dma_config, // usart_rx_dma_config
};
#endif

GPIOInitConfig_t gpio_config[] = {
    //GPIO_INIT_USART_TX(GPIOA, 2),
    //GPIO_INIT_USART_RX(GPIOA, 3),
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12,
    GPIO_INIT_OUTPUT(GPIOD, 4, GPIO_OUTPUT_LOW_SPEED),
};

#if 0
#define TargetCoreClockrateHz 96000000
ClockRateConfig_t clock_config = {
    .system_source              = SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    = PLL_SRC_HSI16,
    .vco_output_rate_target_hz  = 192000000, //288000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 1),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 1),
};
#else
#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =16000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};
#endif

static inline int _iodev_write(usart_init_t* handle, char *buffer, int size)
{
    PHAL_usartTxBl(&lte_usart_config, (uint8_t *)buffer, size);
    //PHAL_usartTxDma(&lte_usart_config, (uint16_t *)buffer, size);
    return 0;
}

static inline int _iodev_putchar(usart_init_t* handle, uint8_t c)
{
    return _iodev_write(&lte_usart_config, (uint8_t *)&c, sizeof(uint8_t));
}

static inline int _iodev_printf(usart_init_t* handle, const char *fmt, ...)
{
    va_list args;
    char buffer[512];
    int i;

    va_start(args, fmt);
    i = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    _iodev_write(handle, buffer, i);

    return i;
}

typedef struct __attribute__((packed))
{
    uint32_t tick;      //!< ms timestamp of reception
    uint32_t type;
    uint32_t size;
    uint32_t data;
} daq_uart_frame_t;

#define STARTCODE 0xDEADBEEF

static void uart_frame_handler(void);

static void send_frame(uint32_t data)
{
    uint32_t startcode = STARTCODE;
    daq_uart_frame_t s;
    memset(&s, 0, sizeof(s));
    s.tick = tick_ms;
    s.type = 0x10000000 | 1;
    s.size = 8;
    s.data = data;
    PHAL_usartTxBl(&lte_usart_config, (uint8_t *)&startcode, sizeof(startcode));
    PHAL_usartTxBl(&lte_usart_config, (uint8_t *)&s, sizeof(s));
}

static bool receive_frame_start(void)
{
    uint8_t b = 0;

    PHAL_usartRxBl(&lte_usart_config, &b, 1);
    if (b == (STARTCODE & 0xff))
    {
        PHAL_usartRxBl(&lte_usart_config, &b, 1);
        if (b == ((STARTCODE & (0xff << 8)) >> 8))
        {
            PHAL_usartRxBl(&lte_usart_config, &b, 1);
            if (b == ((STARTCODE & (0xff << 16)) >> 16))
            {
                PHAL_usartRxBl(&lte_usart_config, &b, 1);
                if (b == ((STARTCODE & (0xff << 24)) >> 24))
                {
                    return true;
                }
            }
        }
    }

    return false;
}

static void uart_frame_handler(void)
{
    if (receive_frame_start())
    {
        PHAL_toggleGPIO(GPIOD, 13);
        daq_uart_frame_t frame;
        PHAL_usartRxBl(&lte_usart_config, (uint8_t *)&frame, sizeof(frame));
        switch (frame.type)
        {
            case 0x1: // heartbeat
                break;
            case 0x2: // helloworld
                _iodev_printf(&lte_usart_config, "Hello World\n");
                break;
        }
        send_frame(frame.data + 1); // ack
    }
}

typedef enum {
    RX_TAIL_CAN_RX, //!< CAN rx message parsing
    RX_TAIL_SD,     //!< SD Card
    RX_TAIL_UDP,    //!< UDP Broadcast
    RX_TAIL_USB,    //!< USB Send
    RX_TAIL_COUNT,
} rx_tail_t;

typedef enum {
    TCP_RX_TAIL_CAN_TX,
    TCP_RX_TAIL_SD,
    TCP_RX_TAIL_COUNT,
} tcp_rx_tail_t;

volatile timestamped_frame_t rx_buffer[RX_BUFF_ITEM_COUNT];
b_tail_t tails[RX_TAIL_COUNT];
b_handle_t b_rx_can = {
    .buffer=(volatile uint8_t *)rx_buffer,
    .tails=tails,
    .num_tails=RX_TAIL_COUNT,
};

int daq_disco_pds_can_main()
{
    bConstruct(&b_rx_can, sizeof(*rx_buffer), sizeof(rx_buffer));

    if(0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    #if 0
    if(!PHAL_initUSART(&lte_usart_config, APB2ClockRateHz))
    {
        HardFault_Handler();
    }
    _iodev_printf(&lte_usart_config, "%s: UART initialized\n", "DAQ");
    #endif
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    if(!PHAL_initCAN(CAN1, false, VCAN_BPS))
        HardFault_Handler();

    while (1)
    {
        ;
    }

    return 0;
}

static void can_rx_irq_handler(CAN_TypeDef * can_h)
{
    // TODO: track FIFO overrun and full errors
    if (can_h->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        can_h->RF0R &= !(CAN_RF0R_FOVR0);

    if (can_h->RF0R & CAN_RF0R_FULL0) // FIFO Full
        can_h->RF0R &= !(CAN_RF0R_FULL0);

    if (can_h->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        timestamped_frame_t *rx;
        uint32_t cont;
        if (bGetHeadForWrite(&b_rx_can, (void**) &rx, &cont) == 0)
        {
            rx->tick_ms = tick_ms;

            rx->bus_id = (can_h == CAN1) ? BUS_ID_CAN1 : BUS_ID_CAN2;

            // Get either StdId or ExtId
            if (CAN_RI0R_IDE & can_h->sFIFOMailBox[0].RIR)
            {
                rx->msg_id = CAN_EFF_FLAG | (((CAN_RI0R_EXID | CAN_RI0R_STID) & can_h->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos);
            }
            else
            {
                rx->msg_id = (CAN_RI0R_STID & can_h->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;
            }

            rx->dlc = (CAN_RDT0R_DLC & can_h->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

            rx->data[0] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 0)  & 0xFF;
            rx->data[1] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 8)  & 0xFF;
            rx->data[2] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
            rx->data[3] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
            rx->data[4] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 0)  & 0xFF;
            rx->data[5] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 8)  & 0xFF;
            rx->data[6] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
            rx->data[7] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

            // Bootloader check
            if (rx->msg_id == (ID_DAQ_BL_CMD | CAN_EFF_FLAG) && rx->bus_id == BUS_ID_CAN1)
            {
                CanParsedData_t* msg_data_a = (CanParsedData_t *) &rx->data;
                if (msg_data_a->daq_bl_cmd.cmd == BLCMD_RST)
                {
                    // TODO: stop logging first
                    Bootloader_ResetForFirmwareDownload();
                }
            }

            bCommitWrite(&b_rx_can, 1);
        }
        can_h->RF0R |= (CAN_RF0R_RFOM0);
    }
}

void CAN1_RX0_IRQHandler()
{
    can_rx_irq_handler(CAN1);
}

//volatile uint32_t last_err_stat = 0;
volatile uint32_t error_irq_cnt = 0;
void CAN1_SCE_IRQHandler()
{
    uint32_t err_stat;
    error_irq_cnt++;
    if (CAN1->MSR & CAN_MSR_ERRI)
    {
        err_stat = CAN1->ESR;
        CAN1->ESR &= ~(CAN_ESR_LEC_Msk);

        timestamped_frame_t *rx;
        uint32_t cont;
        if (bGetHeadForWrite(&b_rx_can, (void**) &rx, &cont) == 0)
        {
            rx->tick_ms = tick_ms;
            can_parse_error_status(err_stat, rx);
            bCommitWrite(&b_rx_can, 1);
        }
        CAN1->MSR |= CAN_MSR_ERRI; // clear interrupt
    }
}

bool can_parse_error_status(uint32_t err, timestamped_frame_t *frame)
{
	//frame->echo_id = 0xFFFFFFFF;
    frame->bus_id = 0;
	frame->msg_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
	frame->dlc = CAN_ERR_DLC;
	frame->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->data[5] = 0;
	frame->data[6] = 0;
	frame->data[7] = 0;

	if ((err & CAN_ESR_BOFF) != 0) {
		frame->msg_id |= CAN_ERR_BUSOFF;
	}

	/*
	uint8_t tx_error_cnt = (err>>16) & 0xFF;
	uint8_t rx_error_cnt = (err>>24) & 0xFF;
	*/

	if (err & CAN_ESR_EPVF) {
		frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
	} else if (err & CAN_ESR_EWGF) {
		frame->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
	}

	uint8_t lec = (err>>4) & 0x07;
	if (lec!=0) { /* protocol error */
		switch (lec) {
			case 0x01: /* stuff error */
				frame->msg_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_STUFF;
				break;
			case 0x02: /* form error */
				frame->msg_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_FORM;
				break;
			case 0x03: /* ack error */
				frame->msg_id |= CAN_ERR_ACK;
				break;
			case 0x04: /* bit recessive error */
				frame->msg_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_BIT1;
				break;
			case 0x05: /* bit dominant error */
				frame->msg_id |= CAN_ERR_PROT;
				frame->data[2] |= CAN_ERR_PROT_BIT0;
				break;
			case 0x06: /* CRC error */
				frame->msg_id |= CAN_ERR_PROT;
				frame->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
				break;
			default:
				break;
		}
	}

	return true;
}

#endif // F4_TESTING_DAQ_DISCO_PDS_CAN
