#include "common/bootloader/bootloader_common.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/common_defs/common_defs.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/rtc/rtc.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/usart/usart.h"

#include "buffer.h"
#include "main.h"
#include "sdio.h"
#include "daq_hub.h"
#include "can_parse.h"
#include "common/modules/Wiznet/W5500/Ethernet/wizchip_conf.h"
#include "common/modules/Wiznet/W5500/Ethernet/socket.h"

#include "ff.h"
#include "gs_usb.h"

GPIOInitConfig_t gpio_config[] = {
    // LEDs
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONNECTION_LED_PORT, CONNECTION_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_ACTIVITY_LED_PORT, SD_ACTIVITY_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, GPIO_OUTPUT_LOW_SPEED),

    // W5500 ETH SPI1
    GPIO_INIT_AF(ETH_SCK_PORT, ETH_SCK_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(ETH_MISO_PORT, ETH_MISO_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_AF(ETH_MOSI_PORT, ETH_MOSI_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_OUTPUT(ETH_CS_PORT, ETH_CS_PIN, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT_OPEN_DRAIN(ETH_RST_PORT, ETH_RST_PIN, GPIO_OUTPUT_LOW_SPEED),

    // SDIO
    GPIO_INIT_SDIO_CLK,
    GPIO_INIT_SDIO_CMD,
    GPIO_INIT_SDIO_DT0,
    GPIO_INIT_SDIO_DT1,
    GPIO_INIT_SDIO_DT2,
    GPIO_INIT_SDIO_DT3,

    GPIO_INIT_INPUT(SD_CD_PORT, SD_CD_PIN, GPIO_INPUT_PULL_UP),
    GPIO_INIT_INPUT(LOG_ENABLE_PORT, LOG_ENABLE_PIN, GPIO_INPUT_PULL_UP),
    GPIO_INIT_INPUT(PWR_LOSS_PORT, PWR_LOSS_PIN, GPIO_INPUT_OPEN_DRAIN),

    // LTE UART
    GPIO_INIT_USART6TX_PC6,
    GPIO_INIT_USART6RX_PC7,

#ifdef DISCO_BOARD
    GPIO_INIT_CANRX_PD0,
    GPIO_INIT_CANTX_PD1,
#else
    GPIO_INIT_CANRX_PA11, // VCAN
    GPIO_INIT_CANTX_PA12,
#endif
#ifdef EN_CAN2
    GPIO_INIT_CAN2RX_PB5, // MCAN
    GPIO_INIT_CAN2TX_PB6,
#endif
};

/* SPI CONFIG FOR ETHERNET MODULE */
dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);
SPI_InitConfig_t eth_spi_config = {
    .data_len  = 8,
    .nss_sw = false,
    .nss_gpio_port = ETH_CS_PORT,
    .nss_gpio_pin = ETH_CS_PIN,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI1,
};

RTC_timestamp_t start_time =
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

/* CLOCK CONFIG */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

#if 1
#define TargetCoreClockrateHz 168000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  = 336000000,//288000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
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

volatile uint32_t tick_ms; // Systick 1ms counter

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
   .tx_dma_cfg = &usart_tx_dma_config,
   .rx_dma_cfg = &usart_rx_dma_config
};
DEBUG_PRINTF_USART_DEFINE(&lte_usart_config) // use LTE uart lmao

static void configure_exti(void);
static void cs_sel(void);
static void cs_desel(void);
static uint8_t spi_rb(void);
static void spi_wb(uint8_t b);
static void spi_rb_burst(uint8_t *pBuf, uint16_t len);
static void spi_wb_burst(uint8_t *pBuf, uint16_t len);
bool can_parse_error_status(uint32_t err, timestamped_frame_t *frame);

q_handle_t q_tx_can2_to_can1;

volatile timestamped_frame_t rx_buffer[RX_BUFF_ITEM_COUNT];
b_tail_t tails[RX_TAIL_COUNT];
b_handle_t b_rx_can = {
    .buffer=(volatile uint8_t *)rx_buffer,
    .tails=tails,
    .num_tails=RX_TAIL_COUNT,
};

tcp_can_frame_t tcp_rx_buffer[TCP_RX_BUFF_ITEM_COUNT];
b_tail_t tcp_tails[TCP_RX_TAIL_COUNT];
b_handle_t b_rx_tcp = {
    .buffer=(uint8_t *)tcp_rx_buffer,
    .tails=tcp_tails,
    .num_tails=TCP_RX_TAIL_COUNT,
};

int main()
{

    // TODO: use watchdog to recover if timed out?
    /* Data Struct init */
    // TODO: note: using send to back to q_tx_can2_to_can1 from the interrupt handler! (i.e. DO NOT SEND FROM NORMAL CONTEXT!)
    qConstruct(&q_tx_can2_to_can1, sizeof(CanMsgTypeDef_t));
    bConstruct(&b_rx_can, sizeof(*rx_buffer), sizeof(rx_buffer));
    bConstruct(&b_rx_tcp, 1, sizeof(tcp_rx_buffer)); // Byte resolution for tcp receive

    // TODO: size the buffer to handle the time it takes to write new file (last max loop time was 71ms)
    // TODO: investigate flushing file periodically to get the loop times down

    PHAL_trimHSI(HSI_TRIM_DAQ);
    if(0 != PHAL_configureClockRates(&clock_config))
        HardFault_Handler();

    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
        HardFault_Handler();

    PHAL_writeGPIO(ETH_RST_PORT, ETH_RST_PIN, 1);

    if (!PHAL_SPI_init(&eth_spi_config))
        HardFault_Handler();

    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    if (!PHAL_configureRTC(&start_time, false))
        HardFault_Handler();

    if(!PHAL_initUSART(&lte_usart_config, APB2ClockRateHz))
    {
        HardFault_Handler();
    }
    log_yellow("PER PER PER\n");

    if(!PHAL_initCAN(CAN1, false, VCAN_BPS))
        HardFault_Handler();
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    CAN1->IER |= CAN_IER_ERRIE | CAN_IER_LECIE |
                 CAN_IER_BOFIE | CAN_IER_EPVIE |
                 CAN_IER_EWGIE;
    NVIC_EnableIRQ(CAN1_SCE_IRQn);

#ifdef EN_CAN2
    if(!PHAL_initCAN(CAN2, false, DCAN_BPS))
        HardFault_Handler();
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
#endif

    // Link SPI for ethernet driver
    PHAL_writeGPIO(ETH_CS_PORT, ETH_CS_PIN, 1);
    reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
    reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
    reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);

    configure_exti();

    daq_init();

    log_msg("Starting main loop\n");

    daq_loop();

    log_red("Main loop exited!\n");

    return 0;
}

static void configure_exti(void)
{
    // Configure exti interupt for power loss pin (PE15)

    // Enable the SYSCFG clock for interrupts
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PE; // Map PE15 to EXTI 15
    EXTI->IMR |= EXTI_IMR_MR15; // Unmask EXTI15
    EXTI->FTSR |= EXTI_FTSR_TR15; // Enable the falling edge trigger (active low reset)

    NVIC_SetPriority(EXTI15_10_IRQn, 15); // allow other interrupts to preempt this one (especially systick and dma)
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void SysTick_Handler(void)
{
    tick_ms++;
}

/* SPI Callbacks for Ethernet Driver */

static void cs_sel(void)   {PHAL_writeGPIO(ETH_CS_PORT, ETH_CS_PIN, 0);}
static void cs_desel(void) {PHAL_writeGPIO(ETH_CS_PORT, ETH_CS_PIN, 1);}

static uint8_t spi_rb(void)
{
    uint8_t b;
    PHAL_SPI_transfer_noDMA(&eth_spi_config, NULL, 0, sizeof(b), &b);
    return b;
}

static void spi_wb(uint8_t b)
{
    PHAL_SPI_transfer_noDMA(&eth_spi_config, &b, sizeof(b), 0, NULL);
}

static void spi_rb_burst(uint8_t *pBuf, uint16_t len)
{
    // SPI RX Burst, must block! (uses local pointer)
    PHAL_SPI_transfer_noDMA(&eth_spi_config, NULL, 0, len, pBuf);
}

static void spi_wb_burst(uint8_t *pBuf, uint16_t len)
{
    // SPI TX Burst, must block! (uses local pointer)
    PHAL_SPI_transfer_noDMA(&eth_spi_config, pBuf, len, 0, NULL);
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

            // Pass-through to CAN1
            if (rx->bus_id == BUS_ID_CAN2 && rx->msg_id == (ID_LWS_STANDARD))
            {
                CanMsgTypeDef_t msg;
                msg.Bus = CAN1;
                msg.IDE = !(rx->msg_id & CAN_EFF_FLAG);
                if (msg.IDE) msg.ExtId = rx->msg_id & CAN_EFF_MASK;
                else msg.StdId = rx->msg_id & CAN_SFF_MASK;
                msg.DLC = rx->dlc;
                msg.Data[0] = rx->data[0];
                msg.Data[1] = rx->data[1];
                msg.Data[2] = rx->data[2];
                msg.Data[3] = rx->data[3];
                msg.Data[4] = rx->data[4];
                msg.Data[5] = rx->data[5];
                msg.Data[6] = rx->data[6];
                msg.Data[7] = rx->data[7];
                qSendToBack(&q_tx_can2_to_can1, &msg);
            }

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

#ifdef EN_CAN2
void CAN2_RX0_IRQHandler()
{
    can_rx_irq_handler(CAN2);
}
#endif
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



void HardFault_Handler()
{
    PHAL_writeGPIO(ERROR_LED_PORT, ERROR_LED_PIN, 1);
    while(1)
    {
        __asm__("nop");
    }
}
