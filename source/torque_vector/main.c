/* System Includes */
#include "common/bootloader/bootloader_common.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/faults/faults.h"
#include "common/common_defs/common_defs.h"

/* Module Includes */
#include "main.h"
#include "source/torque_vector/can/can_parse.h"

#include "bsxlite_interface.h"

#include "bmi088.h"
#include "imu.h"
#include "gps.h"

#include "ac_ext.h"
#include "ac_compute_R.h"

#include "em.h"
#include "em_pp.h"

#include "tv.h"
#include "tv_pp.h"

uint8_t collect_test[100] = {0};

GPIOInitConfig_t gpio_config[] = {
    // Status Indicators
    GPIO_INIT_OUTPUT(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_OUTPUT_LOW_SPEED),

    // SPI
    GPIO_INIT_AF(SPI_SCLK_GPIO_Port, SPI_SCLK_Pin, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MISO_GPIO_Port, SPI_MISO_Pin, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_OUTPUT(SPI_CS_ACEL_GPIO_Port, SPI_CS_ACEL_Pin, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(SPI_CS_MAG_GPIO_Port, SPI_CS_MAG_Pin, GPIO_OUTPUT_HIGH_SPEED),

    // GPS USART
    GPIO_INIT_UART4RX_PA1,
    GPIO_INIT_UART4TX_PA0,

    // GPS Auxillary pins
    GPIO_INIT_OUTPUT(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_OUTPUT_LOW_SPEED),

    // EEPROM
    GPIO_INIT_OUTPUT(NAV_EEPROM_CS_GPIO_PORT, NAV_EEPROM_CS_PIN, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(NAV_WP_GPIO_PORT, NAV_WP_PIN, GPIO_OUTPUT_HIGH_SPEED),

    // CAN
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12
    };

// GPS USART Configuration
dma_init_t usart_gps_tx_dma_config = USART4_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_gps_rx_dma_config = USART4_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t huart_gps =
{
    .baud_rate = 115200,
    .word_length = WORD_8,
    .hw_flow_ctl = HW_DISABLE,
    .stop_bits = SB_ONE,
    .parity = PT_NONE,
    .obsample = OB_DISABLE,
    .ovsample = OV_16,
    .periph      = UART4,
    .wake_addr = false,
    .usart_active_num = USART4_ACTIVE_IDX,
    .tx_errors = 0,
    .rx_errors = 0,
    .tx_dma_cfg = &usart_gps_tx_dma_config,
    .rx_dma_cfg = &usart_gps_rx_dma_config
};

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source = SYSTEM_CLOCK_SRC_HSI,
    .system_clock_target_hz = TargetCoreClockrateHz,
    .ahb_clock_target_hz = (TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz = (TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz = (TargetCoreClockrateHz / (1)),
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

SPI_InitConfig_t spi_config = {
    .data_rate = TargetCoreClockrateHz / 64,
    .data_len = 8,
    .nss_sw = true,
    .nss_gpio_port = SPI_CS_ACEL_GPIO_Port,
    .nss_gpio_pin = SPI_CS_ACEL_Pin,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI1};

// Test Nav Message
GPS_Handle_t GPSHandle = {};
vector_3d_t accel_in, gyro_in, mag_in;


BMI088_Handle_t bmi_config = {
    .accel_csb_gpio_port = SPI_CS_ACEL_GPIO_Port,
    .accel_csb_pin = SPI_CS_ACEL_Pin,
    .accel_range = ACCEL_RANGE_3G,
    .accel_odr = ACCEL_ODR_50Hz,
    .accel_bwp = ACCEL_OS_NORMAL,
    .gyro_csb_gpio_port = SPI_CS_GYRO_GPIO_Port,
    .gyro_csb_pin = SPI_CS_GYRO_Pin,
    .gyro_datarate = GYRO_DR_100Hz_32Hz,
    .gyro_range = GYRO_RANGE_250,
    .spi = &spi_config};

IMU_Handle_t imu_h = {
    .bmi = &bmi_config,
};

/* Function Prototypes */
void canTxUpdate(void);
void heartBeatLED(void);
void preflightAnimation(void);
void preflightChecks(void);
void sendIMUData(void);
void collectIMUData(void);
void VCU_MAIN(void);
extern void HardFault_Handler(void);
/**
 * q_tx_can_0 -> hlp [0,1] -> mailbox 1
 * q_tx_can_1 -> hlp [2,3] -> mailbox 2 
 * q_tx_can_2 -> hlp [4,5] -> mailbox 3
*/
q_handle_t q_tx_can_0, q_tx_can_1, q_tx_can_2;
q_handle_t q_rx_can;

/* Torque Vectoring Definitions */
static ExtU_tv rtU_tv; /* External inputs */
static ExtY_tv rtY_tv; /* External outputs */
static RT_MODEL_tv rtM_tvv;
static RT_MODEL_tv *const rtMPtr_tv = &rtM_tvv; /* Real-time model */
static DW_tv rtDW_tv;                        /* Observable states */
static RT_MODEL_tv *const rtM_tv = rtMPtr_tv;
static int16_t tv_timing;

/* Engine Map Definitions */
static ExtU_em rtU_em; /* External inputs */
static ExtY_em rtY_em; /* External outputs */
static RT_MODEL_em rtM_emv;
static RT_MODEL_em *const rtMPtr_em = &rtM_emv; /* Real-time model */
static DW_em rtDW_em;                           /* Observable states */
static RT_MODEL_em *const rtM_em = rtMPtr_em;
static int16_t em_timing;

/* Moving Median Definition */
static vec_accumulator vec_mm; /* Vector of Accumulation */
static int16_T ac_counter; /* Number of data points collected */
static bool TV_Calibrated = true; /* Flag Indicating if TV is calibrated */
static int16_T gyro_counter = 0; /* Number of steps that gyro has not been checked */

int main(void)

{
    /* Data Struct Initialization */
    qConstruct(&q_tx_can_0, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_tx_can_1, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_tx_can_2, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));

    /* HAL Initialization */
    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }

    /* GPIO initialization */
    if (!PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    configureAnim(preflightAnimation, preflightChecks, 74, 1000);

    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);

    taskCreate(heartBeatLED, 500);
    taskCreate(heartBeatTask, 100);

    taskCreate(sendIMUData, 10);
    taskCreate(collectIMUData, 10);
    taskCreate(VCU_MAIN,15);

    /* No Way Home */
    schedStart();

    return 0;
}

void preflightChecks(void)
{
    static uint16_t state;

    switch (state++)
    {
    case 0:
        if (!PHAL_initCAN(CAN1, false))
        {
            HardFault_Handler();
        }
        NVIC_EnableIRQ(CAN1_RX0_IRQn);
        break;
    case 2:
        /* USART initialization */
        if (!PHAL_initUSART(&huart_gps, APB1ClockRateHz))
        {
            HardFault_Handler();
        }
    break;
    case 3:
        // GPS Initialization
        PHAL_writeGPIO(GPS_RESET_GPIO_Port, GPS_RESET_Pin, 1);
        PHAL_usartRxDma(&huart_gps, (uint16_t *)GPSHandle.raw_message, 100, 1);
    break;
    case 5:
        initFaultLibrary(FAULT_NODE_NAME, &q_tx_can_0, ID_FAULT_SYNC_TORQUE_VECTOR);
        break;
    case 1:
        /* SPI initialization */
        if (!PHAL_SPI_init(&spi_config))
        {
            HardFault_Handler();
        }
        spi_config.data_rate = APB2ClockRateHz / 16;

        PHAL_writeGPIO(SPI_CS_ACEL_GPIO_Port, SPI_CS_ACEL_Pin, 1);
        PHAL_writeGPIO(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, 1);
        PHAL_writeGPIO(SPI_CS_MAG_GPIO_Port, SPI_CS_MAG_Pin, 1);
    break;
    case 4:
        if (!BMI088_init(&bmi_config))
        {
            HardFault_Handler();
        }
        break;
    case 250:
        BMI088_powerOnAccel(&bmi_config);
        break;
    case 500:
        if (!BMI088_initAccel(&bmi_config))
            HardFault_Handler();
        break;
    case 700:
        /* Pack torque vectoring data into rtM_tv */
        rtM_tv->dwork = &rtDW_tv;

        /* Initialize Torque Vectoring */
        tv_initialize(rtM_tv);

        /* Initialize TV IO */
        tv_IO_initialize(&rtU_tv);

        /* Pack Engine map data into rtM_em */
        rtM_em->dwork = &rtDW_em;

        /* Initialize Engine Map */
        em_initialize(rtM_em);
    default:
        if (state > 750)
        {
            if (!imu_init(&imu_h))
                HardFault_Handler();
            initCANParse(&q_rx_can);
            registerPreflightComplete(1);
            state = 750; // prevent wrap around
        }
        break;
    }
}

void preflightAnimation(void)
{
    static uint32_t time;

    PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 0);
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 0);
    PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);

    switch (time++ % 6)
    {
    case 0:
    case 5:
        PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 1);
        break;
    case 1:
    case 2:
    case 3:
        PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
        break;
    case 4:
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
        break;
    }
}

void heartBeatLED(void)
{
    PHAL_toggleGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);

    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
         PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    else PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
}

void sendIMUData(void)
{
    imu_periodic(&imu_h);
}

void collectIMUData(void)
{
    GPSHandle.messages_received++;
    BMI088_readGyro(&bmi_config, &gyro_in);
    BMI088_readAccel(&bmi_config, &accel_in);
    GPSHandle.acceleration = accel_in;
    GPSHandle.gyroscope = gyro_in;

    /* Update Gyro OK flag */
    if (gyro_counter == 150){
        GPSHandle.gyro_OK = BMI088_gyroOK(&bmi_config);
        gyro_counter = 0;
    } else {
        ++gyro_counter;
    }
}

void usart_recieve_complete_callback(usart_init_t *handle)
{
   parseVelocity(&GPSHandle);
}

/* CAN Message Handling */
void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    if (msg->IDE == 1)
    {
        // extended id, check hlp
        switch((msg->ExtId >> 26) & 0b111)
        {
            case 0:
            case 1:
                qSendToBack(&q_tx_can_0, &msg);
                break;
            case 2:
            case 3:
                qSendToBack(&q_tx_can_1, &msg);
                break;
            default:
                qSendToBack(&q_tx_can_2, &msg);
                break;
        }
    }
    else
    {
        qSendToBack(&q_tx_can_0, &msg);
    }
}

void canTxUpdate(void)
{
    CanMsgTypeDef_t tx_msg;
    if(PHAL_txMailboxFree(CAN1, 0))
    {
        if (qReceive(&q_tx_can_0, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
        {
            PHAL_txCANMessage(&tx_msg, 0);
        }
    }
    if(PHAL_txMailboxFree(CAN1, 1))
    {
        if (qReceive(&q_tx_can_1, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
        {
            PHAL_txCANMessage(&tx_msg, 1);
        }
    }
    if(PHAL_txMailboxFree(CAN1, 2))
    {
        if (qReceive(&q_tx_can_2, &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
        {
            PHAL_txCANMessage(&tx_msg, 2);
        }
    }
}

void CAN1_RX0_IRQHandler()
{
    if (CAN1->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        CAN1->RF0R &= !(CAN_RF0R_FOVR0);

    if (CAN1->RF0R & CAN_RF0R_FULL0) // FIFO Full
        CAN1->RF0R &= !(CAN_RF0R_FULL0);

    if (CAN1->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        CanMsgTypeDef_t rx;
        rx.Bus = CAN1;

        // Get either StdId or ExtId
        rx.IDE = CAN_RI0R_IDE & CAN1->sFIFOMailBox[0].RIR;
        if (rx.IDE)
        {
            rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN1->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
        }
        else
        {
            rx.StdId = (CAN_RI0R_STID & CAN1->sFIFOMailBox[0].RIR) >> CAN_RI0R_STID_Pos;
        }

        rx.DLC = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

        rx.Data[0] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> 0) & 0xFF;
        rx.Data[1] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
        rx.Data[2] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> 0) & 0xFF;
        rx.Data[5] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
        rx.Data[6] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        CAN1->RF0R |= (CAN_RF0R_RFOM0);

        qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
    }
}

void VCU_MAIN(void)
{
    /* Check if not driving */
    /* If Energized -> Accumulate Acceleration Vector */
    /* If R2D -> Do TV */
    /* Else -> Do Nothing */
    if ((can_data.main_hb.car_state == 2) & (ac_counter <= NUM_ELEM_ACC_CALIBRATION)) {
        /* Accumulate Acceleration Vector */
        vec_mm.ax[ac_counter] = GPSHandle.acceleration.x;
        vec_mm.ay[ac_counter] = GPSHandle.acceleration.y;
        vec_mm.az[ac_counter] = GPSHandle.acceleration.z;
        ++ac_counter;

        /* If length Acceleration Vector > ##, Compute R */
        if (ac_counter == NUM_ELEM_ACC_CALIBRATION) {
            ac_compute_R(vec_mm.ax, vec_mm.ay, vec_mm.az, rtU_tv.R);
        }

    } else if ((can_data.main_hb.car_state == 4)) {
        TV_Calibrated = (ac_counter == NUM_ELEM_ACC_CALIBRATION);
        /* Populate torque vectoring inputs */
        tv_pp(&rtU_tv, &GPSHandle);

        /* Step torque vectoring */
        tv_timing = sched.os_ticks;
        tv_step(rtMPtr_tv, &rtU_tv, &rtY_tv);
        tv_timing = sched.os_ticks - tv_timing;

        /* Populate Engine map inputs */
        em_pp(&rtU_em, &rtY_tv);

        /* Step Engine map */
        em_timing = sched.os_ticks;
        em_step(rtMPtr_em, &rtU_em, &rtY_em);
        em_timing = sched.os_ticks - em_timing;
    }

    /* Set Faults */
    setFault(ID_TV_DISABLED_FAULT,!rtY_em.TVS_PERMIT);
    setFault(ID_TV_UNCALIBRATED_FAULT,!TV_Calibrated);
    setFault(ID_NO_GPS_FIX_FAULT,!rtU_tv.F_raw[8]);

    /* Send Messages */
    SEND_THROTTLE_VCU((int16_t)(rtY_em.k[0]*4095),(int16_t)(rtY_em.k[1]*4095));
    SEND_THROTTLE_REMAPPED((int16_t)(rtY_em.k[0]*4095),(int16_t)(rtY_em.k[1]*4095));

    SEND_SFS_ACC((int16_t)(rtY_tv.sig_filt[15] * 100),
                    (int16_t)(rtY_tv.sig_filt[16] * 100), (int16_t)(rtY_tv.sig_filt[17] * 100));
    SEND_SFS_ANG_VEL((int16_t)(rtY_tv.sig_filt[7] * 10000),
                     (int16_t)(rtY_tv.sig_filt[8] * 10000), (int16_t)(rtY_tv.sig_filt[9] * 10000));

    SEND_MAXR((int16_t)(rtY_tv.max_K*100));

    //SEND_SFS_POS((int16_t)(rtY.pos_VNED[0] * 100),
    //             (int16_t)(rtY.pos_VNED[1] * 100), (int16_t)(rtY.pos_VNED[2] * 100));
    //SEND_SFS_VEL((int16_t)(rtY.vel_VNED[0] * 100),
    //             (int16_t)(rtY.vel_VNED[1] * 100), (int16_t)(rtY.vel_VNED[2] * 100));
    //SEND_SFS_ANG((int16_t)(rtY.ang_NED[0] * 10000),
    //             (int16_t)(rtY.ang_NED[1] * 10000), (int16_t)(rtY.ang_NED[2] * 10000), (int16_t)(rtY.ang_NED[3] * 10000));
}

void torquevector_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.torquevector_bl_cmd.cmd == BLCMD_RST)
        Bootloader_ResetForFirmwareDownload();
}

void HardFault_Handler()
{
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
    while (1)
    {
        __asm__("nop");
    }
}