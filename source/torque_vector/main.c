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

    // LEDS
    GPIO_INIT_OUTPUT(ERR_LED_PORT, ERR_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_PORT, CONN_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_ERR_LED_PORT, SD_ERR_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_ACT_LED_PORT, SD_ACT_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_DET_LED_PORT, SD_DET_LED_PIN, GPIO_OUTPUT_LOW_SPEED),

    // SPI1 - ACCEL & GYRO
    // TODO why open drain here?
    GPIO_INIT_AF(SPI1_SCLK_PORT, SPI1_SCLK_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI1_MOSI_PORT, SPI1_MOSI_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI1_MISO_PORT, SPI1_MISO_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_OUTPUT(SPI1_CSB_ACCEL_PORT, SPI1_CSB_ACCEL_PIN, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, GPIO_OUTPUT_HIGH_SPEED),

    // GPS SPI
    // GPIO_INIT_AF(SPI2_CLK_GPS_PORT, SPI2_CLK_GPS_PIN, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    // GPIO_INIT_UART4RX_PA1,
    // GPIO_INIT_UART4TX_PA0,

    // GPS Auxillary pins
    GPIO_INIT_OUTPUT(RESET_GPS_PORT, RESET_GPS_PIN, GPIO_OUTPUT_LOW_SPEED),

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

#define TargetCoreClockrateHz 96000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =192000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
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
    .nss_gpio_port = SPI1_CSB_ACCEL_PORT,
    .nss_gpio_pin = SPI1_CSB_ACCEL_PIN,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI1};

// Test Nav Message
GPS_Handle_t GPSHandle = {};
vector_3d_t accel_in, gyro_in, mag_in;


BMI088_Handle_t bmi_config = {
    .accel_csb_gpio_port = SPI1_CSB_ACCEL_PORT,
    .accel_csb_pin = SPI1_CSB_ACCEL_PIN,
    .accel_range = ACCEL_RANGE_3G,
    .accel_odr = ACCEL_ODR_50Hz,
    .accel_bwp = ACCEL_OS_NORMAL,
    .gyro_csb_gpio_port = SPI1_CSB_GYRO_PORT,
    .gyro_csb_pin = SPI1_CSB_GYRO_PIN,
    .gyro_datarate = GYRO_DR_100Hz_32Hz,
    .gyro_range = GYRO_RANGE_250,
    .spi = &spi_config};

IMU_Handle_t imu_h = {
    .bmi = &bmi_config,
};

/* Function Prototypes */
void heartBeatLED(void);
void preflightAnimation(void);
void preflightChecks(void);
extern void HardFault_Handler(void);

void parseIMU(void);
void pollIMU(void);
void VCU_MAIN(void);

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
static DW_em rtDW_em;                        /* Observable states */
static RT_MODEL_em *const rtM_em = rtMPtr_em;
static int16_t em_timing;

/* Moving Median Definition */
static vec_accumulator vec_mm; /* Vector of Accumulation */
static int16_t ac_counter = 0; /* Number of data points collected */
static bool TV_Calibrated = false; /* Flag Indicating if TV is calibrated */
static int16_t gyro_counter = 0; /* Number of steps that gyro has not been checked */

int main(void)
{
    /* Data Struct Initialization */

    /* HAL Initialization */
    PHAL_trimHSI(HSI_TRIM_TORQUE_VECTOR);
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

    taskCreate(parseIMU, 20);
    taskCreate(pollIMU, 20);
    taskCreate(VCU_MAIN, 15);

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
        if (!PHAL_initCAN(CAN1, false, VCAN_BPS))
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
        PHAL_writeGPIO(RESET_GPS_PORT, RESET_GPS_PIN, 1);
        PHAL_usartRxDma(&huart_gps, (uint16_t *)GPSHandle.raw_message, 100, 1);
    break;
    case 5:
        initFaultLibrary(FAULT_NODE_NAME, &q_tx_can[CAN1_IDX][CAN_MAILBOX_HIGH_PRIO], ID_FAULT_SYNC_TORQUE_VECTOR);
        break;
    case 1:
        /* SPI initialization */
        if (!PHAL_SPI_init(&spi_config))
        {
            HardFault_Handler();
        }
        spi_config.data_rate = APB2ClockRateHz / 16;

        PHAL_writeGPIO(SPI1_CSB_ACCEL_PORT, SPI1_CSB_ACCEL_PIN, 1);
        PHAL_writeGPIO(SPI1_CSB_GYRO_PORT, SPI1_CSB_GYRO_PIN, 1);
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
            initCANParse();
            registerPreflightComplete(1);
            state = 750; // prevent wrap around
        }
        break;
    }
}

void preflightAnimation(void)
{
    static uint32_t time;

    PHAL_writeGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, 0);
    PHAL_writeGPIO(ERR_LED_PORT, ERR_LED_PIN, 0);
    PHAL_writeGPIO(CONN_LED_PORT, CONN_LED_PIN, 0);

    switch (time++ % 6)
    {
    case 0:
    case 5:
        PHAL_writeGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, 1);
        break;
    case 1:
    case 2:
    case 3:
        PHAL_writeGPIO(ERR_LED_PORT, ERR_LED_PIN, 1);
        break;
    case 4:
        PHAL_writeGPIO(CONN_LED_PORT, CONN_LED_PIN, 1);
        break;
    }
}

void heartBeatLED(void)
{
    PHAL_toggleGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);

    if ((sched.os_ticks - last_can_rx_time_ms) >= 1000)
         PHAL_writeGPIO(CONN_LED_PORT, CONN_LED_PIN, 0);
    else PHAL_writeGPIO(CONN_LED_PORT, CONN_LED_PIN, 1);


    static uint8_t trig;
    if (trig) SEND_TV_CAN_STATS(can_stats.can_peripheral_stats[CAN1_IDX].tx_of,
                                can_stats.can_peripheral_stats[CAN1_IDX].tx_fail,
                                can_stats.rx_of, can_stats.can_peripheral_stats[CAN1_IDX].rx_overrun);
    trig = !trig;
}

void pollIMU(void)
{
    imu_periodic(&imu_h);
}

void parseIMU(void)
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
void CAN1_RX0_IRQHandler()
{
    canParseIRQHandler(CAN1);
}

void VCU_MAIN(void)
{
    /* Initialize Throttles */
    int16_t tvs_k_rl = 0;
    int16_t tvs_k_rr = 0;
    int16_t equal_k_rl = 0;
    int16_t equal_k_rr = 0;

    /* If precharging -> Accumulate acceleration vector */
    if (/*(can_data.main_hb.car_state == 1) & */(ac_counter <= NUM_ELEM_ACC_CALIBRATION)) {
        /* Accumulate acceleration vector */
        vec_mm.ax[ac_counter] = GPSHandle.acceleration.x;
        vec_mm.ay[ac_counter] = GPSHandle.acceleration.y;
        vec_mm.az[ac_counter] = GPSHandle.acceleration.z;
        ++ac_counter;

        /* If length of acceleration vector == ##, compute R */
        if (ac_counter == NUM_ELEM_ACC_CALIBRATION) {
            ac_compute_R(vec_mm.ax, vec_mm.ay, vec_mm.az, rtU_tv.R);
            TV_Calibrated = true;
        }
    }

    /* Populate torque vectoring inputs */
    tv_pp(&rtU_tv, &GPSHandle);

    /* Step torque vectoring */
    tv_timing = sched.os_ticks;
    tv_step(rtMPtr_tv, &rtU_tv, &rtY_tv);
    tv_timing = sched.os_ticks - tv_timing;

    /* Populate engine map inputs */
    em_pp(&rtU_em, &rtY_tv);

    /* Step engine map */
    em_timing = sched.os_ticks;
    em_step(rtMPtr_em, &rtU_em, &rtY_em);
    em_timing = sched.os_ticks - em_timing;

    /* Set TV faults */
    setFault(ID_TV_DISABLED_FAULT,!rtY_tv.TVS_STATE);
    setFault(ID_MM_DISABLED_FAULT,!rtY_em.MM_STATE);
    setFault(ID_TV_UNCALIBRATED_FAULT,!TV_Calibrated);
    setFault(ID_NO_GPS_FIX_FAULT,!rtU_tv.F_raw[8]);

    setFault(ID_TV_ENABLED_FAULT,rtY_tv.TVS_STATE);
    setFault(ID_MM_ENABLED_FAULT,rtY_em.MM_STATE);
    setFault(ID_TV_CALIBRATED_FAULT,TV_Calibrated);
    setFault(ID_YES_GPS_FIX_FAULT,rtU_tv.F_raw[8]);

    /* Get motor commands */
    tvs_k_rl = (int16_t)(rtY_em.kTVS[0]*4095);
    tvs_k_rr = (int16_t)(rtY_em.kTVS[1]*4095);
    equal_k_rl = (int16_t)(rtY_em.kEQUAL[0]*4095);
    equal_k_rr = (int16_t)(rtY_em.kEQUAL[1]*4095);

    /* Send messages */
    SEND_THROTTLE_VCU(tvs_k_rl,tvs_k_rr);
    SEND_MAXR((int16_t)(rtY_tv.max_K*4095));

    SEND_SFS_ACC((int16_t)(rtY_tv.sig_filt[15] * 100),(int16_t)(rtY_tv.sig_filt[16] * 100), (int16_t)(rtY_tv.sig_filt[17] * 100));
    SEND_SFS_ANG_VEL((int16_t)(rtY_tv.sig_filt[7] * 10000),(int16_t)(rtY_tv.sig_filt[8] * 10000), (int16_t)(rtY_tv.sig_filt[9] * 10000));
}

void torquevector_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.torquevector_bl_cmd.cmd == BLCMD_RST)
        Bootloader_ResetForFirmwareDownload();
}

void HardFault_Handler()
{
    PHAL_writeGPIO(ERR_LED_PORT, ERR_LED_PIN, 1);
    while (1)
    {
        __asm__("nop");
    }
}
