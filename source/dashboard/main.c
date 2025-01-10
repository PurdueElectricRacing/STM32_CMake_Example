/* System Includes */
#include "common/bootloader/bootloader_common.h"
#include "common/common_defs/common_defs.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/adc/adc.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/faults/faults.h"

/* Module Includes */
#include "main.h"
#include "can_parse.h"
#include "daq.h"
#include "pedals.h"
#include "lcd.h"
#include "nextion.h"

GPIOInitConfig_t gpio_config[] = {
 // Status Indicators
 GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
 GPIO_INIT_OUTPUT(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
 GPIO_INIT_OUTPUT(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
 GPIO_INIT_OUTPUT_OPEN_DRAIN(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
 GPIO_INIT_OUTPUT_OPEN_DRAIN(IMD_LED_GPIO_Port, IMD_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
 GPIO_INIT_OUTPUT_OPEN_DRAIN(BMS_LED_GPIO_Port, BMS_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
 GPIO_INIT_INPUT(START_BTN_GPIO_Port, START_BTN_Pin, GPIO_INPUT_PULL_UP),
 GPIO_INIT_INPUT(BRK_STAT_TAP_GPIO_Port, BRK_STAT_TAP_Pin, GPIO_INPUT_OPEN_DRAIN),
 GPIO_INIT_INPUT(BRK_FAIL_TAP_GPIO_Port, BRK_FAIL_TAP_Pin, GPIO_INPUT_OPEN_DRAIN),

 // CAN
 GPIO_INIT_CANRX_PD0,
 GPIO_INIT_CANTX_PD1,

 // SPI Peripherals
 GPIO_INIT_SPI2_SCK_PB13,
 GPIO_INIT_SPI2_MISO_PB14,
 GPIO_INIT_SPI2_MOSI_PB15,
 GPIO_INIT_OUTPUT(EEPROM_nWP_GPIO_Port, EEPROM_nWP_Pin, GPIO_OUTPUT_LOW_SPEED),
 GPIO_INIT_OUTPUT(EEPROM_NSS_GPIO_Port, EEPROM_NSS_Pin, GPIO_OUTPUT_LOW_SPEED),

 // Throttle
 GPIO_INIT_ANALOG(THTL_1_GPIO_Port, THTL_1_Pin),
 GPIO_INIT_ANALOG(THTL_2_GPIO_Port, THTL_2_Pin),

 // Brake
 GPIO_INIT_ANALOG(BRK_1_GPIO_Port, BRK_1_Pin),
 GPIO_INIT_ANALOG(BRK_2_GPIO_Port, BRK_2_Pin),

 // Shock Pots
 GPIO_INIT_ANALOG(SHOCK_POT_L_GPIO_Port, SHOCK_POT_L_Pin),
 GPIO_INIT_ANALOG(SHOCK_POT_R_GPIO_Port, SHOCK_POT_R_Pin),

 // Normal Force
 GPIO_INIT_ANALOG(LOAD_FL_GPIO_Port, LOAD_FL_Pin),
 GPIO_INIT_ANALOG(LOAD_FR_GPIO_Port, LOAD_FR_Pin),

 // LCD
 GPIO_INIT_USART1TX_PA9,
 GPIO_INIT_USART1RX_PA10,

 // Buttons/Switches
 GPIO_INIT_INPUT(B_SELECT_GPIO_Port, B_SELECT_Pin, GPIO_INPUT_OPEN_DRAIN),
 GPIO_INIT_INPUT(B_DOWN_GPIO_Port, B_DOWN_Pin, GPIO_INPUT_OPEN_DRAIN),
 GPIO_INIT_INPUT(B_UP_GPIO_Port, B_UP_Pin, GPIO_INPUT_OPEN_DRAIN),
 GPIO_INIT_INPUT(ENC_A_GPIO_Port, ENC_A_Pin, GPIO_INPUT_OPEN_DRAIN),
 GPIO_INIT_INPUT(ENC_B_GPIO_Port, ENC_B_Pin, GPIO_INPUT_OPEN_DRAIN),
 GPIO_INIT_INPUT(DAQ_SWITCH_GPIO_Port, DAQ_SWITCH_Pin, GPIO_INPUT_OPEN_DRAIN),

 // LV Status
 GPIO_INIT_ANALOG(LV_5V_V_SENSE_GPIO_Port, LV_5V_V_SENSE_Pin),
 GPIO_INIT_ANALOG(LV_3V3_V_SENSE_GPIO_Port, LV_3V3_V_SENSE_Pin),
 GPIO_INIT_ANALOG(LV_12_V_SENSE_GPIO_Port, LV_12_V_SENSE_Pin),
 GPIO_INIT_ANALOG(LV_24_V_SENSE_GPIO_Port, LV_24_V_SENSE_Pin),
 GPIO_INIT_INPUT(LV_24_V_FAULT_GPIO_Port, LV_24_V_FAULT_Pin, GPIO_INPUT_OPEN_DRAIN),
};

volatile raw_adc_values_t raw_adc_values;

/* ADC Configuration */
ADCInitConfig_t adc_config = {
   .clock_prescaler = ADC_CLK_PRESC_6,
   .resolution      = ADC_RES_12_BIT,
   .data_align      = ADC_DATA_ALIGN_RIGHT,
   .cont_conv_mode  = true,
   .dma_mode        = ADC_DMA_CIRCULAR,
   .adc_number      = 1,
};
ADCChannelConfig_t adc_channel_config[] = {
   {.channel=THTL_1_ADC_CHNL, .rank=1, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=THTL_2_ADC_CHNL, .rank=2, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=BRK_1_ADC_CHNL,  .rank=3, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=BRK_2_ADC_CHNL,  .rank=4, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=SHOCK_POT_L_ADC_CH, .rank=5, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=SHOCK_POT_R_ADC_CH, .rank=6, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=LV_5V_V_SENSE_ADC_CHNL, .rank=7, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=LV_3V3_V_SENSE_ADC_CHNL, .rank=8, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=LV_12_V_SENSE_ADC_CHNL, .rank=9, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=LV_24_V_SENSE_ADC_CHNL, .rank=10, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=LOAD_FL_ADC_CH, .rank=11, .sampling_time=ADC_CHN_SMP_CYCLES_480},
   {.channel=LOAD_FR_ADC_CH, .rank=12, .sampling_time=ADC_CHN_SMP_CYCLES_480},
};
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &raw_adc_values, sizeof(raw_adc_values) / sizeof(raw_adc_values.t1), 0b01);

// USART Configuration for LCD
dma_init_t usart_tx_dma_config = USART1_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART1_RXDMA_CONT_CONFIG(NULL, 2);
usart_init_t lcd = {
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART1,
   .wake_addr   = false,
   .usart_active_num = USART1_ACTIVE_IDX,
   .tx_dma_cfg = &usart_tx_dma_config,
   .rx_dma_cfg = &usart_rx_dma_config
};

// Clock Configuration
#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =160000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

lcd_t lcd_data = {
    .encoder_position = 0,
};

/* Locals for Clock Rates */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

// LCD Variables
extern page_t curr_page;
volatile int8_t prev_rot_state = 0;
static volatile uint8_t dashboard_input;

/* Function Prototypes */
void preflightChecks(void);
void preflightAnimation(void);
void heartBeatLED();
void usartTxUpdate();
extern void HardFault_Handler();
void enableInterrupts();
void encoder_ISR();
void pollDashboardInput();
void sendBrakeStatus();
void interpretLoadSensor(void);
void send_shockpots();
float voltToForce(uint16_t load_read);

// Communication queues
q_handle_t q_tx_usart;

int main(void){
    /* Data Struct init */
    qConstruct(&q_tx_usart, NXT_STR_SIZE);

    /* HAL Initilization */
    PHAL_trimHSI(HSI_TRIM_DASHBOARD);
    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }
    if (false == PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
    {
        HardFault_Handler();
    }
    if (false == PHAL_initDMA(&adc_dma_config))
    {
        HardFault_Handler();
    }
    PHAL_startTxfer(&adc_dma_config);
    PHAL_startADC(ADC1);

    initFaultLibrary(FAULT_NODE_NAME, &q_tx_can[CAN1_IDX][CAN_MAILBOX_HIGH_PRIO], ID_FAULT_SYNC_DASHBOARD);

    PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 1);
    PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 1);
    PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 1);

    /* Task Creation */
    schedInit(APB1ClockRateHz);
    configureAnim(preflightAnimation, preflightChecks, 60, 2500);

    taskCreate(updateFaultDisplay, 500);
    taskCreate(updateFaultPageIndicators, 500);
    taskCreate(heartBeatLED, 500);
    taskCreate(pedalsPeriodic, 15);
    taskCreate(pollDashboardInput, 25);
    taskCreate(heartBeatTask, 100);
    taskCreate(send_shockpots, 15);
    taskCreate(interpretLoadSensor, 15);
    taskCreate(updateTelemetryPages, 200);
    taskCreate(sendTVParameters, 10000);
    taskCreate(updateSDCDashboard, 500);
    taskCreateBackground(usartTxUpdate);
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);

    schedStart();

    return 0;
}

// Call initially to ensure the LCD is initialized to the proper value -
// should be replaced with the struct prev page stuff eventually
int zeroEncoder(volatile int8_t* start_pos) {
    // Collect initial raw reading from encoder
    uint8_t raw_enc_a = PHAL_readGPIO(ENC_A_GPIO_Port, ENC_A_Pin);
    uint8_t raw_enc_b = PHAL_readGPIO(ENC_B_GPIO_Port, ENC_B_Pin);
    uint8_t raw_res = (raw_enc_b | (raw_enc_a << 1));
    *start_pos = raw_res;
    lcd_data.encoder_position = 0;

    return 1;
}

void preflightChecks(void) {
    static uint8_t state;

    switch (state++)
    {
        case 0:
            if (false == PHAL_initCAN(CAN1, false, VCAN_BPS))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
            break;
        case 1:
            if (false == PHAL_initUSART(&lcd, APB2ClockRateHz))
            {
                HardFault_Handler();
            }
            break;
        case 2:
            if (false == PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
            {
                HardFault_Handler();
            }
            if (false == PHAL_initDMA(&adc_dma_config))
            {
                HardFault_Handler();
            }
            PHAL_startTxfer(&adc_dma_config);
            PHAL_startADC(ADC1);
            break;
        case 3:
            /* Module Initialization */
            initCANParse();
            if (daqInit(&q_tx_can[CAN1_IDX][CAN_MAILBOX_LOW_PRIO]))
                HardFault_Handler();
            break;
        case 4:
            enableInterrupts();
            break;
        case 5:
            initLCD();
            break;
        case 6:
            // Zero Rotary Encoder
            zeroEncoder(&prev_rot_state);
            break;
        default:
            registerPreflightComplete(1);
            state = 255; // prevent wrap around
    }
}

void send_shockpots()
{
    uint16_t shock_l = raw_adc_values.shock_left;
    uint16_t shock_r = raw_adc_values.shock_right;
    int16_t shock_l_parsed;
    int16_t shock_r_parsed;
    // Will scale linearly from 0 - 3744. so 75 - (percent of 3744 * 75)
    shock_l_parsed =  -1 * ((POT_MAX_DIST - (int16_t)((shock_l / (POT_VOLT_MIN_L - POT_VOLT_MAX_L)) * POT_MAX_DIST)) - POT_DIST_DROOP_L);
    shock_r_parsed = -1 * ((POT_MAX_DIST - (int16_t)((shock_r / (POT_VOLT_MIN_R - POT_VOLT_MAX_R)) * POT_MAX_DIST)) - POT_DIST_DROOP_R);
    SEND_SHOCK_FRONT(shock_l_parsed, shock_r_parsed);
}

void preflightAnimation(void) {
    // Controls external LEDs since they are more visible when dash is in car
    static uint32_t time_ext;
    static uint32_t time;

    PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 1);
    PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 1);
    PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 1);
    
    PHAL_writeGPIO(HEART_LED_GPIO_Port, HEART_LED_Pin, 0);
    PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
    PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);

    switch (time++ % 6) // Creates a sweeping pattern
    {
        case 0:
        case 5:
            PHAL_writeGPIO(HEART_LED_GPIO_Port, HEART_LED_Pin, 1);
            break;
        case 1:
        case 4:
            PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
            break;
        case 2:
        case 3:
            PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
            break;
    }

    switch (time_ext++ % 4) // Creates a 25/75 blinking pattern
    {
        case 0:
            PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 0);
            PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 0);
            PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 0);
            break;
    }
}

#define SCALE_F = (1 + (3.4/6.6))
float voltToForce(uint16_t load_read) {
    /*
    //Return in newtons
    float v_out_load_l = adc_readings.load_l / 4095 * 3.3;
    float v_out_load_r = adc_readings.load_r / 4095 * 3.3;
    //voltage -> weight
    //V_out = (V_in * R_2) / (R_1 + R_2)
    //Solve for V_in
    //R_1 = 3.4K
    //R_2 = 6.6K
    float v_in_load_l = (v_out_load_l * 10) / 6.6;
    float v_in_load_r = (v_out_load_r * 10) / 6.6;
    //voltage * 100 = mass
    //weight (in newtons) = mass * g
    float force_load_l = v_in_load_l * 100 * g;
    float force_load_r = v_in_load_r * 100 * g;
    */
    float g = 9.8;
    // float val = ((load_read / 4095.0 * 3.3) * 10.0)
    float val = ((load_read / 4095.0 * 3.3) * (1.0 + (3.4/6.6)));
    // return ( val / 6.6) * 100.0 * g;
    return val * 100.0 * g;
}

void interpretLoadSensor(void) {
    float force_load_l = voltToForce(raw_adc_values.load_l);
    float force_load_r = voltToForce(raw_adc_values.load_r);
    //send a can message w/ minimal force info
    //every 15 milliseconds
    SEND_LOAD_SENSOR_READINGS_DASH(force_load_l, force_load_r);
}

void heartBeatLED()
{
    static uint8_t imd_prev_latched;
    static uint8_t bms_prev_latched;
    PHAL_toggleGPIO(HEART_LED_GPIO_Port, HEART_LED_Pin);
    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
         PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    else PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
    if (!can_data.main_hb.stale && can_data.main_hb.precharge_state) {
        PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 0);
    }
    else {
        PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 1);
    }
    if (!can_data.precharge_hb.stale) {
        if (can_data.precharge_hb.IMD)
            imd_prev_latched = 1;
        if (can_data.precharge_hb.BMS)
            bms_prev_latched = 1;
    }
    else {
        PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 0);
        PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 0);
    }
    PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, !imd_prev_latched);
    PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, !bms_prev_latched);


   static uint8_t trig;
   if (trig) SEND_DASH_CAN_STATS(can_stats.can_peripheral_stats[CAN1_IDX].tx_of,
                                can_stats.can_peripheral_stats[CAN1_IDX].tx_fail,
                                can_stats.rx_of, can_stats.can_peripheral_stats[CAN1_IDX].rx_overrun);
    trig = !trig;
}

static volatile uint32_t last_click_time;

void EXTI9_5_IRQHandler(void) {
    // EXTI9 triggered the interrupt (ENC_B_FLT)
    if (EXTI->PR & EXTI_PR_PR9) {
        encoder_ISR();
        dashboard_input |= (1 << DASH_INPUT_ROT_ENC);
        EXTI->PR |= EXTI_PR_PR9;        // Clear the interrupt pending bit for EXTI9

    }
}

void EXTI15_10_IRQHandler() {
    // EXTI10 triggered the interrupt (ENC_A_FLT)
    if (EXTI->PR & EXTI_PR_PR10) {
        encoder_ISR();
        dashboard_input |= (1 << DASH_INPUT_ROT_ENC);
        EXTI->PR |= EXTI_PR_PR10;       // Clear the interrupt pending bit for EXTI14
    }

    // EXTI14 triggered the interrupt (B1_FLT)
    // This is the TOP button on the dashboard
    if (EXTI->PR & EXTI_PR_PR14) {
        if (sched.os_ticks - last_click_time < 100) {
            last_click_time = sched.os_ticks;
            EXTI->PR |= EXTI_PR_PR14;       // Clear the interrupt pending bit for EXTI14
        }
        else {
            last_click_time = sched.os_ticks;
            dashboard_input |= (1 << DASH_INPUT_UP_BUTTON);
            EXTI->PR |= EXTI_PR_PR14;       // Clear the interrupt pending bit for EXTI14
        }
    }

    // EXTI13 triggered the interrupt (B2_FLT)
    // This is the MIDDLE button on the dashbaord
    if (EXTI->PR & EXTI_PR_PR13)
    {
        if (sched.os_ticks - last_click_time < 100) {
            last_click_time = sched.os_ticks;
            EXTI->PR |= EXTI_PR_PR13;       // Clear the interrupt pending bit for EXTI13
        }
        else
        {
            last_click_time = sched.os_ticks;
            dashboard_input |= (1 << DASH_INPUT_DOWN_BUTTON);
            EXTI->PR |= EXTI_PR_PR13;       // Clear the interrupt pending bit for EXTI13
        }
    }

    // EXTI12 triggered the interrupt (B3_FLT)
    // This is the BOTTOM button on the dashboard
    if (EXTI->PR & EXTI_PR_PR12)
    {
        if (sched.os_ticks - last_click_time < 100) {
            last_click_time = sched.os_ticks;
            EXTI->PR |= EXTI_PR_PR12;       // Clear the interrupt pending bit for EXTI12
        }
        else
        {
            last_click_time = sched.os_ticks;
            dashboard_input |= (1 << DASH_INPUT_SELECT_BUTTON);
            EXTI->PR |= EXTI_PR_PR12;       // Clear the interrupt pending bit for EXTI12
        }
    }

    // EXTI11 triggered the interrupt (START_FLT)
    if (EXTI->PR & EXTI_PR_PR11) {
        PHAL_toggleGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin); // Toggle LED for testing
        dashboard_input |= (1 << DASH_INPUT_START_BUTTON);
        EXTI->PR |= EXTI_PR_PR11;       // Clear the interrupt pending bit for EXTI11
    }
}

void encoder_ISR() {
    // [prev_state][current_state] = direction (1 = CW, -1 = CCW, 0 = no movement)
    static const int8_t encoder_transition_table[ENC_NUM_STATES][ENC_NUM_STATES] = {
        { 0, -1,  1,  0},
        { 1,  0,  0, -1},
        {-1,  0,  0,  1},
        { 0,  1, -1,  0}
    };

    uint8_t raw_enc_a = PHAL_readGPIO(ENC_A_GPIO_Port, ENC_A_Pin);
    uint8_t raw_enc_b = PHAL_readGPIO(ENC_B_GPIO_Port, ENC_B_Pin);
    uint8_t current_state = (raw_enc_b | (raw_enc_a << 1));

    // Get direction from the state transition table
    int8_t direction = encoder_transition_table[prev_rot_state][current_state];

    if (direction != 0) {
        lcd_data.encoder_position += direction;

        if (lcd_data.encoder_position >= LCD_NUM_PAGES) {
            lcd_data.encoder_position -= LCD_NUM_PAGES;
        } else if (lcd_data.encoder_position < 0) {
            lcd_data.encoder_position += LCD_NUM_PAGES;
        }
    }

    prev_rot_state = current_state;
}

void enableInterrupts()
{
    // Enable the SYSCFG clock for interrupts
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // START_FLT is on PD11 (EXTI11)
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PD;   // Map PD11 to EXTI11

    EXTI->IMR |= EXTI_IMR_MR11;                      // Unmask EXTI11
    EXTI->RTSR &= ~EXTI_RTSR_TR11;                   // Disable the rising edge trigger for START_FLT
    EXTI->FTSR |= EXTI_FTSR_TR11;                    // Enable the falling edge trigger for START_FLT

    // ENC_B_FLT is on PD9 (EXTI9)
    // ENC_A_FLT is on PD10 (EXTI10)
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PD;    // Map PD9 to EXTI9
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PD;   // Map PD10 to EXTI10

    EXTI->IMR  |= (EXTI_IMR_MR9 | EXTI_IMR_MR10);     // Unmask EXTI9 and EXTI10
    EXTI->RTSR |= (EXTI_RTSR_TR9 | EXTI_RTSR_TR10);   // Enable the rising edge trigger for both ENC_B_FLT and ENC_A_FLT
    EXTI->FTSR |= (EXTI_FTSR_TR9 | EXTI_FTSR_TR10);   // Enable the falling edge trigger for both ENC_B_FLT and ENC_A_FLT

    // B3_FLT is on PD12 (EXTI 12)
    // B2_FLT is on PD13 (EXTI 13)
    // B1_FLT is on PD14 (EXTI 14)
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PD;   // Map PD12 to EXTI 12
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PD;   // Map PD13 to EXTI 13
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PD;   // Map PD14 to EXTI 14

    EXTI->IMR |= (EXTI_IMR_MR12 | EXTI_IMR_MR13 | EXTI_IMR_MR14);         // Unmask EXTI12, EXTI13, and EXTI 14
    EXTI->RTSR &= ~(EXTI_RTSR_TR12 | EXTI_RTSR_TR13 | EXTI_RTSR_TR14);    // Disable the rising edge trigger for B3_FLT, B2_FLT, B1_FLT
    EXTI->FTSR |= (EXTI_FTSR_TR12 | EXTI_FTSR_TR13 | EXTI_FTSR_TR14);     // Enable the falling edge trigger for B3_FLT, B2_FLT, B1_FLT

    NVIC_EnableIRQ(EXTI9_5_IRQn);                    // Enable EXTI9_5 IRQ for ENC_B_FLT
    NVIC_EnableIRQ(EXTI15_10_IRQn);                  // Enable EXTI15_10 IRQ: START_FLT, ENC_A_FLT, B3_FLT, B2_FLT, B1_FLT
}

// LCD USART Communication
uint8_t cmd[NXT_STR_SIZE] = {'\0'};
void usartTxUpdate()
{
    if ((false == PHAL_usartTxBusy(&lcd)) && (SUCCESS_G == qReceive(&q_tx_usart, cmd)))
    {
        PHAL_usartTxDma(&lcd, (uint16_t *) cmd, strlen(cmd));
    }
}

void CAN1_RX0_IRQHandler()
{
    canParseIRQHandler(CAN1);
}

void dashboard_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.dashboard_bl_cmd.cmd == BLCMD_RST)
        Bootloader_ResetForFirmwareDownload();
}

// Poll for Dashboard User Input
void pollDashboardInput()
{
    static uint8_t upButtonBuffer;
    static uint8_t downButtonBuffer;

    // Check for Encoder Input
    upButtonBuffer <<= 1;
    if (PHAL_readGPIO(B_UP_GPIO_Port, B_UP_Pin) == 0)
    {
        upButtonBuffer |= 1;
    }
    upButtonBuffer &= 0b00011111;
    if (upButtonBuffer == 0b00000001)
    {
        moveUp();
    }

    downButtonBuffer <<= 1;
    if (PHAL_readGPIO(B_DOWN_GPIO_Port, B_DOWN_Pin) == 0)
    {
        downButtonBuffer |= 1;
    }
    downButtonBuffer &= 0b00011111;
    if (downButtonBuffer == 0b00000001)
    {
        moveDown();
    }

    if (dashboard_input & (1U << DASH_INPUT_ROT_ENC))
    {
        updatePage();
        dashboard_input &= ~(1U << DASH_INPUT_ROT_ENC);
    }

    // Check for Start Button Pressed
    if (dashboard_input & (1U << DASH_INPUT_START_BUTTON))
    {
        SEND_START_BUTTON(1);                     // Report start button pressed
        dashboard_input &= ~(1U << DASH_INPUT_START_BUTTON);
    }

    // Check Select Item Pressed
    if (dashboard_input & (1U << DASH_INPUT_SELECT_BUTTON))
    {
        selectItem();
        dashboard_input &= ~(1U << DASH_INPUT_SELECT_BUTTON);
    }
}

void HardFault_Handler()
{
   schedPause();
   while(1) IWDG->KR = 0xAAAA;
}
