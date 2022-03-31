#include "bms.h"

// Static function prototypes
static void memsetu(uint8_t* ptr, uint8_t val, uint32_t size);

bms_t bms;

// @funcname: bmsStatus
//
// @brief: Posts BMS status and tracks mode
void bmsStatus()
{
    static uint8_t flag;

    if (bms.error != 0)
    {
        flag = !flag;
    }
    else
    {
        flag = 0;
    }

    PHAL_toggleGPIO(LED_HEART_GPIO_Port, LED_HEART_Pin);
    PHAL_writeGPIO(LED_ERR_GPIO_Port, LED_ERR_Pin, flag);
    PHAL_writeGPIO(LED_CONN_GPIO_Port, LED_CONN_Pin, bms.afe_con);

    if (bms.veh_con == 1)
    {
        bms.op_mode = MODE_DISCHARGE;
    }
    else if (bms.power_out < MIN_CHG_CURR)
    {
        bms.op_mode = MODE_CHARGE;
    }
    else
    {
        bms.op_mode = MODE_IDLE;
    }
}

// @funcname: initBMS
//
// @brief: Initializes BMS parameters and scheduler
void initBMS(SPI_InitConfig_t* hspi)
{
    memsetu((uint8_t*) &bms, 0, sizeof(bms));
    bms.spi = hspi;
    bms.temp_master = 0;

#ifdef BMS_ACCUM
    bms.cell_count = 10;
    bms.temp_count = 20;
    bms.curr_sense_conn = 1;
#endif
#ifdef BMS_LV
    bms.cell_count = 8;
    bms.temp_count = 0;
    bms.curr_sense_conn = 0;
#endif

    checkConn();

    while ((bms.error & 0x1)) {
        PHAL_writeGPIO(LED_ERR_GPIO_Port, LED_ERR_Pin, 1);
        PHAL_writeGPIO(LED_HEART_GPIO_Port, LED_HEART_Pin, 0);
        checkConn();
    }

    // TODO: Fix timeout
    // bms.temp_master = checkTempMaster(TEMP_ID1);
}

// @funcname: setPLim
//
// @brief: Determine the master power limit to use
void setPLim(void)
{
    uint16_t lim_temp = bms.p_lim.soc_max;

    if (bms.p_lim.temp_max < lim_temp)
    {
        lim_temp = bms.p_lim.temp_max;
    }

    if (bms.p_lim.v_max < lim_temp)
    {
        lim_temp = bms.p_lim.v_max;
    }

    bms.master_p_lim = lim_temp;
}

// @funcname: calcMisc
//
// @brief: Run miscellaneous, pack level calcs
void calcMisc(void)
{
    bms.power_out = bms.current_out * bms.voltage_out;
}

void checkLVStatus(void)
{
    static float mod_volts_last;

    if (bms.curr_sense_conn)
    {
        return;
    }

    if (bms.cells.mod_volts_conv > (mod_volts_last + CELL_CHARGE_IMPLAUS))
    {
        bms.current_out = -10;
    }
    else if (bms.cells.mod_volts_conv < (mod_volts_last - CELL_CHARGE_IMPLAUS))
    {
        bms.current_out = 10;
    }
    else
    {
        bms.current_out = 0;
    }

    mod_volts_last = bms.cells.mod_volts_conv;
}

// @funcname: checkSleep
//
// @brief: Checks if we want to sleep, and ensures that all
//         tasks are in a solid state for sleep
void checkSleep(void)
{
    // Wait for tasks to spool down (handled on a case by case basis)
    if (bms.no_sleep)
    {
        return;
    }

    // Handle WWDG
    IWDG->KR  = 0x5555;
    IWDG->PR  = 7;
    IWDG->RLR = 0x7ff;

    while ((IWDG->SR & 0b111) != 0);
    IWDG->KR  = 0xAAAA;

    // Put everything to sleep (AFE autosleeps after 2s)

    // Finalize sleep parameters

    // Night night
    schedPause();
}

// @funcname: memsetu
//
// @brief: Simple memset routine
//
// @param: ptr: Pointer to location to set
// @param: val: Value to set each memory address to
// @param: size: Length of data to set
static void memsetu(uint8_t* ptr, uint8_t val, uint32_t size)
{
    // Locals
    uint32_t i;

    for (i = 0; i < size; i++)
    {
        ptr[i] = val;
    }
}