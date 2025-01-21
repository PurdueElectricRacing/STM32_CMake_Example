/**
 * @file eeprom_spi.c
 * @author Luke Oxley (lcoxley@purdue.edu), modification of Dawson Moore's eeprom.c for SPI support
 * @version 0.1
 * @date 2022-12-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "common/eeprom_spi/eeprom_spi.h"

// Local defines
#define E_READ  0x03    //!< Read data from memory beginning at selected address
#define E_WRITE 0x02    //!< Write data to memory beginning at selected address
#define E_WRDI  0x04    //!< Reset the write enable latch (disable write operations)
#define E_WREN  0x06    //!< Set the write enable latch (enable write operations)
#define E_RDSR  0x05    //!< Read STATUS register
#define E_WRSR  0x01    //!< Write STATUS register

// EEPROM struct
struct eeprom mem;
uint8_t       mem_zero[MICRO_PG_SIZE];

// Static prototypes
static int  readMem(uint16_t phys_addr, uint8_t* loc_addr, uint16_t len);
// static int  writePage(uint16_t addr, uint8_t* page, uint8_t size);
// static int  readPage(uint16_t addr, uint8_t* page);
static void memClear(void);
static int  memTest(void);
static int  fnameSearch(char* name);
static int  ee_memcheck(uint8_t* src, uint8_t* dest, size_t len);
static void ee_memset(uint8_t* dest, size_t len, uint8_t value);
static void ee_memcpy(uint8_t* src, uint8_t* dest, size_t len);
static uint8_t ee_get_idx(uint32_t *req);
static void ee_clear_idx(uint32_t *req, uint8_t idx);
static void ee_request_flush_physical();

/**
 * @brief: Initializes chip metadata. Attemps to read
 *         and load all metadata from last run. If memory
 *         isn't initialized, force_init can be set to
 *         set a default version of 1 and mark the chip
 *         as officially in use
 *
 * @param: wc_gpio_port: Mem lock port0
 * @param: wc_gpio_pin: Mem lock pin
 * @param: version: Version of app code
 * @param: force_init: Force initialization if memory isn't
 *                     initialized. Only set if version is truly 1
 */
int initMem(GPIO_TypeDef* wc_gpio_port, uint32_t wc_gpio_pin, SPI_InitConfig_t* spi, uint16_t version, bool force_init) {
    int      ret;
    uint16_t i, size, end;
    uint8_t  page[MICRO_PG_SIZE];

    mem = (struct eeprom) {0};

    // Set WC pins
    mem.wc_gpio_port = wc_gpio_port;
    mem.wc_gpio_pin = wc_gpio_pin;

    mem.spi = spi;

    // Read EEPROM metadata
    ret = readMem(0, (uint8_t*) &mem.phys, sizeof(struct phys_mem));
    ee_memset(mem_zero, MICRO_PG_SIZE, 0);

    // Ensure there are no errors
    if (ret < 0) {
        return ret;
    }

    // Check if the memory isn't initialized
    if (mem.phys.init_key != INIT_KEY) {
        // It isn't, and we want it to be, so set to default values
        if (force_init) {
            mem.phys.init_key = INIT_KEY;
            mem.phys.version = 1;
            mem.init_physical = true;
            ee_memset(mem.phys.filename, sizeof(mem.phys.filename), 0);
            ee_memset((uint8_t *) mem.phys.pg_bound, sizeof(mem.phys.pg_bound), 0);
            ee_memset((uint8_t *) mem.phys.mem_size, sizeof(mem.phys.mem_size) , 0);
            ee_memset(mem.phys.bcmp, sizeof(mem.phys.bcmp) , 0);
            ee_request_flush_physical();

            return E_SUCCESS;
        } else {
            return -E_NO_INIT;
        }
    }

    // Check if we have a version match
    ret = checkVersion(version);

    // Return error code if version is off
    if (ret < 0) {
        return -E_V_MISMATCH;
    }

    // Start loading process
    // TODO: commented out, I think load should happen after init called inside the mapMem function if file found
    // ee_memset(page, NAME_LEN, 0);

    // for (i = 0; i < MAX_PAGES; i++) {
    //     if (ee_memcheck(page, (char*) &mem.phys.filename[i], NAME_LEN) < 0) {
    //         // If filename exists, load file from eeprom, assuming mapMem called
    //         ret = readMem(mem.phys.pg_bound[i], mem.pg_addr[i], mem.pg_size[i]);

    //         if (ret < 0) {
    //             return ret;
    //         }
    //     }
    // }

    mem.init_physical = true;

    return E_SUCCESS;
}

/**
 * @brief: Checks version to ensure app code is new enough
 *
 * @param: dest: Pointer to location to set
 * @param: len: Length to set
 * @param: value: Value to set each memory address to
 * @param: Difference between current and chip versions
 *         if app code is newer, -E_V_MISMATCH if app
 *         code is old. -E_NO_INIT if memory hasn't
 *         been initialized yet
 */
int checkVersion(uint16_t version) {
    // Check if we're even initialized
    if (mem.phys.init_key != INIT_KEY) {
        return -E_NO_INIT;
    }

    // Check if our software is a later version
    if (mem.phys.version < version) {
        return -E_V_MISMATCH;
    } else {
        return mem.phys.version - version;
    }
}

/**
 * @brief: Maps local address to chip address
 *
 * @param: addr: Pointer to local address
 * @param: len: Length of data
 * @param: fname: File name (NAME_LEN characters)
 * @param: bcmp: Backwards compatibility enabled. Disable for temp storage
 */
int mapMem(uint8_t* addr, uint16_t len, uint8_t* fname, bool bcmp) {
    int     i;
    uint8_t null_name[NAME_LEN], ret;

    ee_memset(null_name, NAME_LEN, 0U);

    // Check if we already found this memory on EEPROM
    i = fnameSearch(fname);

    if (i >= 0) {
        mem.pg_addr[i] = addr;
        mem.pg_size[i] = len;
        if (len < mem.phys.mem_size[i])
        {
            // Variable deleted, update size
            mem.phys.mem_size[i] = mem.pg_size[i];
            ee_request_flush_physical();
            ret = readMem(mem.phys.pg_bound[i], mem.pg_addr[i], mem.pg_size[i]);
            if (ret < 0) return ret;
        }
        else if (len > mem.phys.mem_size[i])
        {
            // Variable added, ensure enough space left in page
            if ((mem.phys.mem_size[i] % MICRO_PG_SIZE) != 0 &&
                (mem.phys.mem_size[i] % MICRO_PG_SIZE) + (len - mem.phys.mem_size[i]) <= MICRO_PG_SIZE)
            {
                // Read portion that was saved before
                ret = readMem(mem.phys.pg_bound[i], mem.pg_addr[i], mem.phys.mem_size[i]);
                if (ret < 0) return ret;
                mem.phys.mem_size[i] = mem.pg_size[i];
                ee_request_flush_physical();
                // Make sure the new defaults get saved
                requestFlush(fname);
            }
            else return -12;
        }
        else
        {
            // Files equal lengths, normal operation
            ret = readMem(mem.phys.pg_bound[i], mem.pg_addr[i], mem.pg_size[i]);
            if (ret < 0) return ret;
        }

        return E_SUCCESS;
    }

    // Check for the first free location
    for (i = 0; i < MAX_PAGES; i++) {
        if (ee_memcheck(null_name, &mem.phys.filename[i * NAME_LEN], NAME_LEN) == 0) {
            break;
        }
    }

    // Check if we found a free location
    if (i == MAX_PAGES) {
        return -E_NO_MEM;
    }

    // Copy over new metadata
    ee_memcpy(fname, &mem.phys.filename[i * NAME_LEN], NAME_LEN);
    mem.phys.mem_size[i] = len;
    mem.phys.bcmp[i] = bcmp;
    mem.phys.pg_bound[i] = (i == 0) ? MACRO_PG_SIZE : ROUNDUP(mem.phys.pg_bound[i - 1] + mem.phys.mem_size[i - 1], MICRO_PG_SIZE);
    mem.pg_addr[i] = addr;
    mem.pg_size[i] = len;
    ee_request_flush_physical();
    requestFlush(fname); // Write to default values

    return E_SUCCESS;
}

typedef enum {
    BG_IDLE  = 0,
    BG_ZERO  = 1,
    BG_FLUSH_META = 2,
    BG_LOAD  = 3,
    BG_FLUSH = 4
} BG_State;

static uint8_t  curr_page;
static uint32_t addr;
static BG_State bg_state;

/**
 * @brief: Background task for searching for stale data
 *         If metadata or any mapped struct is updated,
 *         it will be written to the device once searched
 *         by the foreground loop
 *
 * @note: Application code must add to background queue with rate MEM_FG_TIME
 */
void memBg(void) {
    int      ret;
    static uint8_t  page[MICRO_PG_SIZE];
    uint16_t len, end;

    // Check if we're waiting on foreground to execute a write
    if (mem.write_pending || !mem.init_physical) {
        return;
    }

    // Check for action
    if (bg_state == BG_IDLE)
    {
        if (mem.zero_req) bg_state = BG_ZERO;
        else if (mem.flush_physical) bg_state = BG_FLUSH_META;
        else if (mem.req_flush[0] | mem.req_flush[1]) bg_state = BG_FLUSH;
        else if (mem.req_load[0] | mem.req_load[1]) bg_state = BG_LOAD;
        else return; // Nothing to do
        addr = 0; // restart
        curr_page = 0;
    }

    switch(bg_state)
    {
        case BG_IDLE:
        break;
        case BG_ZERO:
            mem.write_pending = true;
            mem.source_loc = mem_zero;
            mem.dest_loc = addr;
            mem.update_len = MICRO_PG_SIZE;
            addr += MICRO_PG_SIZE;
            if (addr >= CHIP_SIZE) {
                mem.zero_req = false;
                bg_state = BG_IDLE;
            }
        break;
        case BG_LOAD:
            curr_page = ee_get_idx(mem.req_load);
            addr = mem.phys.pg_bound[curr_page];
            // Loads do all pages at once (don't want half data in memory when using it)
            readMem(addr, mem.pg_addr[curr_page], mem.phys.mem_size[curr_page]);
            ee_clear_idx(mem.req_load, curr_page);
            bg_state = BG_IDLE;
            addr = 0;
            curr_page = 0;
        break;
        case BG_FLUSH:
            if (curr_page == 0)
            {
                curr_page = ee_get_idx(mem.req_flush);
                addr = mem.phys.pg_bound[curr_page];
            }
            end = mem.phys.pg_bound[curr_page] + mem.pg_size[curr_page];
            len = (addr + MICRO_PG_SIZE > end) ? (mem.pg_size[curr_page] % MICRO_PG_SIZE) : MICRO_PG_SIZE;
            ret = readMem(addr, page, len);

            if (ret < 0) {
                return;
            }

            ret = ee_memcheck(page, (uint8_t*) (mem.pg_addr[curr_page] + addr - mem.phys.pg_bound[curr_page]), len);

            if (ret < 0) {
                mem.write_pending = true;
                mem.dest_loc = addr;
                mem.source_loc = (uint8_t*) (mem.pg_addr[curr_page] + addr - mem.phys.pg_bound[curr_page]);
                mem.update_len = len;
            }

            addr += MICRO_PG_SIZE;

            if (addr >= end) {
                ee_clear_idx(mem.req_flush, curr_page);
                addr = 0;
                curr_page = 0;
                bg_state = BG_IDLE;
            }
        break;
        case BG_FLUSH_META:
            // assumes curr_page and addr reset to 0 before initial
            end = sizeof(struct phys_mem);
            len = (addr + MICRO_PG_SIZE > end) ? (sizeof(struct phys_mem) % MICRO_PG_SIZE) : MICRO_PG_SIZE;
            ret = readMem(addr, page, len);

            if (ret < 0) {
                return;
            }

            ret = ee_memcheck(page, ((uint8_t*) &mem.phys) + addr, len);

            if (ret < 0) {
                mem.write_pending = true;
                mem.dest_loc = addr;
                mem.source_loc = ((uint8_t*) &mem.phys) + addr;
                mem.update_len = len;
            }

            addr += MICRO_PG_SIZE;

            if (addr >= sizeof(struct phys_mem)) {
                addr = 0;
                curr_page = 0;
                mem.flush_physical = 0;
                bg_state = BG_IDLE;
            }
        break;
        default:
        return;
        break;
    }
}

/**
 * @brief: Foreground routine for 5ms write coherency
 *
 * @note: Application code must add to foreground queue
 */
void memFg(void) {
    if (!mem.write_pending) {
        return;
    }

    // TODO: Signal to app code if an error occurs
    writePage(mem.dest_loc, mem.source_loc, mem.update_len);
    mem.write_pending = false; // TODO: Since this is called t 5ms, should be okay to set here...
}

/**
 * @brief: Reads memory across micro page boundaries
 *         Blocks until data fully read
 *         Assumes phys_addr is a page_bd
 *
 * @param: phys_addr: On chip address of data
 * @param: loc_addr: Local address of data
 * @param: len: Length of data
 *
 * @return: E_SUCCESS if read, error code if failed
 */
static int readMem(uint16_t phys_addr, uint8_t* loc_addr, uint16_t len) {
    int      ret;
    uint8_t  page[MICRO_PG_SIZE];
    uint16_t i, size, end;

    // Calculate ending address
    end = phys_addr + len; // (non-inclusive)

    // Read page by page
    for (i = phys_addr; i < end; i += MICRO_PG_SIZE) {
        while(PHAL_SPI_busy(mem.spi)); // Block
        ret = readPage(i, page);
        while(PHAL_SPI_busy(mem.spi)); // Block

        if (ret < 0) {
            return ret;
        }

        // Check if micro page would overflow during memcpy
        size = (i + MICRO_PG_SIZE > end) ? (len % MICRO_PG_SIZE) : MICRO_PG_SIZE;
        ee_memcpy(page, (uint8_t*) (loc_addr + (i - phys_addr)), size);
    }

    return E_SUCCESS;
}

/**
 * @brief: Writes a single micro page to chip
 *         Must wait 5ms between calls
 *         Fails if SPI busy
 *
 * @param: addr: On chip address of data
 * @param: page: Pointer to data to write
 * @param: size: Length of write (capped at MICRO_PG_SIZE)
 *
 * @note: E_SUCCESS if written, -E_SPI if failed
 */
int writePage(uint16_t addr, uint8_t* page, uint8_t size) {
    uint8_t ret;

    size = (size > MICRO_PG_SIZE) ? MICRO_PG_SIZE : size;

    static uint8_t tx_command[3];
    static uint8_t rx_buff[3];

    if (PHAL_SPI_busy(mem.spi)) return -E_SPI;

    // Must send a E_WREN before each write command (NSS must go low and back high to latch)
    tx_command[0] = E_WREN;
    mem.spi->nss_sw = true; // SPI library can deal with this
    ret = !PHAL_SPI_transfer(mem.spi, tx_command, 1, NULL);
    while(PHAL_SPI_busy(mem.spi));

    tx_command[0] = E_WRITE;
    tx_command[1] = addr >> 8;
    tx_command[2] = addr & 0xFF;

    // Complete write
    mem.spi->nss_sw = false; // Take control for sending address
    PHAL_writeGPIO(mem.spi->nss_gpio_port, mem.spi->nss_gpio_pin, 0); // start transfer
    ret |= !PHAL_SPI_transfer(mem.spi, tx_command, 3, NULL);
    while(PHAL_SPI_busy(mem.spi));
    mem.spi->nss_sw = true; // Give back control (DMA interrupt will set NSS back high in the interrupt)
    ret |= !PHAL_SPI_transfer(mem.spi, page, size, NULL);

    return ret ? -E_SPI : E_SUCCESS;
}

/**
* @brief: Reads a single micro page from chip
*         Fails if SPI busy
*
* @param: addr: On chip address of data
* @param: page: Pointer to returned data
*
* @param: E_SUCCESS if read, -E_SPI if failed
*/
int readPage(uint16_t addr, uint8_t* page) {
    uint8_t ret;

    static uint8_t tx_command[3];
    tx_command[0] = E_READ;
    tx_command[1] = addr >> 8;
    tx_command[2] = addr & 0xFF;

    if(PHAL_SPI_busy(mem.spi)) return -E_SPI;

    // Complete read
    mem.spi->nss_sw = false; // Take control for sending address
    PHAL_writeGPIO(mem.spi->nss_gpio_port, mem.spi->nss_gpio_pin, 0); // Start transfer
    ret = !PHAL_SPI_transfer(mem.spi, tx_command, 3, NULL);
    while(PHAL_SPI_busy(mem.spi));
    mem.spi->nss_sw = true; // Give back control (DMA interrupt will set NSS back high in the interrupt)
    ret |= !PHAL_SPI_transfer(mem.spi, NULL, MICRO_PG_SIZE, page);

    return ret ? -E_SPI : E_SUCCESS;
}

void requestLoad(char* name)
{
    int i = fnameSearch(name);
    if (i < 0) return;
    if (i < 32)
        mem.req_load[0] |= 0b1 << i;
    else
        mem.req_load[1] |= 0b1 << (i - 32);
}

void requestFlush(char* name)
{
    int i = fnameSearch(name);
    if (i < 0) return;
    if (i < 32)
        mem.req_flush[0] |= 0b1 << i;
    else
        mem.req_flush[1] |= 0b1 << (i - 32);
    // In the middle of flushing, restart process
    if (bg_state == BG_FLUSH && curr_page == i)
    {
        curr_page = 0;
        addr = 0;
    }
}

/**
 * @brief: Clears all addresses to 0 on chip
 */
static void memClear() {
    mem.zero_req = true;
}

/**
 * @brief: Fill chip with ones and check each location
 *         to ensure writes were complete
 *
 * @return: E_SUCCESS if memory matches, -E_M_MISMATCH
 *
 * @note: DO NOT USE! WILL RESET STRUCTS AND CAUSE DEVICE
 *        NAKS FOLLOWED BY I2C STOPPAGE
 */
static int memTest() {
    size_t  i;
    uint8_t one[MICRO_PG_SIZE];
    uint8_t page[MICRO_PG_SIZE];

    ee_memset(one, MICRO_PG_SIZE, 0xff);

    for (i = 0; i < CHIP_SIZE; i += MICRO_PG_SIZE) {
        writePage(i, one, MICRO_PG_SIZE);
    }

    for (i = 0; i < CHIP_SIZE; i += MICRO_PG_SIZE) {
        readPage(i, page);

        if (ee_memcheck(page, one, MICRO_PG_SIZE) < 0) {
            return -E_M_MISMATCH;
        }
    }

    return E_SUCCESS;
}

/**
* @brief: Find index
*
* @param: dest: Pointer to location to set
* @param: len: Length to set
* @param: value: Value to set each memory address to
*
* @return: Index of name if it exists, -E_NO_NAME else
*/
static int fnameSearch(char* name) {
    uint8_t i;

    // Search for index of name
    for (i = 0; i < MAX_PAGES; i++) {
        if (ee_memcheck(&mem.phys.filename[i * NAME_LEN], name, NAME_LEN) == 0) {
            return i;
        }
    }

    return -E_NO_NAME;
}

/**
 * @brief: Simple memcheck routine
 *
 * @param: src: Pointer to location to check
 * @param: dest: Pointer to second location to check
 * @param: len: Length to check
 *
 * @return: E_SUCCESS if memory matches, -E_M_MISMATCH else
 */
static int ee_memcheck(uint8_t* src, uint8_t* dest, size_t len) {
    size_t i;

    for (i = 0; i < len; i++) {
        if (src[i] != dest[i]) {
            return -E_M_MISMATCH;
        }
    }

    return E_SUCCESS;
}

/**
 * @brief: Simple memset routine
 *
 * @param: dest: Pointer to location to set
 * @param: len: Length to set
 * @param: value: Value to set each memory address to
 */
static void ee_memset(uint8_t* dest, size_t len, uint8_t value) {
    size_t i;

    for (i = 0; i < len; i++) {
        dest[i] = value;
    }
}

/**
 * @brief: Simple memcpy routine
 *
 * @param: src: Pointer to location to copy
 * @param: dest: Pointer to location from which to copy
 * @param: len: Length of data to copy
 */
static void ee_memcpy(uint8_t* src, uint8_t* dest, size_t len) {
    size_t i;

    for (i = 0; i < len; i++) {
        dest[i] = src[i];
    }
}

/**
 * @brief gets index of first set bit, assumes a bit is set
 * 
 */
static uint8_t ee_get_idx(uint32_t *req)
{
    uint8_t idx = 0;
    uint32_t search_vals = req[0];

    if (req[0] == 0)
    {
        search_vals = req[1];
        idx = 32;
    }
    for (uint8_t i = 0; i < 32; ++i)
    {
        if (search_vals & 0b1)
        {
            idx += i;
            break;
        }
        search_vals >>= 1;
    }
    return idx;
}

/**
 * @brief clears bit at index
 */
static void ee_clear_idx(uint32_t *req, uint8_t idx)
{
    if (idx < 32)
        req[0] &= ~(0b1 << idx);
    else
        req[1] &= ~(0b1 << (idx - 32));
}

static void ee_request_flush_physical()
{
    mem.flush_physical = true;
    if (bg_state == BG_FLUSH_META)
    {
        addr = 0;
        curr_page = 0;
    }
}
