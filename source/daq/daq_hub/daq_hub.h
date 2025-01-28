/**
 * @file daq_hub.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Data acquisition from CAN to:
 *          - SD Card
 *          - Ethernet
 *          - USB (maybe)
 * @version 0.1
 * @date 2024-02-08
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _DAQ_HUB_H_
#define _DAQ_HUB_H_

#include <stdint.h>
#include <assert.h>

#include "common/freertos/freertos.h"
#include "can_flags.h"
#include "ff.h"
#include "sdio.h"

typedef uint32_t canid_t;
typedef uint8_t busid_t;

// from DAQ POV
#define DAQ_FRAME_CAN_RX  0  // RX to DAQ over CAN (interrupt), broadcast message
#define DAQ_FRAME_TCP2CAN 1  // RX to DAQ over TCP, relay to other nodes on CAN
#define DAQ_FRAME_TCP2DAQ 2  // RX to DAQ over TCP, process within DAQ (encapsulates CAN msg intended for DAQ)
#define DAQ_FRAME_TCP_TX  3  // TX from DAQ over TCP
#define DAQ_FRAME_UDP_TX  4  // TX from DAQ over UDP

typedef struct __attribute__((packed))
{
    uint8_t  frame_type;   //!< command
    uint32_t tick_ms;      //!< ms timestamp of reception
    canid_t  msg_id;       //!< message id
    busid_t  bus_id;       //!< bus the message was rx'd on
    uint8_t  dlc;          //!< data length code
    uint8_t  data[8];      //!< message data
} timestamped_frame_t;

typedef enum
{
    TCP_CMD_HANDSHAKE = 0,
    TCP_CMD_CAN_FRAME = 1, // authentic CAN frame, i.e. daq can't TX CAN to itself
    TCP_CMD_UDS_FRAME = 2,
} tcp_cmd_t;

typedef enum
{
    SD_STATE_IDLE = 0,
    SD_STATE_MOUNTED,
    SD_STATE_FILE_CREATED,
    SD_FAIL,
    SD_SHUTDOWN,
} sd_state_t;

typedef enum
{
    SD_ERROR_NONE = 0,
    SD_ERROR_MOUNT,
    SD_ERROR_FOPEN,
    SD_ERROR_FCLOSE,
    SD_ERROR_WRITE,
    SD_ERROR_DETEC,
    SD_ERROR_SYNC,
} sd_error_t;

typedef enum __attribute__ ((__packed__))
{
    ETH_IDLE = 0,
    ETH_LINK_DOWN,
    ETH_LINK_UP,
    ETH_FAIL,
} eth_state_t;
static_assert(sizeof(eth_state_t) == sizeof(uint8_t));

typedef enum
{
    ETH_TCP_IDLE = 0,
    ETH_TCP_LISTEN,
    ETH_TCP_ESTABLISHED,
    ETH_TCP_FAIL,
} eth_tcp_state_t;

typedef enum
{
    ETH_ERROR_NONE       = 0,
    ETH_ERROR_INIT       = 1,
    ETH_ERROR_VERS       = 2,
    ETH_ERROR_UDP_SOCK   = 3,
    ETH_ERROR_UDP_SEND   = 4,
    ETH_ERROR_TCP_SOCK   = 5,
    ETH_ERROR_TCP_LISTEN = 6,
    ETH_ERROR_TCP_SEND   = 7,
} eth_error_t;

#define ETH_PHY_RESET_PERIOD_MS 10
#define ETH_PHY_LINK_TIMEOUT_MS 5000

#define SD_NEW_FILE_PERIOD_MS   (1*60*1000) // (2*60*1000) 2 minutes
#define SD_MAX_WRITE_COUNT      (512) // Assuming approx 1kHz  rx rate
#define UDP_MAX_BUFFER_SIZE     (8192)  // Assuming approx 1kHz  rx rate
#define UDP_MAX_WRITE_COUNT     (UDP_MAX_BUFFER_SIZE / (sizeof(timestamped_frame_t)))  // Assuming approx 1kHz  rx rate

typedef struct
{
    // Ethernet
    eth_state_t eth_state;
    eth_tcp_state_t eth_tcp_state;
    bool eth_enable_udp_broadcast; // TODO: determine if I want this var
    bool eth_enable_tcp_reception;
    uint32_t eth_error_ct;
    eth_error_t eth_last_err;
    int32_t eth_last_err_res;
    uint32_t eth_last_error_time;

    // SD Card
    sd_state_t sd_state;
    FATFS fat_fs;
    uint32_t sd_error_ct;
    sd_error_t sd_last_err;
    FRESULT sd_last_err_res;
    uint32_t sd_last_error_time;

    FIL log_fp;
    bool ftp_busy;
    uint32_t log_start_ms;
    uint32_t last_write_ms;
    uint32_t last_file_tick;
    bool log_enable_sw; //!< Debounced switch state
    bool log_enable_tcp;
    bool log_enable_uds;
} daq_hub_t;

extern daq_hub_t dh;

// W5500 has 8 sockets internally
#define DAQ_SOCKET_UDP_BROADCAST 0
#define DAQ_SOCKET_TCP           1
#define DAQ_SOCKET_FTP_CTRL0     2  // FTP uses 3 sockets
#define DAQ_SOCKET_FTP_DATA      3
#define DAQ_SOCKET_FTP_CTRL1     4

void daq_catch_error(void);
void daq_hub_init(void);
void daq_create_threads(void);
void uds_receive_periodic(void);
void shutdown(void);
void daq_shutdown_hook(void);

#endif
