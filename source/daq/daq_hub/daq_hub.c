/**
 * @file daq_hub.c
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

#include <string.h>

#include "common/modules/Wiznet/W5500/Ethernet/wizchip_conf.h"
#include "common/modules/Wiznet/W5500/Ethernet/socket.h"
#include "common/phal_F4_F7/gpio/gpio.h"

#include "buffer.h"
#include "main.h"
#include "daq_can.h"
#include "daq_hub.h"
#include "daq_sd.h"
#include "ftpd.h"

typedef struct
{
    wiz_NetInfo net_info;
    uint8_t udp_bc_addr[4];
    uint16_t udp_bc_port;
    uint8_t udp_bc_sock;
    uint16_t tcp_port;
    uint8_t tcp_sock;
} eth_config_t;

eth_config_t eth_config = {
    .net_info = {
        .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
        .ip 	= {192, 168, 10, 41},					// IP address of DAQ PCB
        .sn 	= {255, 255, 255, 0},					// Subnet mask
        .gw 	= {192, 168, 10, 1}	     				// Gateway address
    },
    .udp_bc_addr = {192, 168, 10, 255},                 // Broadcast address
    .udp_bc_port = 5005,
    .udp_bc_sock = DAQ_SOCKET_UDP_BROADCAST,
    .tcp_port = 5005,
    .tcp_sock = DAQ_SOCKET_TCP,
};

daq_hub_t dh;

// Local protoptypes
static void daq_heartbeat(void);
static void can_send_periodic(void);
static void can_relay_can2_periodic(void);

static int8_t eth_init(void);
static int8_t eth_get_link_up(void);
static void eth_update_connection_state(void);
static bool eth_get_tcp_connected(void);
static int8_t eth_init_udp_broadcast(void);
static void eth_send_udp_periodic(void);
static void eth_send_tcp_periodic(void);
static void eth_receive_tcp_periodic(void);
static void eth_update_tcp_connection_state(void);
static void eth_check_fail(void);

uint8_t gFTPBUF[_FTP_BUF_SIZE];
static void ftp_server_run(void);

static void conv_tcp_frame_to_can_msg(timestamped_frame_t *t, CanMsgTypeDef_t *c);
static void conv_can_msg_to_tcp_frame(CanMsgTypeDef_t *c, timestamped_frame_t *t);

defineThreadStack(daq_heartbeat, 500, osPriorityNormal, 128); // HB
//defineThread(can_relay_can2_periodic, 100, osPriorityNormal); // CAN2
defineThreadStack(uds_receive_periodic, 50, osPriorityHigh, 2048); // DCAN RX
defineThreadStack(can_send_periodic, 50, osPriorityNormal, 128); // CAN1 TX
//defineThreadStack(ftp_server_run, 500, osPriorityNormal, 512); // FTP
defineThreadStack(sd_write_periodic, 100, osPriorityNormal, 2048); // SD WRITE
defineThreadStack(sd_create_file_periodic, 100, osPriorityNormal, 2048); // SD FILE
defineThreadStack(sd_update_connection_state, 100, osPriorityNormal, 2048); // SD STATE
defineThreadStack(eth_receive_tcp_periodic, 100, osPriorityNormal, 512); // TCP RX
defineThreadStack(eth_update_tcp_connection_state, 100, osPriorityNormal, 1024); // TCP STATE
defineThreadStack(eth_send_udp_periodic, 100, osPriorityNormal, 512); // UDP TX
defineThreadStack(eth_send_tcp_periodic, 100, osPriorityNormal, 512); // UDP TX
defineThreadStack(eth_update_connection_state, 100, osPriorityNormal, 1024); // UDP STATE
defineThreadStack(eth_check_fail, 500, osPriorityNormal, 64); // UDP STATE

void daq_catch_error(void)
{
    PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 1);
    while(1)
    {
        __asm__("bkpt");
        __asm__("nop");
    }
}

void daq_hub_init(void)
{
    // Ethernet
    dh.eth_state = ETH_IDLE;
    dh.eth_tcp_state = ETH_TCP_IDLE;
    dh.eth_enable_udp_broadcast = true;
    dh.eth_enable_tcp_reception = true;
    dh.eth_error_ct = 0;
    dh.eth_last_error_time = 0;
    dh.eth_last_err = ETH_ERROR_NONE;

    // SD Card
    dh.sd_state = SD_STATE_IDLE;
    dh.sd_error_ct = 0;
    dh.sd_last_error_time = 0;
    dh.sd_last_err = SD_ERROR_NONE;
    dh.sd_last_err_res = 0;

    dh.last_file_tick = 0;
    dh.log_enable_sw = false;
    dh.log_enable_tcp = false;
    dh.log_enable_uds = false;
    dh.ftp_busy = false;
}

void daq_create_threads(void)
{
    createThread(daq_heartbeat); // HB
    //createThread(can_relay_can2_periodic); // CAN2
    createThread(uds_receive_periodic); // DCAN RX
    createThread(can_send_periodic); // CAN1 TX
    //createThread(ftp_server_run); // FTP
    createThread(sd_write_periodic); // SD WRITE
    createThread(eth_send_tcp_periodic); // UDP TX
    createThread(eth_send_udp_periodic); // UDP TX
    createThread(sd_create_file_periodic); // SD FILE
    createThread(eth_receive_tcp_periodic); // TCP RX
    createThread(eth_update_tcp_connection_state); // TCP STATE
    createThread(eth_update_connection_state); // UDP STATE
    createThread(sd_update_connection_state); // SD STATE
    createThread(eth_check_fail); // UDP+TCP STATE
}

static void daq_heartbeat(void)
{
    // SEND_DAQ_CAN_STATS(can_stats[0].tx_of, can_stats[0].tx_fail, can_stats[0].rx_of, can_stats[0].rx_overrun);
    PHAL_toggleGPIO(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN);
    #if 0
    static uint32_t last = 0;
    static uint32_t last_count = 0;
    //debug_printf("count: sd: %4d udp: %2d | tail: sd: %4d udp: %4d\n", bGetItemCount(&b_rx_can, RX_TAIL_SD), bGetItemCount(&b_rx_can, RX_TAIL_UDP), b_rx_can.tails[RX_TAIL_SD]._tail, b_rx_can.tails[RX_TAIL_UDP]._tail);
    float busload = 160 * ((uint32_t)can_hit_count - last_count) * 2 / 500000.0; // (float)(getTick() - last)
    uint32_t load = busload * 100;
    debug_printf("can hit count: %3d msgs: %2d/s load: %3d%%\n", (uint32_t)can_hit_count, (uint32_t)can_hit_count - last_count, load);
    last = getTick();
    last_count = (uint32_t)can_hit_count;
    #endif
}

static void ftp_server_run(void)
{
    static uint8_t ftp_up = 0;
    uint32_t ulNotifiedBits = 0, ulNotifiedValue = 0;
    if (!ftp_up)
    {
        while (ulNotifiedBits != 3) // bitmask: 1 | 2
        {
            xTaskNotifyWait(0, 3, &ulNotifiedValue, (TickType_t)portMAX_DELAY);
            ulNotifiedBits |= ulNotifiedValue; // Wait for both tasks (eth link up && sd mount)
        }
        ftpd_init(eth_config.net_info.ip);
        debug_printf("FTP UP!\n");
    }
    ftp_up = 1;
    ftpd_run(gFTPBUF);
}

static void can_send_periodic(void)
{
    canTxUpdate();
}

static void can_relay_can2_periodic(void)
{
    CanMsgTypeDef_t msg;
    if (xQueueReceive(q_can2_tx, &msg, (TickType_t )10) == pdPASS)
    {
        canTxSendToBack(&msg); // TODO figure out CAN2 path
    }
}

static void _eth_handle_error(eth_error_t err, int32_t reason)
{
    ++dh.eth_error_ct;
    dh.eth_last_err = err;
    dh.sd_last_err_res = reason;
    dh.eth_last_error_time = getTick();
    PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 1);
    debug_printf("eth err: %d res: %d\n", err, reason);
}

#define eth_handle_error_reason(err, reason) _eth_handle_error(err, reason)
#define eth_handle_error(err) eth_handle_error_reason(err, 0)

static void eth_check_fail(void)
{
    if (dh.eth_state == ETH_FAIL)
    {
        // Do not re-attempt (immediately), something is very wrong
        PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 1);
        dh.eth_state = ETH_IDLE; // Retry
    }
    if (dh.eth_tcp_state == ETH_TCP_FAIL)
    {
        PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 1);
        dh.eth_tcp_state = ETH_TCP_IDLE; // Retry
    }
    else
    {
        PHAL_writeGPIO(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, 0);
    }

    if (dh.eth_state == ETH_LINK_UP)
    {
        if (dh.eth_tcp_state == ETH_TCP_ESTABLISHED || dh.ftp_busy)
        {
            PHAL_toggleGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN); // Blink on TCP
        }
        else
        {
            PHAL_writeGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN, 1);
        }
    }
    else
    {
        PHAL_writeGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN, 0);
    }
}

static void eth_update_connection_state(void)
{
    static uint8_t eth_startup = 0;
    if (!eth_startup)
    {
        mDelay(250); // Wait for module to kick up
        eth_startup = 1;
    }

    if (!(dh.eth_enable_udp_broadcast || dh.eth_enable_tcp_reception)) return;

    switch (dh.eth_state)
    {
        case ETH_IDLE:
            if (eth_init() == ETH_ERROR_NONE)
            {
                dh.eth_state = ETH_LINK_DOWN;
                if (dh.eth_enable_udp_broadcast)
                {
                    // Chip initialized, setup UDP socket
                    if (eth_init_udp_broadcast() != ETH_ERROR_NONE)
                        dh.eth_state = ETH_FAIL;
                }
            }
            else
            {
                dh.eth_state = ETH_FAIL;
            }
            break;
        case ETH_LINK_UP:  // intentional fall through
        case ETH_LINK_DOWN:
            if (eth_get_link_up())
            {
                PHAL_writeGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN, 1);
                if (dh.eth_state != ETH_LINK_UP) // once, only on down -> up
                {
                    debug_printf("UDP UP!\n");
                    bActivateTail(&b_rx_can, RX_TAIL_UDP);
                    xTaskNotify(getTaskHandle(eth_send_udp_periodic), 0, eNoAction);
                    dh.eth_state = ETH_LINK_UP;
                    xTaskNotify(getTaskHandle(eth_update_tcp_connection_state), 0, eNoAction);
                    //xTaskNotify(getTaskHandle(ftp_server_run), (1<<1), eSetBits); // set 1st bit HIGH
                }
                dh.eth_state = ETH_LINK_UP;
            }
            else
            {
                dh.eth_state = ETH_LINK_DOWN;
                PHAL_writeGPIO(CONNECTION_LED_PORT, CONNECTION_LED_PIN, 0);
                bDeactivateTail(&b_rx_can, RX_TAIL_UDP);
            }
            break;
    }
}

static int8_t eth_init(void)
{
    PHAL_writeGPIO(ETH_RST_PORT, ETH_RST_PIN, 0);
    osDelay(ETH_PHY_RESET_PERIOD_MS);
    PHAL_writeGPIO(ETH_RST_PORT, ETH_RST_PIN, 1);
    osDelay(ETH_PHY_RESET_PERIOD_MS); // datasheet says 50 ms

    // Check for sign of life
    if (getVERSIONR() != 0x04)
    {
        eth_handle_error(ETH_ERROR_VERS);
        return ETH_ERROR_VERS;
    }

    // W5500 has 16kB memory for RX/TX each internally. It doesnt matter how the block is distributed
    // across its 8 internal sockets, so allocate all 16kB to the sockets we actually use.
    // 0: UDP broadcast (don't need RX buffer, need big TX)
    // 1: TCP/UDS (need big RX buffer for bootloader, only need small TX for ack messages)
    // 2/3/4: FTP
    uint8_t rxBufSizes[] = {8, 8, 0, 0, 0, 0, 0, 0}; // kB
    uint8_t txBufSizes[] = {8, 8, 0, 0, 0, 0, 0, 0}; // kB

    if (wizchip_init(txBufSizes, rxBufSizes))
    {
        eth_handle_error(ETH_ERROR_INIT);
        return ETH_ERROR_INIT;
    }

    wizchip_setnetinfo(&eth_config.net_info);

    return ETH_ERROR_NONE;
}

static int8_t eth_get_link_up(void)
{
    volatile uint8_t phycfgr = getPHYCFGR();
    return phycfgr & PHY_LINK_ON;
}

static int8_t eth_init_udp_broadcast(void)
{
    // UDP Broadcast address
    // 255.255.255.255 is not forwarded by routers
    // local address | (~subnet address) is forwarded
    int8_t bc_sock;
    bc_sock = socket(eth_config.udp_bc_sock, SOCK_DGRAM, eth_config.udp_bc_port, 0);
    if (bc_sock != eth_config.udp_bc_sock)
    {
        eth_handle_error(ETH_ERROR_UDP_SOCK);
        return -ETH_ERROR_UDP_SOCK;
    }

    return ETH_ERROR_NONE;
}

static void eth_send_udp_periodic(void)
{
    int32_t ret;
    timestamped_frame_t *buf;
    timestamped_frame_t *frame;
    uint32_t consecutive_items;

    if (dh.eth_state != ETH_LINK_UP)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    if (bGetTailForRead(&b_rx_can, RX_TAIL_UDP, (void**) &buf, &consecutive_items) == 0)
    {
        // Write time :D
        ret = sendto(eth_config.udp_bc_sock, (uint8_t *)buf,
                     consecutive_items * sizeof(*buf),
                     eth_config.udp_bc_addr,
                     eth_config.udp_bc_port);
        if (ret < consecutive_items * sizeof(*buf))
        {
            eth_handle_error_reason(ETH_ERROR_UDP_SEND, ret);
        }
        else
        {
            bCommitRead(&b_rx_can, RX_TAIL_UDP, consecutive_items);
        }
    }
}

static int8_t eth_init_tcp(void)
{
    // Open socket
    int8_t tcp_sock;
    tcp_sock = socket(eth_config.tcp_sock, SOCK_STREAM, eth_config.tcp_port, 0);
    if (tcp_sock != eth_config.tcp_sock)
    {
        eth_handle_error_reason(ETH_ERROR_TCP_SOCK, tcp_sock);
        return -ETH_ERROR_TCP_SOCK;
    }

    // Set to non-blocking
    uint8_t io_mode = SOCK_IO_NONBLOCK;
    ctlsocket(eth_config.tcp_sock, CS_SET_IOMODE, &io_mode);

    // Set socket to listen -> verify by checking SR went to ESTABLISHED
    if (listen(eth_config.tcp_sock) != SOCK_OK)
    {
        close(eth_config.tcp_sock);
        eth_handle_error(ETH_ERROR_TCP_LISTEN);
        return -ETH_ERROR_TCP_LISTEN;
    }

    return ETH_ERROR_NONE;
}

static void eth_update_tcp_connection_state(void)
{
    if (dh.eth_state != ETH_LINK_UP)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    uint8_t stat;
    switch (dh.eth_tcp_state)
    {
        case ETH_TCP_IDLE:
            if (eth_init_tcp() == ETH_ERROR_NONE)
                dh.eth_tcp_state = ETH_TCP_LISTEN;
            else
                dh.eth_tcp_state = ETH_TCP_FAIL;
        break;
        case ETH_TCP_LISTEN:
            stat = getSn_SR(eth_config.tcp_sock);
            if (stat == SOCK_ESTABLISHED)
            {
                debug_printf("TCP UP!\n");
                dh.eth_tcp_state = ETH_TCP_ESTABLISHED; // Connected
            }
        break;
        case ETH_TCP_ESTABLISHED:
            stat = getSn_SR(eth_config.tcp_sock);
            if (stat == SOCK_CLOSE_WAIT)
            {
                close(eth_config.tcp_sock);
                dh.eth_tcp_state = ETH_TCP_IDLE;
            }
        break;
    }

    if (dh.eth_tcp_state == ETH_TCP_ESTABLISHED)
    {
        xTaskNotify(getTaskHandle(eth_receive_tcp_periodic), 0, eNoAction);
        xTaskNotify(getTaskHandle(eth_send_tcp_periodic), 0, eNoAction);
    }
}

static bool eth_get_tcp_connected(void)
{
    return dh.eth_state == ETH_LINK_UP && dh.eth_enable_tcp_reception && dh.eth_tcp_state == ETH_TCP_ESTABLISHED;
}

static void conv_tcp_frame_to_can_msg(timestamped_frame_t *t, CanMsgTypeDef_t *c)
{
    c->Bus = t->bus_id == BUS_ID_CAN1 ? CAN1 : CAN2;

    if (t->msg_id & CAN_EFF_FLAG)
    {
        c->IDE = 1;
        c->ExtId = t->msg_id & CAN_EFF_MASK;
    }
    else
    {
        c->IDE = 0;
        c->StdId = t->msg_id & CAN_SFF_MASK;
    }

    c->DLC = t->dlc;
    for (uint8_t i = 0; i < 8; ++i) c->Data[i] = t->data[i];
}

static void _eth_tcp_send_frame(timestamped_frame_t *frame)
{
    int32_t ret; // TODO error handle
    if (dh.eth_tcp_state == ETH_TCP_ESTABLISHED)
    {
        frame->frame_type = DAQ_FRAME_TCP_TX;
        ret = send(eth_config.tcp_sock, (uint8_t *)frame, sizeof(*frame));
        if (ret != sizeof(*frame))
        {
            eth_handle_error_reason(ETH_ERROR_TCP_SEND, ret);
            daq_catch_error();
        }
    }
}

static void eth_send_tcp_periodic(void)
{
    if (dh.eth_state != ETH_LINK_UP)
    {
        // No reason to check if notification came from CAN or TCP
        // We relay back on CAN unconditionally, and TCP iff connection is established
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    timestamped_frame_t frame;
    if (xQueueReceive(q_tcp_tx, &frame, (TickType_t )50) == pdPASS)
    {
        _eth_tcp_send_frame(&frame);
    }
}

static void eth_udp_send_frame(timestamped_frame_t *frame)
{
    timestamped_frame_t *rx; // TODO check if this is safe (two producers)
    uint32_t cont;
    if (bGetHeadForWrite(&b_rx_can, (void**) &rx, &cont) == 0)
    {
        memcpy(rx, frame, sizeof(*frame));
        rx->frame_type = DAQ_FRAME_UDP_TX;
        bCommitWrite(&b_rx_can, 1); // simply add it to regular CAN RX queue that DAQ broadcasts
    }
}

static void eth_tcp_send_frame(timestamped_frame_t *frame)
{
    if (dh.eth_tcp_state == ETH_TCP_ESTABLISHED) // only send UDS response back if TCP established
    {
        if (xQueueSendToBack(q_tcp_tx, frame, (TickType_t)10) != pdPASS)
        {
            daq_catch_error();
        }
    }
}

void uds_frame_send(uint64_t data)
{
#if 0
    timestamped_frame_t frame = {.frame_type = DAQ_FRAME_UDP_TX, .tick_ms = getTick(), .msg_id = ID_UDS_RESPONSE_DAQ, .bus_id = BUS_ID_CAN1, .dlc = 8 };
    frame.msg_id |= CAN_EFF_FLAG;
    memcpy(frame.data, (uint8_t *)&data, sizeof(uint64_t));

    SEND_UDS_RESPONSE_DAQ(data);
    eth_tcp_send_frame(&frame);
    //eth_udp_send_frame(&frame); // dont send for now
#endif
}

/**
 * Pull UDS CAN frames out of UDS queue added during CAN1/CAN2 ISR
 * and process them in non-interrupt context
 */
void uds_receive_periodic(void)
{
#if 0
    timestamped_frame_t rx_msg;
    while (xQueueReceive(q_can1_rx, &rx_msg, portMAX_DELAY) == pdPASS)
    {
        CanParsedData_t *msg_data_a = (CanParsedData_t *) &rx_msg.data;
        uds_command_daq_CALLBACK(msg_data_a->uds_command_daq.payload);
    }
#endif
}

// Pull out of TCP queue and add it to CAN TX queue
static void eth_tcp_relay_can_frame(timestamped_frame_t *t)
{
    CanMsgTypeDef_t msg;
    conv_tcp_frame_to_can_msg(t, &msg);
    canTxSendToBack(&msg);
}

// Pull out of TCP queue and add it to UDS queue
static void eth_tcp_relay_uds_frame(timestamped_frame_t *t)
{
    if (xQueueSendToBack(q_can1_rx, t, (TickType_t)10) != pdPASS)
    {
        daq_catch_error();
    }
}

/**
 * Pull frames out of TCP RX queue (from PC) and process them by the prefixed command ID
 * If TCP_CMD_CAN_FRAME, put it on CAN for other nodes
 * If TCP_CMD_UDS_FRAME, it's a CAN message intended for DAQ (i.e. UDS) so add it to UDS queue
 */
static void eth_receive_tcp_periodic(void)
{
    if (dh.eth_tcp_state != ETH_TCP_ESTABLISHED)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    timestamped_frame_t *frame;
    int32_t ret = recv(eth_config.tcp_sock, (uint8_t *)tcp_rx_buf,
                        TCP_RX_ITEM_COUNT * sizeof(timestamped_frame_t)); // TODO check ret
    if (ret > 0)
    {
        uint32_t count = (uint32_t)ret / sizeof(timestamped_frame_t);
        for (uint32_t i = 0; i < count; i++)
        {
            frame = &tcp_rx_buf[i];
            switch(frame->frame_type)
            {
                case DAQ_FRAME_TCP2CAN:
                    eth_tcp_relay_can_frame(frame);
                    break;
                case DAQ_FRAME_TCP2DAQ:
                    eth_tcp_relay_uds_frame(frame);
                    break;
                default:
                    break;
            }
        }
    }
    else if (ret == SOCKERR_SOCKSTATUS)
    {
        dh.eth_tcp_state = ETH_TCP_IDLE; // Go back to the start of state machine
        // recv() API already calls close() on socket for us (part of FIN/ACK)
        // So no need to close here
    }
}

void daq_shutdown_hook(void)
{
    // First, turn off all power consuming devices to increase our write time to sd card
    // To whoever is doing future DAQ rev: also change the GPIO ports
    GPIOD->BSRR |= (1 << ((1 << 4) | HEARTBEAT_LED_PIN)) | (1 << ((1 << 4) | CONNECTION_LED_PIN)) | (1 << ((1 << 4) | ERROR_LED_PIN)); // all LEDs go bye bye
    GPIOA->BSRR |= (1 << ((1 << 4) | SD_ACTIVITY_LED_PIN)) | (1 << ((1 << 4) | SD_ERROR_LED_PIN)) | (1 << ((1 << 4) | SD_DETECT_LED_PIN));

    PHAL_writeGPIO(ETH_RST_PORT, ETH_RST_PIN, 0);
    PHAL_deinitCAN(CAN1);
    PHAL_deinitCAN(CAN2);

    f_close(&dh.log_fp);   // Close file
    f_mount(0, "", 1);     // Unmount drive
    SD_DeInit();           // Shutdown SDIO peripheral

    // Hooray, we made it, blink an LED to show the world
    PHAL_writeGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, 1);
}

/**
 * @brief Disables high power consumption devices
 *        If file open, flushes it to the sd card
 *        Then unmounts sd card
 */
void shutdown(void)
{
    daq_shutdown_hook();
    uint32_t start_tick = getTick();
    while (getTick() - start_tick < 3000 || PHAL_readGPIO(PWR_LOSS_PORT, PWR_LOSS_PIN) == 0) // wait for power to fully turn off -> if it does not, restart
    {
        //if (getTick() % 250 == 0) PHAL_toggleGPIO(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN);
    }
    NVIC_SystemReset(); // oof, we assumed wrong, restart and resume execution since the power is still on!
}

// Interrupt handler for power loss detection
// Note: this is set to lowest priority to allow preemption by other interrupts
void EXTI15_10_IRQHandler()
{
    if (EXTI->PR & EXTI_PR_PR15)
    {
        EXTI->PR |= EXTI_PR_PR15; // Clear interrupt
        shutdown();
    }
}
