#ifndef __PDS_H__
#define __PDS_H__

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28      : CAN identifier (11/29 bit)
 * bit 29        : error frame flag (0 = data frame, 1 = error frame)
 * bit 30        : remote transmission request flag (1 = rtr frame)
 * bit 31        : frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef uint32_t canid_t;

/* End CAN Definitions from <linux/can.h> */

// 0 = CAN1, 1 = CAN2,
#define BUS_ID_CAN1 0
#define BUS_ID_CAN2 1
// TODO: add like UDP, USB, etc. ?
typedef uint8_t busid_t;

typedef struct __attribute__((packed))
{
    uint32_t tick_ms;      //!< ms timestamp of reception
    canid_t  msg_id;       //!< message id
    busid_t  bus_id;       //!< bus the message was rx'd on
    uint8_t  dlc;          //!< data length code
    uint8_t  data[8];      //!< message data
} timestamped_frame_t;


#endif // __PDS_H__
