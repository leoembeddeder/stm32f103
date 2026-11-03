/**
 * @file can_frame.h
 * @brief CAN Frame Data Structure header forwarding to can_frame_fd.h
 */

#ifndef CAN_FRAME_H
#define CAN_FRAME_H

#include <stdint.h>
#include <stdbool.h>

#define CAN_FD_MAX_DLEN 8U//64U

typedef struct {
    uint32_t id;                       /**< 11-bit standard or 29-bit extended CAN identifier */
    uint8_t  dlc;                      /**< Payload length in bytes (0..64) */
    uint8_t  data[CAN_FD_MAX_DLEN];    /**< Payload data bytes */
    bool     is_ext;                   /**< True if 29-bit extended ID */
    bool     is_rtr;                   /**< True if Remote Transmission Request */
    bool     is_fd;                    /**< True if CAN-FD frame (allows DLC > 8) */
    bool     brs;                      /**< Bit Rate Switch flag */
} can_frame_t;


#endif /* CAN_FRAME_H */
