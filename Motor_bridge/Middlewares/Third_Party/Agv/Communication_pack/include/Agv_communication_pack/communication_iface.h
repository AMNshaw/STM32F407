#ifndef AGV_COMMUNICATION_PACK__COMMUNICATION_IFACE_H_
#define AGV_COMMUNICATION_PACK__COMMUNICATION_IFACE_H_

#include <stddef.h>
#include <stdint.h>

#include "Agv_communication_pack/communication_msgs.h"

/**
 * Link
 */

typedef struct AgvCommLinkIface {
    int (*send_bytes)(struct AgvCommLinkIface* iface, const uint8_t* data,
                      size_t data_len);
    int (*recv_bytes)(struct AgvCommLinkIface* iface, uint8_t* data,
                      size_t data_len);
    int (*read_buf)(struct AgvCommLinkIface* iface, uint8_t* buf_out,
                    size_t* buf_len, uint32_t* timestamp_out);
    int (*destroy)(struct AgvCommLinkIface* iface);
    void* impl;
} AgvCommLinkIface;

/**
 * Format
 */

typedef struct AgvCommFormatIface {
    int (*feed_bytes)(struct AgvCommFormatIface* iface, const uint8_t* bytes_in,
                      size_t bytes_len);
    int (*pop_payload)(struct AgvCommFormatIface* iface, uint8_t* payload_out,
                       size_t* payload_len);
    int (*make_frame)(struct AgvCommFormatIface* iface,
                      const uint8_t* payload_in, size_t payload_len,
                      uint8_t* frame_out, size_t* frame_len);
    int (*destroy)(struct AgvCommFormatIface* iface);
    void* impl;
} AgvCommFormatIface;

/**
 * Protocol
 */

typedef struct AgvCommProtocolIface {
    int (*feed_payload)(struct AgvCommProtocolIface* iface,
                        const uint8_t* payload_in, size_t payload_len);
    int (*pop_msg)(struct AgvCommProtocolIface* iface, AgvCommMsg* msg_out);
    int (*make_payload)(struct AgvCommProtocolIface* iface,
                        const AgvCommMsg* msg, uint8_t* payload_out,
                        size_t* payload_len);
    int (*destroy)(struct AgvCommProtocolIface* iface);
    void* impl;
} AgvCommProtocolIface;

#endif  // AGV_COMMUNICATION_PACK__COMMUNICATION_IFACE_H_