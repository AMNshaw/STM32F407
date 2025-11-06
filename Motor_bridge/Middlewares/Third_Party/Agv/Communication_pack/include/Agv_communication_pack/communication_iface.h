#ifndef AGV_COMMUNICATION_PACK__COMMUNICATION_IFACE_H_
#define AGV_COMMUNICATION_PACK__COMMUNICATION_IFACE_H_

#include <stddef.h>
#include <stdint.h>

#include "Agv_core/agv_types.h"

/**
 * Link
 */

typedef struct AgvCommLinkIface {
    int (*send_bytes)(struct AgvCommLinkIface* iface, const uint8_t* data,
                      size_t len);
    int (*recv_bytes)(struct AgvCommLinkIface* iface, uint8_t* buf, size_t len);
    int (*on_buf_rcv)(struct AgvCommLinkIface* iface, uint8_t* buf, size_t len);
    int (*read_buf)(struct AgvCommLinkIface* iface, uint8_t* out_buf,
                    size_t max_out_size);
    int (*destroy)(struct AgvCommLinkIface* iface);
    void* impl;
} AgvCommLinkIface;

/**
 * Format
 */

typedef struct AgvCommFormatIface {
    int (*feed)(struct AgvCommFormatIface*, const uint8_t* bytes, size_t n);
    int (*pop_frame)(struct AgvCommFormatIface*, uint8_t* out,
                     size_t* inout_len);
    int (*make_frame)(struct AgvCommFormatIface*, const uint8_t* payload,
                      size_t len, uint8_t* out, size_t* inout_len);
    int (*destroy)(struct AgvCommFormatIface*);
    void* impl;
} AgvCommFormatIface;

/**
 * Protocol
 */

typedef struct AgvCommProtocolIface {
    int (*feed_frame)(struct AgvCommProtocolIface*, const uint8_t* frame,
                      size_t len);
    int (*pop_msg)(struct AgvCommProtocolIface*, AgvCommMsg* out);
    int (*build_frame)(struct AgvCommProtocolIface*, const AgvCommMsg* msg,
                       uint8_t* out_frame, size_t* inout_len);
    int (*destroy)(struct AgvCommProtocolIface*);
    void* impl;
} AgvCommProtocolIface;

#endif  // AGV_COMMUNICATION_PACK__COMMUNICATION_IFACE_H_