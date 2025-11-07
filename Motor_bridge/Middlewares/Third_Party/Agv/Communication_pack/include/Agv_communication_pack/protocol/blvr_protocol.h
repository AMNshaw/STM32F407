#ifndef AGV_COMMUNICATION_PACK__PROTOCOL_BLVR_PROTOCOL_H_
#define AGV_COMMUNICATION_PACK__PROTOCOL_BLVR_PROTOCOL_H_

#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"

typedef struct {
    const AgvCommPrtclBlvrCfg* cfg;

    AgvCommMsg pending_msg;
    int has_pending;

} BlvrPrtclImpl;

static int BlvrProto_feed_frame(AgvCommProtocolIface* iface,
                                const uint8_t* frame, size_t len);

static int BlvrProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* out);

static int BlvrProto_build_frame(AgvCommProtocolIface* iface,
                                 const AgvCommMsg* msg, uint8_t* out_frame,
                                 size_t max_out_len);

static int BlvrProto_destroy(AgvCommProtocolIface* iface);

static uint8_t* put_be32(uint8_t* p, int32_t v);

static const uint8_t* get_be32(const uint8_t* p, int32_t* out);

#endif  // AGV_COMMUNICATION_PACK__PROTOCOL_BLVR_PROTOCOL_H_