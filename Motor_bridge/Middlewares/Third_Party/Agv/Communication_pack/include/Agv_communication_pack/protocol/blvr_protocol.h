#ifndef AGV_COMMUNICATION_PACK__PROTOCOL_BLVR_PROTOCOL_H_
#define AGV_COMMUNICATION_PACK__PROTOCOL_BLVR_PROTOCOL_H_

#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "Agv_core/agv_types.h"

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
                                 size_t* inout_len);

static int BlvrProto_destroy(AgvCommProtocolIface* iface);

#endif  // AGV_COMMUNICATION_PACK__PROTOCOL_BLVR_PROTOCOL_H_