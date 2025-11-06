#ifndef AGV_COMMUNICATION_PACK__PROTOCOL_ROS_PROTOCOL_H_
#define AGV_COMMUNICATION_PACK__PROTOCOL_ROS_PROTOCOL_H_

#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/communication_msg.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"

typedef struct {
    const AgvCommPrtclHostCfg* cfg;
    AgvCommMsg last_msg;
    int has_msg;
} HostProtoImpl;

static int hostProto_feed_frame(AgvCommProtocolIface* iface,
                                const uint8_t* frame, size_t len);

static int hostProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* out);

static int hostProto_build_frame(AgvCommProtocolIface* iface,
                                 const AgvCommMsg* msg, uint8_t* out_frame,
                                 size_t* inout_len);

#endif  // AGV_COMMUNICATION_PACK__PROTOCOL_ROS_PROTOCOL_H_