#ifndef AGV_COMMUNICATION_PACK__PROTOCOL_ROS_PROTOCOL_H_
#define AGV_COMMUNICATION_PACK__PROTOCOL_ROS_PROTOCOL_H_

#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"

typedef struct {
    const AgvCommPrtclHostCfg* cfg;

    AgvCommMsg pending_msg;
    int has_pending;
} HostProtoImpl;

static int hostProto_feed_frame(AgvCommProtocolIface* iface,
                                const uint8_t* frame, size_t frame_len);

static int hostProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* msg_out);

static int hostProto_make_payload(AgvCommProtocolIface* iface,
                                  const AgvCommMsg* msg_in,
                                  uint8_t* payload_out, size_t* payload_len);

static int hostProto_destroy(AgvCommProtocolIface* iface);

uint8_t* put_f32_le(uint8_t* p, float v);

const uint8_t* get_f32_le(const uint8_t* p, float* out);

#endif  // AGV_COMMUNICATION_PACK__PROTOCOL_ROS_PROTOCOL_H_