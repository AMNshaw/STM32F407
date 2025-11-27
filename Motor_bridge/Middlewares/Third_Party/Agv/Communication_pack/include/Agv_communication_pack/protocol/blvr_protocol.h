#ifndef AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_H_
#define AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/msg/blvr_motor_msgs.h"

int BlvrProto_get_response_frame_len(AgvCommProtocolIface* iface,
                                     const BlvrMsg* request, size_t* frame_len);

#endif  // AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_H_