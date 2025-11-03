#ifndef AGV_COMMUNICATION__COMMUNICATION_HOST_H_
#define AGV_COMMUNICATION__COMMUNICATION_HOST_H_

#include "communication_base.h"
#include "communication_iface.h"
#include "types.h"

typedef struct {
    AgvCommLinkIface link_;
    AgvCommFormatIface fmt_;
    AgvCommProtocolIface prtcl_;
} Host_comm_impl;

void receive_cmd(AgvCommunicationBase* self);
void send_odom(AgvCommunicationBase* self);

#endif  // AGV_COMMUNICATION__COMMUNICATION_HOST_H_