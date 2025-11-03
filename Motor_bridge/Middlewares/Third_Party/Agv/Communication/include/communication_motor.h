#ifndef AGV_COMMUNICATION__COMMUNICATION_MOTOR_H_
#define AGV_COMMUNICATION__COMMUNICATION_MOTOR_H_

#include "communication_base.h"
#include "communication_iface.h"
#include "types.h"

typedef struct {
    AgvCommLinkIface link_;
    AgvCommFormatIface fmt_;
    AgvCommProtocolIface prtcl_;
} Motor_comm_impl;

void receive_encoder(AgvCommunicationBase* self);
void send_ctrl(AgvCommunicationBase* self);

#endif  // AGV_COMMUNICATION__COMMUNICATION_MOTOR_H_