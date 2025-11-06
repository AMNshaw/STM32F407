#ifndef AGV_MOTOR_COMMUNICATION__BLVR_H_
#define AGV_MOTOR_COMMUNICATION__BLVR_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_core/modules/motor_communication_base.h"

typedef struct {
    AgvCommLinkIface link;
    AgvCommFormatIface fmt;
    AgvCommProtocolIface prtcl;

} BlvrMotorImpl;

int blvr_read(AgvMotorCommunicationBase* self, WheelVel* out);
int blvr_write(AgvMotorCommunicationBase* self, const WheelVel* in);

#endif  // AGV_MOTOR_COMMUNICATION__BLVR_H_