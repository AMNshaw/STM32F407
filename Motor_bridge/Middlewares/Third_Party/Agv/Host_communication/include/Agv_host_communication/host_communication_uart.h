#ifndef AGV_HOST_COMMUNICATION__HOST_COMMUNICATION_UART_H_
#define AGV_HOST_COMMUNICATION__HOST_COMMUNICATION_UART_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_core/agv_types.h"
#include "Agv_core/modules/host_communication_base.h"

typedef struct {
    AgvCommLinkIface link;
    AgvCommFormatIface fmt;
    AgvCommProtocolIface prtcl;
} CommHostImpl;

int uart_receive_cmd(AgvHostCommunicationBase* self, VelCmd* out);
int uart_send_odom(AgvHostCommunicationBase* self, const Odom* in);

#endif  // AGV_HOST_COMMUNICATION__HOST_COMMUNICATION_UART_H_