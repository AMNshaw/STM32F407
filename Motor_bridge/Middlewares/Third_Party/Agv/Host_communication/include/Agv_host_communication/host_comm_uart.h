#ifndef AGV_HOST_COMMUNICATION__HOST_COMM_UART_H_
#define AGV_HOST_COMMUNICATION__HOST_COMM_UART_H_

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "stdlib.h"

typedef struct {
    AgvCommLinkIface link;
    AgvCommFormatIface fmt;
    AgvCommProtocolIface prtcl;
} CommHostUartImpl;

static int uart_get_host_cmd(AgvHostCommunicationBase* self, HostMsg* out_msg);
static int uart_send_odom(AgvHostCommunicationBase* self, const Odom* in);

#endif  // AGV_HOST_COMMUNICATION__HOST_COMM_UART_H_