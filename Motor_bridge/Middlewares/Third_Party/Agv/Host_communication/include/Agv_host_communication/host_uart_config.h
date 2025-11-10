#ifndef AGV_HOST_COMMUNICATION__HOST_COMM_UART_H_
#define AGV_HOST_COMMUNICATION__HOST_COMM_UART_H_

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "stdlib.h"

typedef struct {
    // link
    AgvCommLnkUartCfg uart_cfg;
    // format
    AgvCommFmtRosCfg rosFmt_cfg;
    // protocol
    AgvCommPrtclHostCfg prtcl_host_cfg;
} AgvHostUartCfg;

#endif  // AGV_HOST_COMMUNICATION__HOST_COMM_UART_H_