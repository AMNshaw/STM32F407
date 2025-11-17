#ifndef AGV_HOST_COMMUNICATION__ROS_HOST_CFG_H_
#define AGV_HOST_COMMUNICATION__ROS_HOST_CFG_H_

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "stdlib.h"

typedef struct {
    // link
    AgvCommLnkUartTtlCfg uart_cfg;
    // format
    AgvCommFmtRosCfg rosFmt_cfg;
    // protocol
    AgvCommPrtclHostCfg prtcl_host_cfg;

    uint32_t data_expiration_ticks;
    uint32_t cmd_vel_timeout_ticks;

} AgvHostRosCfg;

#endif  // AGV_HOST_COMMUNICATION__ROS_HOST_CFG_H_