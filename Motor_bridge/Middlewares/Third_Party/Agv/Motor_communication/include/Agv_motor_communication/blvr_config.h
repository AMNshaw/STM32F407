#ifndef AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_
#define AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_

#include <stdlib.h>

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"

typedef struct {
    size_t axis_count;
    // link
    AgvCommLnkUartRs485Cfg uart_cfg;
    // format
    AgvCommFmtModbusRtuCfg modbus_cfg;
    // protocol
    AgvCommPrtclBlvrCfg prtcl_blvr_cfg;
} AgvMotorBlvrConfig;

#endif  // AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_