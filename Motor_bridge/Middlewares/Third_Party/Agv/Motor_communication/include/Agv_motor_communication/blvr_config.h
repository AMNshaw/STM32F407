#ifndef AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_
#define AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"

typedef struct {
    size_t axis_count;
    // link
    AgvCommLnkUartCfg uart_cfg;
    // format
    AgvCommFmtModbusRtuCfg modbus_cfg;
    // protocol
    AgvCommPrtclBlvrCfg prtcl_blvr_cfg;
} Agv_Blvr_config;

#endif  // AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_