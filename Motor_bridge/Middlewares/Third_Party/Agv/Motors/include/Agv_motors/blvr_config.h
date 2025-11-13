#ifndef AGV_MOTORS__BLVR_CONFIG_H_
#define AGV_MOTORS__BLVR_CONFIG_H_

#include <stdlib.h>

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"

typedef struct {
    // link
    AgvCommLnkUartRs485Cfg uart_cfg;
    // format
    AgvCommFmtModbusRtuCfg modbus_cfg;
    // protocol
    AgvCommPrtclBlvrCfg prtcl_blvr_cfg;

    size_t axis_count;
    float gear_ratio;
} AgvMotorBlvrConfig;

#endif  // AGV_MOTORS__BLVR_CONFIG_H_