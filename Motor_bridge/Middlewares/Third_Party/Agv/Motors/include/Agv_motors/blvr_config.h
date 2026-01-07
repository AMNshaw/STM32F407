#ifndef AGV_MOTORS__BLVR_CONFIG_H_
#define AGV_MOTORS__BLVR_CONFIG_H_

#include <stdlib.h>

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "stm32f407xx.h"

typedef struct {
    // link
    AgvCommLnkUartRs485Cfg uart_cfg;
    // format
    AgvCommFmtModbusRtuCfg modbus_cfg;
    // protocol
    AgvCommPrtclBlvrCfg prtcl_blvr_cfg;

    size_t axis_count;

    GPIO_TypeDef* io_hwto_reset_port;
    uint16_t io_hwto_reset_pin;

    float motor_acc;
    float motor_dec;

    float unit_step_degree;
    float unit_vel_rpm;

    float gearRatio_motor_to_wheel;
} AgvMotorBlvrConfig;

#endif  // AGV_MOTORS__BLVR_CONFIG_H_