#ifndef AGV_MODULE_FACTORY__MOTOR_BUILDER_H_
#define AGV_MODULE_FACTORY__MOTOR_BUILDER_H_

#include "configs/communication_config.h"
#include "modules/motor_base.h"

int Motor_blvr_create(AgvMotorBase* out, const AgvCommUartCfg* link_cfg,
                      const AgvCommFmtModbusRtuCfg* format_cfg,
                      const AgvCommPrtclBlvrCfg* protocol_cfg);
int Motor_blvr_destroy(AgvMotorBase* base);

#endif  // AGV_MODULE_FACTORY__MOTOR_BUILDER_H_