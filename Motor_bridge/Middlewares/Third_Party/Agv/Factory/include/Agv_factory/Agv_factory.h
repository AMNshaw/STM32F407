#ifndef AGV_FACTORY__AGV_FACTORY_H_
#define AGV_FACTORY__AGV_FACTORY_H_

#include "Agv_core/core.h"
#include "Agv_factory/control_law_builder.h"
#include "Agv_factory/host_communication_builder.h"
#include "Agv_factory/kinematics_builder.h"
#include "Agv_factory/motor_communication_builder.h"

// clang-format off
int Agv_garmin_create(AgvCore* core, 
                      const AgvHostUartCfg* host_uart_cfg,
                      const AgvBlvrConfig* blvr_cfg,
                      const AgvMecanumConfig* mecanum_cfg,
                      const AgvPidConfig* pid_cfg);

int Agv_garmin_destroy(AgvCore* core);

// clang-format on

#endif  // AGV_FACTORY__AGV_FACTORY_H_