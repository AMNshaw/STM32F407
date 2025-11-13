#ifndef AGV_FACTORY__AGV_FACTORY_H_
#define AGV_FACTORY__AGV_FACTORY_H_

#include "Agv_core/core.h"
#include "Agv_factory/control_law_builder.h"
#include "Agv_factory/host_communication_builder.h"
#include "Agv_factory/kinematics_builder.h"
#include "Agv_factory/motors_builder.h"

// clang-format off
int Agv_garmin_create(AgvCore* core, 
                      const AgvHostRosCfg* host_ros_cfg,
                      const AgvMotorBlvrConfig* blvr_cfg,
                      const AgvMecanumConfig* mecanum_cfg,
                      const AgvPidConfig* pid_cfg);
// clang-format on

int Agv_comm_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg);

int Agv_comm_kin_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg,
                        const AgvMecanumConfig* mecanum_cfg);

int Agv_destroy(AgvCore* core);

#endif  // AGV_FACTORY__AGV_FACTORY_H_