#ifndef __AGV_APP_H__
#define __AGV_APP_H__

#include "Agv_core/core.h"
#include "Agv_factory/agv_factory.h"

int AGV_attach_core_task(AgvCore* agv_core);

// clang-format off
int Agv_garmin_init(AgvCore* agv_core,
                    AgvHostRosCfg* host_ros_cfg, 
                    AgvMotorBlvrConfig* blvr_cfg,
                    AgvKineMecanumConfig* mecanum_cfg, 
                    AgvCtrlPassthroughConfig* passthrough_cfg);
// clang-format on

int Host_ros_init(AgvHostRosCfg* host_ros_cfg);

int Motor_blvr_init(AgvMotorBlvrConfig* blvr_cfg);

int Kinematic_Mecanum_init(AgvKineMecanumConfig* mecanum_cfg);

int Control_passthrogh_init(AgvCtrlPassthroughConfig* passthrough_cfg);
int Control_pid_init(AgvPidConfig* pid_cfg);

#endif  // __AGV_APP_H__