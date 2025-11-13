#ifndef __AGV_APP_H__
#define __AGV_APP_H__

#include "Agv_core/core.h"
#include "Agv_factory/agv_factory.h"

// clang-format off
int Agv_garmin_init(AgvCore* agv_core,
                    AgvHostRosCfg* host_ros_cfg, 
                    AgvMotorBlvrConfig* blvr_cfg,
                    AgvMecanumConfig* mecanum_cfg, 
                    AgvPidConfig* pid_cfg);
// clang-format on
int Agv_comm_init(AgvCore* agv_core, AgvHostRosCfg* host_ros_cfg);

int Host_ros_init(AgvHostRosCfg* host_ros_cfg);
int Motor_blvr_init(AgvMotorBlvrConfig* blvr_cfg);
int Kinematic_Mecanum_init(AgvMecanumConfig* mecanum_cfg);
int Control_pid_init(AgvPidConfig* pid_cfg);

int AGV_attach_core_task(AgvCore* agv_core);

#endif  // __AGV_APP_H__