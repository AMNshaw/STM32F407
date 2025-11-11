#ifndef __AGV_APP_H__
#define __AGV_APP_H__

#include "Agv_core/core.h"
#include "Agv_factory/agv_factory.h"

AgvCore agv_core;

AgvHostRosCfg host_ros_cfg;
AgvMotorBlvrConfig blvr_cfg;
AgvMecanumConfig mecanum_cfg;
AgvPidConfig pid_cfg;

void Host_ros_init();
void Motor_blvr_init();
void Kinematic_Mecanum_init();
void Control_pid_init();

void Agv_garmin_init();
void Agv_comm_init();

#endif  // __AGV_APP_H__