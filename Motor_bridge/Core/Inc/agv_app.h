#ifndef __AGV_APP_H__
#define __AGV_APP_H__

#include "Agv_core/core.h"
#include "Agv_factory/Agv_factory.h"

AgvCore agv_core;

AgvHostUartCfg host_uart_cfg;
AgvBlvrConfig blvr_cfg;
AgvMecanumConfig mecanum_cfg;
AgvPidConfig pid_cfg;

void Host_uart_init();
void Motor_blvr_init();
void Kinematic_Mecanum_init();
void Control_pid_init();

void Agv_garmin_init();

#endif  // __AGV_APP_H__