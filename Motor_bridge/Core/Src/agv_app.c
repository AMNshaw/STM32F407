#include "agv_app.h"

#include <stdio.h>

#include "Agv_core/error_codes/error_common.h"
#include "usart.h"

int Host_ros_init(AgvHostRosCfg* host_ros_cfg) {
    host_ros_cfg->uart_cfg.huart = &huart2;
    host_ros_cfg->uart_cfg.baudrate = huart2.Init.BaudRate;
    host_ros_cfg->uart_cfg.max_data_len = 256;
    host_ros_cfg->uart_cfg.operation_timeout_ms = 1000;
    host_ros_cfg->uart_cfg.queue_len = 20;

    host_ros_cfg->rosFmt_cfg.max_frame_len = 256;

    host_ros_cfg->prtcl_host_cfg.max_payload_len = 128;

    return 0;
}

int Motor_blvr_init(AgvMotorBlvrConfig* blvr_cfg) {
    blvr_cfg->axis_count = 4;

    blvr_cfg->gearRatio_motor_to_wheel = 30;
    blvr_cfg->unit_step_degree = 0.01;
    blvr_cfg->unit_vel_rpm = 1;

    blvr_cfg->uart_cfg.huart = &huart3;
    blvr_cfg->uart_cfg.auto_DE = true;
    blvr_cfg->uart_cfg.baudrate = huart3.Init.BaudRate;
    blvr_cfg->uart_cfg.max_data_len = 256;
    blvr_cfg->uart_cfg.operation_timeout_ms = 1000;
    blvr_cfg->uart_cfg.queue_len = 20;

    blvr_cfg->modbus_cfg.max_frame_len = 500;

    blvr_cfg->prtcl_blvr_cfg.axis_count = blvr_cfg->axis_count;
    blvr_cfg->prtcl_blvr_cfg.max_payload_len = 400;

    return 0;
}

int Kinematic_Mecanum_init(AgvKineMecanumConfig* mecanum_cfg) {
    mecanum_cfg->wheel_radius = 0.076;
    mecanum_cfg->W = 0.259;
    mecanum_cfg->L = 0.270;
    int8_t dir[4] = {1, -1, 1, -1};  // 跟你現在一樣的 pattern
    for (size_t i = 0; i < 4; ++i) {
        mecanum_cfg->axis_dir[i] = dir[i];
    }
    return 0;
}

int Control_passthrogh_init(AgvCtrlPassthroughConfig* passthrough_cfg) {
    return 0;
}

int Control_pid_init(AgvCtrlPidConfig* pid_cfg) {
    pid_cfg->kp_lin = 10;
    pid_cfg->kp_yaw = 10;
    pid_cfg->passthrough_thres = 0.01;
    return 0;
}

// clang-format off
int Agv_garmin_init(AgvCore* agv_core,
                    AgvHostRosCfg* host_ros_cfg, 
                    AgvMotorBlvrConfig* blvr_cfg,
                    AgvKineMecanumConfig* mecanum_cfg, 
                    AgvCtrlPidConfig* pid_cfg) {
    // clang-format on
    Host_ros_init(host_ros_cfg);
    Motor_blvr_init(blvr_cfg);
    Kinematic_Mecanum_init(mecanum_cfg);
    Control_pid_init(pid_cfg);

    Agv_garmin_create(agv_core, host_ros_cfg, blvr_cfg, mecanum_cfg, pid_cfg);
    return 0;
}
