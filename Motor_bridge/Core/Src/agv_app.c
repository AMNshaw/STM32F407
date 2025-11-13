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

    host_ros_cfg->rosFmt_cfg.crc_cfg.crc_init = 0x00;
    host_ros_cfg->rosFmt_cfg.crc_cfg.crc_poly = 0x07;
    host_ros_cfg->rosFmt_cfg.header0 = 0x55;
    host_ros_cfg->rosFmt_cfg.header1 = 0xAA;
    host_ros_cfg->rosFmt_cfg.tail0 = 0x0D;
    host_ros_cfg->rosFmt_cfg.tail1 = 0x0A;
    host_ros_cfg->rosFmt_cfg.max_frame_len = 256;

    host_ros_cfg->prtcl_host_cfg.max_payload_len = 128;

    return 0;
}

int Motor_blvr_init(AgvMotorBlvrConfig* blvr_cfg) {
    blvr_cfg->axis_count = 4;
    blvr_cfg->prtcl_blvr_cfg.axis_count = blvr_cfg->axis_count;
    return 0;
}

int Kinematic_Mecanum_init(AgvMecanumConfig* mecanum_cfg) {
    mecanum_cfg->wheel_radius = 0.076;
    mecanum_cfg->W = 0.259;
    mecanum_cfg->L = 0.27;
    return 0;
}

int Control_pid_init(AgvPidConfig* pid_cfg) {
    pid_cfg->kp_lin = 1;
    pid_cfg->kd_lin = 1;
    return 0;
}

// clang-format off
int Agv_garmin_init(AgvCore* agv_core,
                    AgvHostRosCfg* host_ros_cfg, 
                    AgvMotorBlvrConfig* blvr_cfg,
                    AgvMecanumConfig* mecanum_cfg, 
                    AgvPidConfig* pid_cfg) {
    // clang-format on
    Host_ros_init(host_ros_cfg);
    Motor_blvr_init(blvr_cfg);
    Kinematic_Mecanum_init(mecanum_cfg);
    Control_pid_init(pid_cfg);

    Agv_garmin_create(agv_core, host_ros_cfg, blvr_cfg, mecanum_cfg, pid_cfg);
    return 0;
}

int Agv_comm_kin_init(AgvCore* agv_core, AgvHostRosCfg* host_ros_cfg,
                      AgvMecanumConfig* mecanum_cfg) {
    Host_ros_init(host_ros_cfg);
    Kinematic_Mecanum_init(mecanum_cfg);
    int code = Agv_comm_kin_create(agv_core, host_ros_cfg, mecanum_cfg);
    if (code != AGV_OK) printf("Failed to init agv, code: %d", code);

    return AGV_OK;
}

int Agv_comm_init(AgvCore* agv_core, AgvHostRosCfg* host_ros_cfg) {
    Host_ros_init(host_ros_cfg);
    int code = Agv_comm_create(agv_core, host_ros_cfg);
    if (code != AGV_OK) printf("Failed to init agv, code: %d", code);

    return AGV_OK;
}
