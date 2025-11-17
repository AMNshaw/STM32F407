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

    blvr_cfg->gearRatio_motor_to_wheel = 30;
    blvr_cfg->unit_step_degree = 0.01;
    blvr_cfg->unit_vel_rpm = 1;

    blvr_cfg->uart_cfg.huart = &huart3;
    blvr_cfg->uart_cfg.baudrate = huart3.Init.BaudRate;
    blvr_cfg->uart_cfg.max_data_len = 512;
    blvr_cfg->uart_cfg.operation_timeout_ms = 1000;

    blvr_cfg->modbus_cfg.crc_cfg.crc_init = (uint16_t)0xFFFFu;
    blvr_cfg->modbus_cfg.crc_cfg.crc_poly = (uint16_t)0xA001u;
    blvr_cfg->modbus_cfg.max_frame_len = 500;

    blvr_cfg->prtcl_blvr_cfg.axis_count = blvr_cfg->axis_count;
    blvr_cfg->prtcl_blvr_cfg.byte_per_rgstr = 2;
    blvr_cfg->prtcl_blvr_cfg.max_payload_len = 400;
    blvr_cfg->prtcl_blvr_cfg.num_read_cmd = 4;
    blvr_cfg->prtcl_blvr_cfg.num_write_cmd = 5;
    blvr_cfg->prtcl_blvr_cfg.operation_trigger = (int32_t)0x01;
    blvr_cfg->prtcl_blvr_cfg.operation_type = (int32_t)0x10;
    blvr_cfg->prtcl_blvr_cfg.reg_address_read.driver_status = (uint16_t)10;
    blvr_cfg->prtcl_blvr_cfg.reg_address_read.real_pos = (uint16_t)12;
    blvr_cfg->prtcl_blvr_cfg.reg_address_read.real_vel = (uint16_t)14;
    blvr_cfg->prtcl_blvr_cfg.reg_address_read.present_alarm = (uint16_t)16;
    blvr_cfg->prtcl_blvr_cfg.reg_address_write.cmd_vel = (uint16_t)0;
    blvr_cfg->prtcl_blvr_cfg.reg_address_write.cmd_acc = (uint16_t)2;
    blvr_cfg->prtcl_blvr_cfg.reg_address_write.cmd_dec = (uint16_t)4;
    blvr_cfg->prtcl_blvr_cfg.reg_address_write.cmd_op = (uint16_t)6;
    blvr_cfg->prtcl_blvr_cfg.reg_address_write.cmd_trg = (uint16_t)8;
    blvr_cfg->prtcl_blvr_cfg.shared_id = (uint16_t)0x0F;

    return 0;
}

int Kinematic_Mecanum_init(AgvKineMecanumConfig* mecanum_cfg) {
    mecanum_cfg->wheel_radius = 0.076;
    mecanum_cfg->W = 0.259;
    mecanum_cfg->L = 0.27;
    return 0;
}

int Control_passthrogh_init(AgvCtrlPassthroughConfig* passthrough_cfg) {
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
                    AgvKineMecanumConfig* mecanum_cfg, 
                    AgvCtrlPassthroughConfig* passthrough_cfg) {
    // clang-format on
    Host_ros_init(host_ros_cfg);
    Motor_blvr_init(blvr_cfg);
    Kinematic_Mecanum_init(mecanum_cfg);
    Control_passthrogh_init(passthrough_cfg);

    Agv_test_create(agv_core, host_ros_cfg, blvr_cfg, mecanum_cfg,
                    passthrough_cfg);
    return 0;
}
