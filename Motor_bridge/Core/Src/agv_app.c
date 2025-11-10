#include "agv_app.h"

#include "usart.h"

void Host_uart_init() {
    host_uart_cfg.uart_cfg.huart = &huart2;
    host_uart_cfg.uart_cfg.baudrate = huart2.Init.BaudRate;
}

void Motor_blvr_init() {
    blvr_cfg.axis_count = 4;
    blvr_cfg.prtcl_blvr_cfg.axis_count = blvr_cfg.axis_count;
}

void Kinematic_Mecanum_init() {
    mecanum_cfg.R = 0.3;
    mecanum_cfg.B = 0.5;
}

void Control_pid_init() {
    pid_cfg.kp_lin = 1;
    pid_cfg.kd_lin = 1;
}

void Agv_garmin_init() {
    Host_uart_init();
    Motor_blvr_init();
    Kinematic_Mecanum_init();
    Control_pid_init();

    Agv_garmin_create(&agv_core, &host_uart_cfg, &blvr_cfg, &mecanum_cfg,
                      &pid_cfg);
}
