#include "Agv_factory/Agv_factory.h"

int Agv_garmin_create(AgvCore* core, const AgvHostUartCfg* host_uart_cfg,
                      const AgvBlvrConfig* blvr_cfg,
                      const AgvMecanumConfig* mecanum_cfg,
                      const AgvPidConfig* pid_cfg) {
    Host_communication_uart_create(&core->host_communication_base,
                                   host_uart_cfg);
    Motor_comm_blvr_create(&core->motors_communication_base, blvr_cfg);
    Kinematics_mecanum_create(&core->kinematic_base, mecanum_cfg);
    Ctrl_PID_create(&core->control_law_base, pid_cfg);
}

int Agv_garmin_destroy(AgvCore* core) {
    Host_communication_uart_destroy(&core->host_communication_base);
    Motor_comm_blvr_destroy(&core->motors_communication_base);
    Kinematics_mecanum_destroy(&core->kinematic_base);
    Ctrl_PID_destroy(&core->control_law_base);
}