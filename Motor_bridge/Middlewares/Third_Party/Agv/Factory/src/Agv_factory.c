#include "Agv_factory/agv_factory.h"

#include "Agv_core/error_codes/error_common.h"

int Agv_garmin_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg,
                      const AgvMotorBlvrConfig* blvr_cfg,
                      const AgvMecanumConfig* mecanum_cfg,
                      const AgvPidConfig* pid_cfg) {
    Host_communication_ros_create(&core->host_communication_base, host_ros_cfg);
    Motor_comm_blvr_create(&core->motors_communication_base, blvr_cfg);
    Kinematics_mecanum_create(&core->kinematic_base, mecanum_cfg);
    Ctrl_PID_create(&core->control_law_base, pid_cfg);
}

int Agv_destroy(AgvCore* core) {
    if (!core) return AGV_ERR_INVALID_ARG;

    AgvHostCommunicationBase* host_comm = &core->host_communication_base;
    AgvMotorCommunicationBase* motor_comm = &core->motors_communication_base;
    AgvKinematicsBase* kine = &core->kinematic_base;
    AgvControlLawBase* ctrl = &core->control_law_base;

    if (host_comm) host_comm->destroy(host_comm);
    if (motor_comm) motor_comm->destroy(motor_comm);
    if (kine) kine->destroy(kine);
    if (ctrl) ctrl->destroy(ctrl);
}

int Agv_comm_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg,
                    const AgvMotorBlvrConfig* blvr_cfg) {
    Host_communication_ros_create(&core->host_communication_base, host_ros_cfg);
    Motor_comm_blvr_create(&core->motors_communication_base, blvr_cfg);
}
