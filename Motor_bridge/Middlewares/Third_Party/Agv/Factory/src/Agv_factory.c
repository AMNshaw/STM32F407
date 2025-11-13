#include "Agv_factory/agv_factory.h"

#include <stdio.h>

#include "Agv_core/error_codes/error_common.h"

int Agv_garmin_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg,
                      const AgvMotorBlvrConfig* blvr_cfg,
                      const AgvMecanumConfig* mecanum_cfg,
                      const AgvPidConfig* pid_cfg) {
    Host_communication_ros_create(&core->host_communication_base, host_ros_cfg);
    Motors_blvr_create(&core->motors_base, blvr_cfg);
    Kinematics_mecanum_create(&core->kinematic_base, mecanum_cfg);
    Ctrl_PID_create(&core->control_law_base, pid_cfg);

    return AGV_OK;
}

int Agv_comm_kin_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg,
                        const AgvMecanumConfig* mecanum_cfg) {
    printf("Creating host communication module...\n");
    int code = Host_communication_ros_create(&core->host_communication_base,
                                             host_ros_cfg);
    if (code != AGV_OK) return code;
    printf("Creating kinematics module...\n");
    int code = Kinematics_mecanum_create(&core->kinematic_base, mecanum_cfg);
    if (code != AGV_OK) return code;

    return code;
}

int Agv_comm_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg) {
    printf("Creating host communication module...\n");
    int code = Host_communication_ros_create(&core->host_communication_base,
                                             host_ros_cfg);
    return code;
}

int Agv_destroy(AgvCore* core) {
    if (!core) return AGV_OK;

    AgvHostCommunicationBase* host_comm = &core->host_communication_base;
    AgvMotorsBase* motors = &core->motors_base;
    AgvKinematicsBase* kine = &core->kinematic_base;
    AgvControlLawBase* ctrl = &core->control_law_base;

    if (host_comm->destroy) host_comm->destroy(host_comm);
    if (motors->destroy) motors->destroy(motors);
    if (kine->destroy) kine->destroy(kine);
    if (ctrl->destroy) ctrl->destroy(ctrl);
    return 0;
}
