#include "Agv_factory/agv_factory.h"

#include <stdio.h>

#include "Agv_core/error_codes/error_common.h"

int Agv_test_create(AgvCore* core, const AgvHostRosCfg* host_ros_cfg,
                    const AgvMotorBlvrConfig* blvr_cfg,
                    const AgvKineMecanumConfig* mecanum_cfg,
                    const AgvCtrlPassthroughConfig* passthrogh_cfg) {
    Host_communication_ros_create(&core->host_communication_base, host_ros_cfg);
    Motors_blvr_create(&core->motors_base, blvr_cfg);
    Kinematics_mecanum_create(&core->kinematic_base, mecanum_cfg);
    Ctrl_passthrough_create(&core->control_law_base, passthrogh_cfg);

    core->mutex_odom = xSemaphoreCreateMutex();
    if (core->mutex_odom == NULL) {
        Agv_destroy(core);
        return AGV_ERR_MUTEX_FAIL;
    }

    return AGV_OK;
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

    vSemaphoreDelete(core->mutex_odom);

    return 0;
}
