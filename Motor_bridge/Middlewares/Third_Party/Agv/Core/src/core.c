#include "Agv_core/core.h"

#include "Agv_core/error_codes/error_common.h"

int AgvCore_step_on_host_msg(AgvCore* core) {
    if (!core) return AGV_ERR_INVALID_ARG;
    AgvHostCommunicationBase* host_comm = &core->host_communication_base;

    return host_comm->process_pending_msg_to_buffer(host_comm);
}

int AgvCore_step_host_control(AgvCore* core) {
    if (!core) return AGV_ERR_INVALID_ARG;
    AgvHostCommunicationBase* host_comm = &core->host_communication_base;
    AgvKinematicsBase* kine = &core->kinematic_base;
    AgvControlLawBase* ctrl = &core->control_law_base;
    AgvMotorsBase* motors = &core->motors_base;

    Twist2D curr_vel;
    xSemaphoreTake(core->mutex_odom, portMAX_DELAY);
    curr_vel = core->odom.twist;
    xSemaphoreGive(core->mutex_odom);

    int code = 0;
    Twist2D cmd;
    code = host_comm->get_des_vel_from_buffer(host_comm, &cmd);
    if (code != AGV_OK) return code;
    code = ctrl->set_des_vel(ctrl, &cmd);
    if (code != AGV_OK) return code;
    code = ctrl->set_curr_vel(ctrl, &curr_vel);
    if (code != AGV_OK) return code;
    code = ctrl->get_ctrl_cmd(ctrl, &cmd);
    if (code != AGV_OK) return code;

    WheelsVel wheels_cmd;
    code = kine->calculate_wheels_vel(kine, &cmd, &wheels_cmd);
    if (code != AGV_OK) return code;
    code = motors->set_des_wheel_vel_to_buffer(motors, &wheels_cmd);
    if (code != AGV_OK) return code;

    return AGV_OK;
}

int AgvCore_step_motor_io(AgvCore* core) {
    if (!core) return AGV_ERR_INVALID_ARG;

    AgvMotorsBase* motors = &core->motors_base;
    AgvKinematicsBase* kine = &core->kinematic_base;

    WheelsAng wheels_ang;
    WheelsVel wheels_vel;
    Odometry odom_out;

    int code = 0;
    code = motors->readTo_and_writeFrom_buffer(motors);
    if (code != AGV_OK) return code;
    code = motors->get_curr_wheels_ang_from_buffer(motors, &wheels_ang);
    if (code != AGV_OK) return code;
    code = motors->get_curr_wheels_vel_from_buffer(motors, &wheels_vel);
    if (code != AGV_OK) return code;
    code = kine->calculate_odom(kine, &wheels_ang, &wheels_vel, &odom_out);
    if (code != AGV_OK) return code;

    xSemaphoreTake(core->mutex_odom, portMAX_DELAY);
    core->odom = odom_out;
    xSemaphoreGive(core->mutex_odom);

    return AGV_OK;
}

int AgvCore_step_feedback(AgvCore* core) {
    if (!core) return AGV_ERR_INVALID_ARG;
    AgvHostCommunicationBase* host_comm = &core->host_communication_base;

    Odometry odom_send;
    xSemaphoreTake(core->mutex_odom, portMAX_DELAY);
    odom_send = core->odom;
    xSemaphoreGive(core->mutex_odom);

    int code = host_comm->send_odom(host_comm, &odom_send);
    if (code != AGV_OK) return code;

    return AGV_OK;
}

int AgvCore_set_cmd_vel(AgvCore* core, Twist2D cmd_in) {
    if (!core) return AGV_ERR_INVALID_ARG;

    AgvKinematicsBase* kine = &core->kinematic_base;
    AgvControlLawBase* ctrl = &core->control_law_base;
    AgvMotorsBase* motors = &core->motors_base;

    Twist2D curr_vel;
    xSemaphoreTake(core->mutex_odom, portMAX_DELAY);
    curr_vel = core->odom.twist;
    xSemaphoreGive(core->mutex_odom);

    int code = 0;
    if (code != AGV_OK) return code;
    code = ctrl->set_des_vel(ctrl, &cmd_in);
    if (code != AGV_OK) return code;
    code = ctrl->set_curr_vel(ctrl, &curr_vel);
    if (code != AGV_OK) return code;
    code = ctrl->get_ctrl_cmd(ctrl, &cmd_in);
    if (code != AGV_OK) return code;

    WheelsVel wheels_cmd;
    code = kine->calculate_wheels_vel(kine, &cmd_in, &wheels_cmd);
    if (code != AGV_OK) return code;
    code = motors->set_des_wheel_vel_to_buffer(motors, &wheels_cmd);
    if (code != AGV_OK) return code;

    return AGV_OK;
}