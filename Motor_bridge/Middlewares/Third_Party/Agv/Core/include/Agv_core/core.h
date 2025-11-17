#ifndef AGV_CORE__CORE_H_
#define AGV_CORE__CORE_H_

#include "Agv_core/agv_types.h"
#include "Agv_core/modules/control_law_base.h"
#include "Agv_core/modules/host_communication_base.h"
#include "Agv_core/modules/kinematics_base.h"
#include "Agv_core/modules/motors_base.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
    AgvKinematicsBase kinematic_base;
    AgvMotorsBase motors_base;
    AgvControlLawBase control_law_base;
    AgvHostCommunicationBase host_communication_base;

    Odometry odom;

    SemaphoreHandle_t mutex_odom;

} AgvCore;

int AgvCore_step_on_host_msg(AgvCore* core);

int AgvCore_step_host_control(AgvCore* core);

int AgvCore_step_motor_io(AgvCore* core);

int AgvCore_step_feedback(AgvCore* core);

int AgvCore_set_cmd_vel(AgvCore* core, Twist2D cmd_in);

#endif  // AGV_CORE__CORE_H_