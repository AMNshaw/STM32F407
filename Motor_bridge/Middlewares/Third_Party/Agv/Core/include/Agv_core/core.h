#ifndef AGV_CORE__CORE_H_
#define AGV_CORE__CORE_H_

#include "Agv_core/agv_types.h"
#include "Agv_core/modules/control_law_base.h"
#include "Agv_core/modules/host_communication_base.h"
#include "Agv_core/modules/kinematics_base.h"
#include "Agv_core/modules/motors_base.h"

typedef struct {
    AgvKinematicsBase kinematic_base;
    AgvMotorsBase motors_base;
    AgvControlLawBase control_law_base;
    AgvHostCommunicationBase host_communication_base;

    Twist2D cmd_latest;
    Odometry odom;

} AgvCore;

#endif  // AGV_CORE__CORE_H_