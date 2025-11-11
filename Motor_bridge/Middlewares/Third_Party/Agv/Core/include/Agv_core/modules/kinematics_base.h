#ifndef AGV_CORE__KINEMATICS_BASE_H_
#define AGV_CORE__KINEMATICS_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvKinematicsBase {
    void (*control_motors)(struct AgvKinematicsBase* base, const VelCmd* cmd_in,
                           const WheelsVel* meas, WheelsVel* target_out);
    void (*calculate_odom)(struct AgvKinematicsBase* base,
                           const WheelsVel* meas, Odom* odom_out);
    int (*destroy)(struct AgvKinematicsBase* base);
    void* impl;
} AgvKinematicsBase;

#endif  // AGV_CORE__KINEMATICS_BASE_H_