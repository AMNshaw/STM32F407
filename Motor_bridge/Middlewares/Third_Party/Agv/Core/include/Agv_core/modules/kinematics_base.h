#ifndef AGV_CORE__KINEMATICS_BASE_H_
#define AGV_CORE__KINEMATICS_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvKinematicsBase {
    char* name;

    int (*calculate_wheels_vel)(struct AgvKinematicsBase* base,
                                const Twist2D* cmd_vel_in,
                                WheelsVel* wheels_vel_out);
    int (*calculate_odom)(struct AgvKinematicsBase* base,
                          const WheelsAng* wheels_ang_in,
                          const WheelsVel* wheels_vel_in, Odometry* odom_out);
    int (*destroy)(struct AgvKinematicsBase* base);
    void* impl;
} AgvKinematicsBase;

#endif  // AGV_CORE__KINEMATICS_BASE_H_