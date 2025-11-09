#ifndef AGV_CORE__KINEMATICS_BASE_H_
#define AGV_CORE__KINEMATICS_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvKinematicsBase {
    void (*control_motors)(struct AgvKinematicsBase* self, const VelCmd* cmd_in,
                           const WheelsVel* meas, WheelsVel* target_out);
    void (*calculate_odom)(struct AgvKinematicsBase* self,
                           const WheelsVel* meas, Odom* odom_out);
    void* impl;  // 幾何參數/內部狀態
} AgvKinematicsBase;

#endif  // AGV_CORE__KINEMATICS_BASE_H_