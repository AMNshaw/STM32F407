#ifndef AGV_CORE__KINEMATICS_BASE_H_
#define AGV_CORE__KINEMATICS_BASE_H_

#include "types.h"

typedef struct AgvKinematicsBase {
    void (*control_motors)(struct AgvKinematicsBase* self, const VelCmd* cmd_in,
                           const WheelVel* meas, WheelVel* target_out);
    void (*calculate_odom)(struct AgvKinematicsBase* self, const WheelVel* meas,
                           Odom* odom_out);
    void* impl;  // 幾何參數/內部狀態
} AgvKinematicsBase;

int Kinematics_mecanum_create(AgvKinematicsBase* out, const KineParams* p);

#endif  // AGV_CORE__KINEMATICS_BASE_H_