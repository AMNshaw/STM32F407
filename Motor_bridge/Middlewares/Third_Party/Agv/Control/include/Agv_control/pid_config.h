#ifndef AGV_CONTROL__PID_CONFIG_H_
#define AGV_CONTROL__PID_CONFIG_H_
#include "Agv_core/agv_types.h"

typedef struct {
    float kp;
    float ki;
    float kd;
} PidConfig;

typedef struct {
    PidConfig gain_x;
    PidConfig gain_y;
    PidConfig gain_yaw;

    XYYaw max_acc;
} AgvCtrlPidConfig;

#endif  // AGV_CONTROL__PID_CONFIG_H_