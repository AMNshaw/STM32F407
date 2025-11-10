#ifndef AGV_CONTROL__PID_CONFIG_H_
#define AGV_CONTROL__PID_CONFIG_H_

typedef struct {
    float kp_lin;
    float ki_lin;
    float kd_lin;
    float kp_yaw;
    float ki_yaw;
    float kd_yaw;
} AgvPidConfig;

#endif  // AGV_CONTROL__PID_CONFIG_H_