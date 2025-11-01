#ifndef AGV_CORE__CONTROL_LAW_BASE_H_
#define AGV_CORE__CONTROL_LAW_BASE_H_

#include "types.h"

typedef struct AgvControlLawBase {
    void (*calc_vel_cmd)(struct AgvControlLawBase* self, const VelCmd* cmd_in,
                         const VelCmd* est, VelCmd* cmd_out);
    void* impl;
} AgvControlLawBase;

void Ctrl_Passthrough_create(AgvControlLawBase* out);
void Ctrl_PID_create(AgvControlLawBase* out, float kp_lin, float ki_lin,
                     float kd_lin, float kp_yaw, float ki_yaw, float kd_yaw);

#endif  // AGV_CORE__CONTROL_LAW_BASE_H_