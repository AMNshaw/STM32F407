#ifndef AGV_CORE__MOTORS_BASE_H_
#define AGV_CORE__MOTORS_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvMotorsBase {
    char* name;
    int (*reset)(struct AgvMotorsBase* base);
    int (*on_off)(struct AgvMotorsBase* base, bool state);
    int (*get_state)(struct AgvMotorsBase* base);
    int (*get_curr_wheels_vel)(struct AgvMotorsBase* base, WheelsVel* out);
    int (*get_curr_wheels_ang)(struct AgvMotorsBase* base, WheelsAng* out);
    int (*set_des_wheel_vel)(struct AgvMotorsBase* base, const WheelsVel* in);
    int (*readTo_and_writeFrom_buffer)(struct AgvMotorsBase* base);
    int (*destroy)(struct AgvMotorsBase* base);
    void* impl;
} AgvMotorsBase;

#endif  // AGV_CORE__MOTORS_BASE_H_