#ifndef AGV_CORE__MOTORS_BASE_H_
#define AGV_CORE__MOTORS_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvMotorsBase {
    int (*get_state_from_buffer)(struct AgvMotorsBase* base);
    int (*get_vel_from_buffer)(struct AgvMotorsBase* base, WheelsVel* out);
    int (*set_desired_vel_to_buffer)(struct AgvMotorsBase* base,
                                     const WheelsVel* in);
    int (*readTo_and_writeFrom_buffer)(struct AgvMotorsBase* base);
    int (*destroy)(struct AgvMotorsBase* base);
    void* impl;
} AgvMotorsBase;

#endif  // AGV_CORE__MOTORS_BASE_H_