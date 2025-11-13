#ifndef AGV_CORE__CONTROL_LAW_BASE_H_
#define AGV_CORE__CONTROL_LAW_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvControlLawBase {
    void (*calc_vel_cmd)(struct AgvControlLawBase* base, const Twist2D* cmd_in,
                         const Twist2D* est, Twist2D* cmd_out);
    int (*destroy)(struct AgvControlLawBase* base);
    void* impl;
} AgvControlLawBase;

#endif  // AGV_CORE__CONTROL_LAW_BASE_H_