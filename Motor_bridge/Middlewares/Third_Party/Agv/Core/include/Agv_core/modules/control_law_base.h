#ifndef AGV_CORE__CONTROL_LAW_BASE_H_
#define AGV_CORE__CONTROL_LAW_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvControlLawBase {
    char* name;

    int (*set_des_vel)(struct AgvControlLawBase* base, const Twist2D* cmd_in);
    int (*set_curr_vel)(struct AgvControlLawBase* base,
                        const Twist2D* curr_vel);
    int (*get_ctrl_cmd)(struct AgvControlLawBase* base, Twist2D* cmd_out);
    int (*destroy)(struct AgvControlLawBase* base);
    void* impl;
} AgvControlLawBase;

#endif  // AGV_CORE__CONTROL_LAW_BASE_H_