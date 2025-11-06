#ifndef AGV_CORE__CONTROL_LAW_BASE_H_
#define AGV_CORE__CONTROL_LAW_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvControlLawBase {
    void (*calc_vel_cmd)(struct AgvControlLawBase* self, const VelCmd* cmd_in,
                         const VelCmd* est, VelCmd* cmd_out);
    void* impl;
} AgvControlLawBase;

#endif  // AGV_CORE__CONTROL_LAW_BASE_H_