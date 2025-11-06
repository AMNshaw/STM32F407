#ifndef AGV_MODULE_FACTORY__CONTROL_LAW_BUILDER_H_
#define AGV_MODULE_FACTORY__CONTROL_LAW_BUILDER_H_

#include "Agv_core/modules/control_law_base.h"

int Ctrl_Passthrough_create(AgvControlLawBase* out);
int Ctrl_Passthrough_destroy(AgvControlLawBase* base);

int Ctrl_PID_create(AgvControlLawBase* out, float kp_lin, float ki_lin,
                    float kd_lin, float kp_yaw, float ki_yaw, float kd_yaw);
int Ctrl_PID_destroy(AgvControlLawBase* base);

#endif  // AGV_MODULE_FACTORY__CONTROL_LAW_BUILDER_H_