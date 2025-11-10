#ifndef AGV_FACTORY__CONTROL_LAW_BUILDER_H_
#define AGV_FACTORY__CONTROL_LAW_BUILDER_H_

#include "Agv_control/pid.h"
#include "Agv_core/modules/control_law_base.h"

int Ctrl_Passthrough_create(AgvControlLawBase* out);
int Ctrl_Passthrough_destroy(AgvControlLawBase* base);

int Ctrl_PID_create(AgvControlLawBase* out, const AgvPidConfig* cfg);
int Ctrl_PID_destroy(AgvControlLawBase* base);

#endif  // AGV_FACTORY__CONTROL_LAW_BUILDER_H_