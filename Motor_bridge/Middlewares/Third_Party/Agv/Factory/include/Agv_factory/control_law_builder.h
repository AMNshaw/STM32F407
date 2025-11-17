#ifndef AGV_FACTORY__CONTROL_LAW_BUILDER_H_
#define AGV_FACTORY__CONTROL_LAW_BUILDER_H_

#include "Agv_control/passthrough_config.h"
#include "Agv_control/pid_config.h"
#include "Agv_core/modules/control_law_base.h"

int Ctrl_passthrough_create(AgvControlLawBase* out,
                            const AgvCtrlPassthroughConfig* cfg);

int Ctrl_pid_create(AgvControlLawBase* out, const AgvPidConfig* cfg);

#endif  // AGV_FACTORY__CONTROL_LAW_BUILDER_H_