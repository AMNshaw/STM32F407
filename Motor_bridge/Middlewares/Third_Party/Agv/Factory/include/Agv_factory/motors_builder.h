#ifndef AGV_FACTORY__MOTOR_BUILDER_H_
#define AGV_FACTORY__MOTOR_BUILDER_H_

#include "Agv_core/modules/motors_base.h"
#include "Agv_motors/blvr_config.h"

int Motors_blvr_create(AgvMotorsBase* out, const AgvMotorBlvrConfig* cfg);

#endif  // AGV_FACTORY__MOTOR_BUILDER_H_