#ifndef AGV_FACTORY__KINEMATICS_BUILDER_H_
#define AGV_FACTORY__KINEMATICS_BUILDER_H_

#include "Agv_core/modules/kinematics_base.h"
#include "Agv_kinematics/mecanum_config.h"

int Kinematics_mecanum_create(AgvKinematicsBase* out,
                              const AgvMecanumConfig* cfg);
int Kinematics_mecanum_destroy(AgvKinematicsBase* base);

#endif  // AGV_FACTORY__KINEMATICS_BUILDER_H_