#ifndef AGV_MODULE_FACTORY__KINEMATICS_BUILDER_H_
#define AGV_MODULE_FACTORY__KINEMATICS_BUILDER_H_

#include "Agv_core/modules/kinematics_base.h"
#include "Agv_kinematics/configs.h/kinematics_config.h"

int Kinematics_mecanum_create(AgvKinematicsBase* out, const KineParams* p);
int Kinematics_mecanum_destroy(AgvKinematicsBase* base);

#endif  // AGV_MODULE_FACTORY__KINEMATICS_BUILDER_H_