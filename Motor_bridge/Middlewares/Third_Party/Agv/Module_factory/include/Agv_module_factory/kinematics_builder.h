#ifndef AGV_MODULE_FACTORY__KINEMATICS_BUILDER_H_
#define AGV_MODULE_FACTORY__KINEMATICS_BUILDER_H_

#include "modules/kinematics_base.h"

int Kinematics_mecanum_create(AgvKinematicsBase* out, const KineParams* p);
int Kinematics_mecanum_destroy(AgvKinematicsBase* base);

#endif  // AGV_MODULE_FACTORY__KINEMATICS_BUILDER_H_