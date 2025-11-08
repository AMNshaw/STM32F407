#ifndef AGV_MODULE_FACTORY__MOTOR_BUILDER_H_
#define AGV_MODULE_FACTORY__MOTOR_BUILDER_H_

#include "Agv_core/modules/motor_communication_base.h"
#include "Agv_motor_communication/blvr_config.h"

int Motor_comm_blvr_create(AgvMotorCommunicationBase* out,
                           const Agv_Blvr_config* cfg);
int Motor_comm_blvr_destroy(AgvMotorCommunicationBase* base);

#endif  // AGV_MODULE_FACTORY__MOTOR_BUILDER_H_