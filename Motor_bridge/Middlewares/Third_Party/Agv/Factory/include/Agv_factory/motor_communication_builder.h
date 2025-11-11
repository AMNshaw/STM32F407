#ifndef AGV_FACTORY__MOTOR_BUILDER_H_
#define AGV_FACTORY__MOTOR_BUILDER_H_

#include "Agv_core/modules/motor_communication_base.h"
#include "Agv_motor_communication/blvr_config.h"

int Motor_comm_blvr_create(AgvMotorCommunicationBase* out,
                           const AgvMotorBlvrConfig* cfg);

#endif  // AGV_FACTORY__MOTOR_BUILDER_H_