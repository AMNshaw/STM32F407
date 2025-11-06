#ifndef AGV_CORE__MOTOR_COMMUNICATION_BASE_H_
#define AGV_CORE__MOTOR_COMMUNICATION_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvMotorCommunicationBase {
    int (*get_feedback_msg)(struct AgvMotorCommunicationBase* self,
                            MotorMsg* out_msg);
    int (*write_targets)(struct AgvMotorCommunicationBase* self,
                         const WheelVel* in);
    void* impl;  // 私有實作
} AgvMotorCommunicationBase;

#endif  // AGV_CORE__MOTOR_COMMUNICATION_BASE_H_