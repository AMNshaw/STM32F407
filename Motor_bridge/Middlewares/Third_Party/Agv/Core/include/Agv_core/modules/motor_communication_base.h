#ifndef AGV_CORE__MOTOR_COMMUNICATION_BASE_H_
#define AGV_CORE__MOTOR_COMMUNICATION_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvMotorCommunicationBase {
    int (*init)(struct AgvMAgvMotorCommunicationBaseotorBase* self);
    int (*write_targets)(struct AgvMotorCommunicationBase* self,
                         const WheelVel* in);
    int (*read_feedback)(struct AgvMotorCommunicationBase* self, WheelVel* out);
    void* impl;  // 私有實作
} AgvMotorCommunicationBase;

#endif  // AGV_CORE__MOTOR_COMMUNICATION_BASE_H_