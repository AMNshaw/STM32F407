#ifndef AGV_CORE__MOTOR_COMMUNICATION_BASE_H_
#define AGV_CORE__MOTOR_COMMUNICATION_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvMotorCommunicationBase {
    int (*get_state_from_buffer)(struct AgvMotorCommunicationBase* self);
    int (*get_vel_from_buffer)(struct AgvMotorCommunicationBase* self,
                               WheelsVel* out);
    int (*set_desired_vel_to_buffer)(struct AgvMotorCommunicationBase* self,
                                     const WheelsVel* in);
    int (*readTo_and_writeFrom_buffer)(struct AgvMotorCommunicationBase* self);
    int (*destroy)(struct AgvMotorCommunicationBase* self);
    void* impl;
} AgvMotorCommunicationBase;

#endif  // AGV_CORE__MOTOR_COMMUNICATION_BASE_H_