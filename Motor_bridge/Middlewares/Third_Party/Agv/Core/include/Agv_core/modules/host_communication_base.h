#ifndef AGV_CORE__HOST_COMMUNICATION_BASE_H_
#define AGV_CORE__HOST_COMMUNICATION_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvHostCommunicationBase {
    int (*receive_cmd)(struct AgvHostCommunicationBase* self, VelCmd* out);
    int (*send_odom)(struct AgvHostCommunicationBase* self, const Odom* in);
    void* impl;
} AgvHostCommunicationBase;

#endif  // AGV_CORE__HOST_COMMUNICATION_BASE_H_