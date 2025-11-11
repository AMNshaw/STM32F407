#ifndef AGV_CORE__HOST_COMMUNICATION_BASE_H_
#define AGV_CORE__HOST_COMMUNICATION_BASE_H_

#include "Agv_core/agv_types.h"

typedef struct AgvHostCommunicationBase {
    int (*get_des_vel_from_buffer)(AgvHostCommunicationBase* base, VelCmd* out);
    int (*send_odom)(struct AgvHostCommunicationBase* base, const Odom* in);
    int (*send_heartbeat)(struct AgvHostCommunicationBase* base);
    int (*process_pending_msg_to_buffer)(struct AgvHostCommunicationBase* base);
    int (*destroy)(struct AgvHostCommunicationBase* base);
    void* impl;
} AgvHostCommunicationBase;

#endif  // AGV_CORE__HOST_COMMUNICATION_BASE_H_