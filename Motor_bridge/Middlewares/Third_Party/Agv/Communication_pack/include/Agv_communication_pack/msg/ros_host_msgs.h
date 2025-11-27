#ifndef AGV_COMMUNICATION_PACK__ROS_HOST_MSGS_H_
#define AGV_COMMUNICATION_PACK__ROS_HOST_MSGS_H_

#include "Agv_core/agv_types.h"

/**
 * Host msg
 */
typedef enum { VEL_CMD, ODOMETRY, HEARTBEAT } RosHostMsgType;

typedef struct {
    RosHostMsgType msg_type;
    union {
        Twist2D vel;
        Odometry odom;
    } u;
} RosHostMsg;

#endif  // AGV_COMMUNICATION_PACK__ROS_HOST_MSGS_H_
