#ifndef AGV_COMMUNICATION_PACK__COMMUNICATION_MSG_H_
#define AGV_COMMUNICATION_PACK__COMMUNICATION_MSG_H_

#include "agv_types.h"

typedef enum {
    AGV_COMM_MSG_NONE = 0,
    AGV_COMM_MSG_CMD_VEL,
    AGV_COMM_MSG_ODOM,
    AGV_COMM_MSG_BLVR_RPM,
    AGV_COMM_MSG_BLVR_ACCEL,
    AGV_COMM_MSG_BLVR_DECEL,
    AGV_COMM_MSG_BLVR_SPEED_CTRL,
    AGV_COMM_MSG_BLVR_TRIGGER
} AgvCommMsgType;

typedef struct {
    int32_t rpm;
    int32_t accel;
    int32_t decel;
    uint16_t speed_ctrl;
    uint16_t trigger;
} BlvrOp;

typedef struct {
    AgvCommMsgType type;
    union {
        VelCmd cmd_vel;
        Odom odom;
        BlvrOp blvr_op[4];
    } data;
} AgvCommMsg;

#endif  // AGV_COMMUNICATION_PACK__COMMUNICATION_MSG_H_
