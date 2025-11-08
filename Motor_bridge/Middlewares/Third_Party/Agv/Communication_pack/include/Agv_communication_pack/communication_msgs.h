#ifndef AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_
#define AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_

#include <stdint.h>

typedef enum { VEL_CMD } HostMsgType;
typedef struct {
} HostCommMsg;

typedef enum { READ, WRITE, READ_WRITE } MotorMsgType;
typedef struct {
    int32_t des_rpm;
    int32_t des_acc;
    int32_t des_dec;
    int32_t spd_ctrl;
    int32_t trigger;

    int32_t driver_st;
    int32_t rl_pos;
    int32_t rl_rpm;
    int32_t alrm;
} MotorMsgPack;
typedef struct {
    MotorMsgType type;
    MotorMsgPack msgs[4];
} MotorsCommMsg;

typedef enum { HOST_MSG, MOTOR_MSG } AgvMsgType;
typedef struct {
    AgvMsgType msg_type;
    union {
        HostCommMsg host_msg;
        MotorsCommMsg motors_msg;
    } u;
} AgvCommMsg;

#endif  // AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_