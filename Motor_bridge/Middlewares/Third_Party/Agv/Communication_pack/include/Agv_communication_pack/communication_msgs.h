#ifndef AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_
#define AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_

#include <stdint.h>

typedef enum { VEL_CMD } HostMsgType;
typedef struct {
} HostCommMsg;

typedef enum { READ, WRITE } MotorMsgType;
typedef struct {
    uint32_t des_rpm;
    uint32_t des_acc;
    uint32_t des_dec;
    uint32_t spd_ctrl;
    uint32_t trigger;

    uint32_t driver_st;
    uint32_t rl_pos;
    uint32_t rl_rpm;
    uint32_t alrm;
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