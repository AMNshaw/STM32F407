#ifndef AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_
#define AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_

#include <stdlib.h>
/**
 * Host msg
 */
typedef enum { VEL_CMD, ODOMETRY } HostMsgType;
typedef struct {
    float x;
    float y;
    float yaw;
} Position;

typedef struct {
    float v_x;
    float v_y;
    float v_yaw;
} Velocity;

typedef struct {
    Position pos;
    Velocity vel;
} Odometry;

typedef struct {
    HostMsgType type;
    union {
        Velocity vel;
        Odometry odom;
    } msg;
} HostCommMsg;

/**
 * Motor msg
 */
typedef enum { READ, WRITE, READ_WRITE } MotorMsgType;
typedef struct {
    int32_t des_vel;  // vel of wheel
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

/**
 * Totol
 */
typedef enum { HOST_MSG, MOTOR_MSG } AgvMsgType;
typedef struct {
    AgvMsgType msg_type;
    union {
        HostCommMsg host_msg;
        MotorsCommMsg motors_msg;
    } u;
} AgvCommMsg;

#endif  // AGV_COMMUNICATION_PACK__COMMUNICATION_MSGS_H_