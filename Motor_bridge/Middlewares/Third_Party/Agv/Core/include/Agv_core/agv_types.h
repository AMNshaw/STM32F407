#ifndef AGV_CORE__AGV_TYPES_H_
#define AGV_CORE__AGV_TYPES_H_

#include <stdint.h>

typedef struct {
    float vx, vy, wz;
} VelCmd;

typedef struct {
    float w[4];
} WheelVel;

typedef struct {
    float x, y, yaw;
    VelCmd twist;
} Odom;

typedef struct {
    float R;
    float L, B;
} KineParams;

typedef enum { VEL_CMD } HostMsgType;
typedef struct {
    HostMsgType msg_type;
    union {
        VelCmd vel_cmd;
    } msg;
} HostMsg;

typedef enum {
    AGV_COMM_MSG_NONE,
    AGV_COMM_MSG_RPM,
    AGV_COMM_MSG_ACCEL,
    AGV_COMM_MSG_DECEL,
    AGV_COMM_MSG_SPEED_CTRL,
    AGV_COMM_MSG_TRIGGER
} MotorMsgType;

typedef struct {
    MotorMsgType msg_type;
    union {
        int32_t rpm;
        int32_t accel;
        int32_t decel;
        uint16_t speed_ctrl;
        uint16_t trigger;
    } msgs[4];
} MotorsMsg;

typedef enum { Host_MSG, MOTOR_MSG } AgvMsgType;
typedef struct {
    AgvMsgType msg_type;
    union {
        HostMsg host_msg;
        MotorsMsg motors_msg;
    } u;
} AgvCommMsg;

// 小工具
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#endif  // AGV_CORE__AGV_TYPES_H_