#ifndef AGV_CORE__AGV_TYPES_H_
#define AGV_CORE__AGV_TYPES_H_

#include <stdint.h>

typedef struct {
    float vx, vy, wz;
} VelCmd;

typedef struct {
    float w[4];
} WheelsVel;

typedef struct {
    float x, y, yaw;
    VelCmd twist;
} Odom;

typedef enum { VEL_CMD } HostMsgType;
typedef struct {
    HostMsgType msg_type;
    union {
        VelCmd vel_cmd;
    } msg;
} HostInterMsg;

typedef struct {
    int32_t rpm;
} MotorInterMsg;

typedef enum { Host_MSG, MOTOR_MSG } AgvMsgType;
typedef struct {
    AgvMsgType msg_type;
    union {
        HostInterMsg host_msg;
        MotorInterMsg motors_msg[4];
    } u;
} AgvInterMsg;

// 小工具
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#endif  // AGV_CORE__AGV_TYPES_H_