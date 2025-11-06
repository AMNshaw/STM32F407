#ifndef AGV_CORE__AGV_TYPES_H_
#define AGV_CORE__AGV_TYPES_H_

#include <stdint.h>

typedef struct {
    float vx, vy, wz;
} VelCmd;

typedef struct {
    float w[];
} WheelVel;

typedef struct {
    float x, y, yaw;
    VelCmd twist;
} Odom;

typedef struct {
    float R;
    float L, B;
} KineParams;

// 小工具
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#endif  // AGV_CORE__AGV_TYPES_H_