#ifndef AGV_CORE__AGV_TYPES_H_
#define AGV_CORE__AGV_TYPES_H_

#include <stdbool.h>
#include <stdlib.h>

typedef struct {
    float x;
    float y;
    float yaw;
} XYYaw;

typedef XYYaw Pose2D;
typedef XYYaw Twist2D;

typedef union {
    float w2[2];
    float w4[4];
} Wheels;

typedef Wheels WheelsStep;
typedef Wheels WheelsAng;
typedef Wheels WheelsVel;

typedef struct {
    Pose2D pose;
    Twist2D twist;
} Odometry;

// 小工具
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#endif  // AGV_CORE__AGV_TYPES_H_