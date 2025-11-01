#ifndef AGV_CORE__TYPES_H_
#define AGV_CORE__TYPES_H_

#include <stdint.h>

typedef struct {
    float vx, vy, wz;
} VelCmd;
// 車體速度 (m/s, rad/s)

typedef struct {
    float w[];
} WheelVel;
// 四輪角速度 (rad/s)

typedef struct {
    float x, y, yaw;  // 可先不積分，只用 twist 當回傳
    VelCmd twist;
} Odom;

typedef struct {
    float R;     // 輪半徑
    float L, B;  // 軸距/輪距（你也可改成 LplusB = L + B）
} KineParams;

// 小工具
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

#endif  // AGV_CORE__TYPES_H__