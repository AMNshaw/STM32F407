#ifndef AGV_KINEMATICS__MECANUM_CONFIG_H_
#define AGV_KINEMATICS__MECANUM_CONFIG_H_
#include <stdint.h>

typedef struct {
    float wheel_radius;
    float L, W;

    int8_t axis_dir[4];
} AgvKineMecanumConfig;

#endif  // AGV_KINEMATICS__MECANUM_CONFIG_H_