#ifndef AGV_CORE__MOTOR_BASE_H_
#define AGV_CORE__MOTOR_BASE_H_

#include "types.h"

typedef struct AgvMotorBase {
    int (*init)(struct AgvMotorBase* self);
    int (*write_targets)(struct AgvMotorBase* self, const WheelVel* target);
    int (*read_feedback)(struct AgvMotorBase* self, WheelVel* meas);
    void* impl;  // 私有實作
} AgvMotorBase;

// 具體實作工廠（Oriental + RS485/Modbus）
void OrientalMotor_create(AgvMotorBase* out /* 可再加 UART/ID 等參數 */);

#endif  // AGV_CORE__MOTOR_BASE_H_