#ifndef AGV_CORE__CORE_H_
#define AGV_CORE__CORE_H_

#include "agv_types.h"
#include "communication_base.h"
#include "control_law_base.h"
#include "kinematics_base.h"
#include "motor_base.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

typedef struct AgvCore {
    AgvKinematicsBase* kine;
    AgvMotorBase* motors;
    AgvControlLawBase* ctrl;  // 可為 NULL 表直通
    AgvCommunicationBase* comm;

    // 共享狀態（以 mutex 保護）
    VelCmd cmd_latest;
    WheelVel wheel_meas;
    WheelVel wheel_target;
    Odom odom;

    // 同步
    SemaphoreHandle_t mtx_state;

    // 任務
    TaskHandle_t task_cmd_ctrl;
    TaskHandle_t task_read_odom;
} AgvCore;

void AgvCore(AgvCore* agv, AgvKinematicsBase* kine, AgvMotorBase* motors,
             AgvControlLawBase* ctrl, AgvCommunicationBase* comm);

void AgvCore(AgvCore* agv, UbaseType_t prio_cmd, UbaseType_t prio_odom);

#endif  // AGV_CORE__CORE_H_