#ifndef AGV_CORE__CORE_H_
#define AGV_CORE__CORE_H_

#include "Agv_core/agv_types.h"
#include "Agv_core/modules/control_law_base.h"
#include "Agv_core/modules/host_communication_base.h"
#include "Agv_core/modules/kinematics_base.h"
#include "Agv_core/modules/motor_communication_base.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

typedef struct {
    AgvKinematicsBase kine;
    AgvMotorCommunicationBase motors_comm;
    AgvControlLawBase ctrl;
    AgvHostCommunicationBase host_comm;

    // 共享狀態（以 mutex 保護）
    VelCmd cmd_latest;
    WheelsVel wheel_meas;
    WheelsVel wheel_target;
    Odom odom;

    // 同步
    SemaphoreHandle_t mtx_state;

    // 任務
    TaskHandle_t task_cmd_ctrl;
    TaskHandle_t task_read_odom;
} AgvCore;

#endif  // AGV_CORE__CORE_H_