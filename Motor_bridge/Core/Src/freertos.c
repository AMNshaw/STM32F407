/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/utils.h"
#include "PS2_controller.h"
#include "agv_app.h"
#include "spi.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static AgvCore* s_agv_core = NULL;

osThreadId_t agv_heartbeatTaskHandle;
const osThreadAttr_t agv_heartbeatTask_attributes = {
    .name = "AgvHeartbeat",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t agv_sendOdomTaskHandle;
const osThreadAttr_t agv_sendOdomTask_attributes = {
    .name = "AgvSendOdom",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t agv_hostMsgCallbackTaskHandle;
const osThreadAttr_t agv_hostMsgCallbackTask_attributes = {
    .name = "AgvHostMsgCallback",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t joystickTaskHandle;
const osThreadAttr_t joystickTask_attributes = {
    .name = "joystickTask",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t agv_motorIoTaskHandle;
const osThreadAttr_t agv_motorIoTask_attributes = {
    .name = "AgvMotorIo",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityRealtime7,
};

osThreadId_t agv_bodyControlTaskHandle;
const osThreadAttr_t agv_modyControlTask_attributes = {
    .name = "AgvBodyControl",
    .stack_size = 2048 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void AgvHeartbeatTask(void* argument);
void AgvSendOdomTask(void* argument);
void AgvHostMsgCallbackTask(void* argument);
void joystickTask(void* argument);
void AgvMotorIoTask(void* argument);
void AgvBodyControlTask(void* argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    agv_heartbeatTaskHandle =
        osThreadNew(AgvHeartbeatTask, NULL, &agv_heartbeatTask_attributes);
    agv_sendOdomTaskHandle =
        osThreadNew(AgvSendOdomTask, NULL, &agv_sendOdomTask_attributes);
    agv_hostMsgCallbackTaskHandle = osThreadNew(
        AgvHostMsgCallbackTask, NULL, &agv_hostMsgCallbackTask_attributes);
    joystickTaskHandle =
        osThreadNew(joystickTask, NULL, &joystickTask_attributes);
    agv_motorIoTaskHandle =
        osThreadNew(AgvMotorIoTask, NULL, &agv_motorIoTask_attributes);
    agv_bodyControlTaskHandle =
        osThreadNew(AgvBodyControlTask, NULL, &agv_modyControlTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

int AGV_attach_core_task(AgvCore* agv_core) {
    if (!agv_core) return -1;
    s_agv_core = agv_core;
    return 0;
}

void AgvHeartbeatTask(void* argument) {
    LOG("Task", "Start monitoring modules heartbeat");
    int code = AGV_OK;
    for (;;) {
        code = AgvCore_get_modules_state(s_agv_core);
        code = AgvCore_enable_motor(s_agv_core);
        vTaskDelay(200);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    }
}

void AgvSendOdomTask(void* argument) {
    int send_count = 0;
    LOG("Task", "Start sending odom ...");
    for (;;) {
        Odometry odom;
        odom.pose.x = 1.0;
        odom.pose.y = 2.0;
        odom.pose.yaw = 3.0;
        int code = s_agv_core->host_communication_base.send_odom(
            &s_agv_core->host_communication_base, &odom);
        if (code == AGV_OK)
            printf("Odom sent %d \n", send_count++);
        else
            printf("error code: %d", code);
        vTaskDelay(5000);
    }
}

void AgvHostMsgCallbackTask(void* argument) {
    LOG("Task", "Start processing host msg...");
    for (;;) {
        int code = AgvCore_step_on_host_msg(s_agv_core);
    }
}

void joystickTask(void* argument) {
    LOG("Task", "Start updating joystick...");
    Ps2CommCfg cfg;
    cfg.hspi = &hspi2;
    cfg.cs_port = GPIOB;
    cfg.cs_pin = GPIO_PIN_12;
    Ps2Cmd cmd;
    cmd.reset = 0;
    cmd.vx = 0;
    cmd.vy = 0;
    cmd.vyaw = 0;

    PS2_init(&cfg);

    for (;;) {
        PS2_update(&cmd);
        static int time = 0;
        time++;
        if (time == 5) {
            LOG("Joystick_task", "cmd: %f %f %f %d", cmd.vx, cmd.vy, cmd.vyaw,
                cmd.reset);
            time = 0;
        }
        if (cmd.reset) {
            AgvCore_reset_motor(s_agv_core);
        }
        Twist2D cmd_in;
        cmd_in.x = cmd.vx;
        cmd_in.y = cmd.vy;
        cmd_in.yaw = cmd.vyaw;
        AgvCore_set_cmd_vel(s_agv_core, cmd_in);
        vTaskDelay(100);  // 10 Hz
    }
}

void AgvMotorIoTask(void* argument) {
    LOG("Task", "Start motors io...");
    for (;;) {
        int code = AgvCore_step_motor_io(s_agv_core);
        if (code != AGV_OK) printf("Motor Io error code: %d\n", code);
        vTaskDelay(20);  // 100 Hz
    }
}

void AgvBodyControlTask(void* argument) {
    LOG("Task", "Start body control...");
    for (;;) {
        int code = AgvCore_step_body_control(s_agv_core);
        if (code != AGV_OK) printf("Motor Io error code: %d\n", code);
        vTaskDelay(10);
    }
}

/* USER CODE END Application */

