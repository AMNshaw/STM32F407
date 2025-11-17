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

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "Agv_core/error_codes/error_common.h"
#include "adc.h"
#include "agv_app.h"
#include "joystick.h"
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

osThreadId_t odomTaskHandle;
const osThreadAttr_t odomTask_attributes = {
    .name = "odomTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t hostMsgTaskHandle;
const osThreadAttr_t hostMsgTask_attributes = {
    .name = "hostMsgTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t joystickTaskHandle;
const osThreadAttr_t joystickTask_attributes = {
    .name = "joystickTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartSendOdomTask(void* argument);
void OnHostMsgTask(void* argument);
void joystickTask(void* argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void* argument);

extern void MX_USB_HOST_Init(void);
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
    defaultTaskHandle =
        osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    odomTaskHandle = osThreadNew(StartSendOdomTask, NULL, &odomTask_attributes);
    hostMsgTaskHandle =
        osThreadNew(OnHostMsgTask, NULL, &hostMsgTask_attributes);
    joystickTaskHandle =
        osThreadNew(joystickTask, NULL, &joystickTask_attributes);
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
void StartDefaultTask(void* argument) {
    /* init code for USB_HOST */
    MX_USB_HOST_Init();
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

void StartSendOdomTask(void* argument) {
    int send_count = 0;
    printf("Start sending odom ...\n");
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
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        osDelay(5000);
    }
}

void OnHostMsgTask(void* argument) {
    for (;;) {
        int code =
            s_agv_core->host_communication_base.process_pending_msg_to_buffer(
                &s_agv_core->host_communication_base);
    }
}

void joystickTask(void* argument) {
    Joystick_Init(&hadc1);
    JoystickCmd cmd;
    printf("Start updating joystick...\n");
    for (;;) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        Joystick_Update(&cmd, now);

        // if (cmd.has_cmd) printf("cmd: %f, %f, %f \n", cmd.vx, cmd.vy,
        // cmd.vyaw);

        osDelay(20);  // 100 Hz
    }
}

/* USER CODE END Application */
