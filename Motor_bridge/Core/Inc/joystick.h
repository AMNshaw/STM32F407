#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include "stm32f4xx_hal.h"

#define ADC_MID 2048.0f  // 12-bit ADC 中點
#define ADC_HALF_RANGE 2048.0f

#define JOY_DEADZONE 0.15f  // 搖桿死區：|norm| < 0.15 視為 0
#define VX_MAX 0.4f         // m/s
#define VY_MAX 0.4f         // m/s
#define VYAW_MAX 0.2f       // rad/s

#define BTN_DEBOUNCE_CNT 4   // 4 次 * 5ms = 20ms 去抖
#define IDLE_TIMEOUT_MS 500  // 超過 0.5 秒沒有活動 -> 視為無指令

typedef struct {
    float vx;         // m/s
    float vy;         // m/s
    float vyaw;       // rad/s
    uint8_t has_cmd;  // 1: 有人正在控制, 0: 沒指令
} JoystickCmd;

void Joystick_Update(JoystickCmd* out, uint32_t now_ms);
void Joystick_Init(ADC_HandleTypeDef* hadc);

#endif