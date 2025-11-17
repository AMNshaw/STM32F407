#include "joystick.h"

#include <math.h>
#include <stdio.h>

static volatile uint16_t adc_buf[2];

typedef struct {
    // ADC 原始值

    uint16_t adc_raw_x;
    uint16_t adc_raw_y;

    // 正規化後 [-1, 1]
    float adc_x_norm;
    float adc_y_norm;

    // 按鈕去抖後狀態 (0:沒按, 1:按著)
    uint8_t btn_left;
    uint8_t btn_right;

    // 內部用
    uint8_t btn_left_cnt;
    uint8_t btn_right_cnt;

    // 活動偵測
    uint32_t last_active_ms;
} JoystickState;

static JoystickState state;

void Joystick_Init(ADC_HandleTypeDef* hadc) {
    HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buf, 2);
}

static inline uint8_t read_js_left_raw(void) {
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == GPIO_PIN_RESET);  // 按下=1
}
static inline uint8_t read_js_right_raw(void) {
    return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) == GPIO_PIN_RESET);
}

static void debounce_button(uint8_t raw, uint8_t* state, uint8_t* cnt) {
    if (raw == *state) {
        // 狀態沒變，計數器清零
        *cnt = 0;
        return;
    }

    // 狀態有變，開始累積
    if (*cnt < BTN_DEBOUNCE_CNT) {
        (*cnt)++;
        if (*cnt >= BTN_DEBOUNCE_CNT) {
            *state = raw;  // 真的換狀態
            *cnt = 0;
        }
    }
}

void Joystick_Update(JoystickCmd* out, uint32_t now_ms) {
    // 1) 讀 ADC

    state.adc_raw_x = adc_buf[0];
    state.adc_raw_y = adc_buf[1];

    float x_norm = ((float)state.adc_raw_x - ADC_MID) / ADC_HALF_RANGE;
    float y_norm = ((float)state.adc_raw_y - ADC_MID) / ADC_HALF_RANGE;

    // 2) 死區
    if (fabsf(x_norm) < JOY_DEADZONE) x_norm = 0.0f;
    if (fabsf(y_norm) < JOY_DEADZONE) y_norm = 0.0f;

    // 3) 限幅
    if (x_norm > 1.0f) x_norm = 1.0f;
    if (x_norm < -1.0f) x_norm = -1.0f;
    if (y_norm > 1.0f) y_norm = 1.0f;
    if (y_norm < -1.0f) y_norm = -1.0f;

    state.adc_x_norm = x_norm;
    state.adc_y_norm = y_norm;

    // 4) 按鈕去抖
    uint8_t raw_left = read_js_left_raw();
    uint8_t raw_right = read_js_right_raw();

    debounce_button(raw_left, &state.btn_left, &state.btn_left_cnt);
    debounce_button(raw_right, &state.btn_right, &state.btn_right_cnt);

    // 5) 映射到速度
    // 注意: 你可以定義 X=前後、Y=左右 或反過來，這裡假設：
    out->vx = x_norm * VX_MAX;
    out->vy = -y_norm * VY_MAX;

    int yaw_dir = 0;
    if (state.btn_left) yaw_dir -= 1;
    if (state.btn_right) yaw_dir += 1;
    out->vyaw = (float)yaw_dir * VYAW_MAX;

    // 6) 判斷「目前有沒有在操控」
    uint8_t active = 0;
    if (fabsf(out->vx) > 0.01f || fabsf(out->vy) > 0.01f || yaw_dir != 0) {
        active = 1;
    }

    if (active) {
        state.last_active_ms = now_ms;
    }

    if ((now_ms - state.last_active_ms) > IDLE_TIMEOUT_MS) {
        // 太久沒活動，視為無指令
        out->has_cmd = 0;
        // 這裡你可以選擇要不要把速度設成 0
        out->vx = 0.0f;
        out->vy = 0.0f;
        out->vyaw = 0.0f;
    } else {
        out->has_cmd = active;
    }
}
