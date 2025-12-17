// 假設：SPI 已設定 Mode3, 8bit, MSB, NSS=Software
#include "PS2_controller.h"

#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#define PS2_CS_GPIO_Port GPIOC
#define PS2_CS_Pin GPIO_PIN_4
#define JOYSTICK_REF 128

typedef struct {
    TickType_t L1;
    TickType_t L2;
    TickType_t R1;
    TickType_t R2;
} ButtonPressTime;

typedef struct {
    const Ps2CommCfg* comm_cfg;
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint16_t buttons;
    float vx_curr;
    float vy_curr;
    float vyaw_curr;
    ButtonPressTime button_last_t;
} Ps2Controller;

static Ps2Controller ps2;

// Private declarations
static void ps2_xfer(const uint8_t* tx, uint8_t* rx, size_t n);

bool ps2_poll(void);

bool ps2_vibration(void);

static inline float apply_deadzone(float u, float dz);

static inline float apply_expo_k(float u, float expo);

static inline float lpf(float y, float x, float alpha);

void update_speed(void);

// helpers
static inline void ps2_start_comm(void) {
    HAL_GPIO_WritePin((GPIO_TypeDef*)ps2.comm_cfg->cs_port,
                      ps2.comm_cfg->cs_pin, GPIO_PIN_RESET);
}
static inline void ps2_stop_comm(void) {
    HAL_GPIO_WritePin((GPIO_TypeDef*)ps2.comm_cfg->cs_port,
                      ps2.comm_cfg->cs_pin, GPIO_PIN_SET);
}

bool is_button_pressed(uint16_t button) { return (ps2.buttons & button) == 0; }

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

float joystick_norm(uint8_t val) {
    float u = ((float)val - 128.0f) / 127.0f;
    return clampf(u, -1.0f, 1.0f);
}

// Public definitions
void PS2_init(const Ps2CommCfg* cfg) {
    ps2.comm_cfg = cfg;
    ps2.buttons = 0xFFFF;
    ps2.lx = JOYSTICK_REF;
    ps2.ly = JOYSTICK_REF;
    ps2.rx = JOYSTICK_REF;
    ps2.ry = JOYSTICK_REF;
    ps2.vx_curr = 0.5;
    ps2.vy_curr = 0.5;
    ps2.vyaw_curr = 0.5;
    TickType_t now = xTaskGetTickCount();
    ps2.button_last_t.L1 = now;
    ps2.button_last_t.L2 = now;
    ps2.button_last_t.R1 = now;
    ps2.button_last_t.R2 = now;

    uint8_t rx[16];

    const uint8_t enter_cfg[] = {0x01, 0x43, 0x00, 0x01, 0x00};
    ps2_xfer(enter_cfg, rx, sizeof(enter_cfg));
    vTaskDelay(50);

    const uint8_t set_mode[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00};
    ps2_xfer(set_mode, rx, sizeof(set_mode));
    vTaskDelay(50);

    const uint8_t exit_cfg[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A};
    ps2_xfer(exit_cfg, rx, sizeof(exit_cfg));
    vTaskDelay(50);
}

void PS2_update(Ps2Cmd* cmd) {
    if (!ps2_poll()) {
        cmd->vx = 0;
        cmd->vy = 0;
        cmd->vyaw = 0;
    }
    if (is_button_pressed(PSB_START)) {
        cmd->reset = 1;
        cmd->vx = 0;
        cmd->vy = 0;
        cmd->vyaw = 0;
        return;
    }
    cmd->reset = 0;

    update_speed();

    // static float vx_f = 0, vy_f = 0, vyaw_f = 0;

    float lx = joystick_norm(ps2.lx);
    float ly = joystick_norm(ps2.ly);
    float rx = joystick_norm(ps2.rx);

    lx = apply_deadzone(lx, 0.08f);
    ly = apply_deadzone(ly, 0.08f);
    rx = apply_deadzone(rx, 0.08f);

    // expo
    // lx = apply_expo_k(lx, 0.5);
    // ly = apply_expo_k(ly, 0.5);
    // rx = apply_expo_k(rx, 0.5);

    // low-pass
    // vx_f = lpf(vx_f, ly * VX_MAX, 0.2f);
    // vy_f = lpf(vy_f, -lx * VY_MAX, 0.2f);
    // vyaw_f = lpf(vyaw_f, -rx * VYAW_MAX, 0.2f);

    cmd->vx = -ly * ps2.vx_curr;
    cmd->vy = -lx * ps2.vy_curr;
    cmd->vyaw = -rx * ps2.vyaw_curr;
}

// Private definitions
static void ps2_xfer(const uint8_t* tx, uint8_t* rx, size_t n) {
    ps2_start_comm();
    for (volatile int i = 0; i < 400; ++i) {
        __NOP();
    }
    // HAL_SPI_TransmitReceive(ps2.comm_cfg->hspi, (uint8_t*)tx, rx, n, 50);
    for (size_t i = 0; i < n; ++i) {
        uint8_t t = tx[i];
        uint8_t r = 0xFF;
        HAL_SPI_TransmitReceive(ps2.comm_cfg->hspi, &t, &r, 1, 50);
        rx[i] = r;
        for (volatile int k = 0; k < 150; k++) __NOP();  // byte gap
    }
    ps2_stop_comm();
}

bool ps2_poll(void) {
    uint8_t rx[9];
    const uint8_t poll[] = {0x01, 0x42, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00};
    ps2_xfer(poll, rx, sizeof(poll));

    if (rx[2] != 0x5A) {
        printf("poll failed rx:\n");
        for (size_t i = 0; i < 9; ++i) {
            printf("%02X ", rx[i]);
        }
        printf("\n");
        return false;
    }

    uint16_t b = (uint16_t)rx[3] | ((uint16_t)rx[4] << 8);
    ps2.buttons = b;
    ps2.rx = rx[5];
    ps2.ry = rx[6];
    ps2.lx = rx[7];
    ps2.ly = rx[8];

    return true;
}

void update_speed(void) {
    TickType_t now = xTaskGetTickCount();
    bool update = false;
    if (is_button_pressed(PSB_L1) && !is_button_pressed(PSB_L2)) {
        bool can_update = now - ps2.button_last_t.L1 > (TickType_t)300;
        if (can_update) {
            ps2.vx_curr += 0.1;
            ps2.vy_curr += 0.1;
            update = true;
        }
    }
    if (is_button_pressed(PSB_L2) && !is_button_pressed(PSB_L1)) {
        bool can_update = now - ps2.button_last_t.L2 > (TickType_t)300;
        if (can_update) {
            ps2.vx_curr -= 0.1;
            ps2.vy_curr -= 0.1;
            update = true;
        }
    }
    if (is_button_pressed(PSB_R1) && !is_button_pressed(PSB_R2)) {
        bool can_update = now - ps2.button_last_t.R1 > (TickType_t)300;
        if (can_update) {
            ps2.vyaw_curr += 0.1;
            update = true;
        }
    }
    if (is_button_pressed(PSB_R2) && !is_button_pressed(PSB_R1)) {
        bool can_update = now - ps2.button_last_t.R2 > (TickType_t)300;
        if (can_update) {
            ps2.vyaw_curr -= 0.1;
            update = true;
        }
    }
    ps2.vx_curr = clampf(ps2.vx_curr, VX_MIN, VX_MAX);
    ps2.vy_curr = clampf(ps2.vy_curr, VY_MIN, VY_MAX);
    ps2.vyaw_curr = clampf(ps2.vyaw_curr, VYAW_MIN, VYAW_MAX);
    if (update) ps2_vibration();
}

bool ps2_vibration(void) {
    uint8_t rx[9];
    const uint8_t vib_cmd[] = {0x01, 0x42, 0x01, 0x40, 0x40,
                               0x00, 0x00, 0x00, 0x00};
    ps2_xfer(vib_cmd, rx, sizeof(vib_cmd));

    if (rx[2] != 0x5A) {
        printf("vib_cmd failed rx:\n");
        for (size_t i = 0; i < 9; ++i) {
            printf("%02X ", rx[i]);
        }
        printf("\n");
        return false;
    }

    return true;
}

static inline float apply_deadzone(float u, float dz) {
    if (u > -dz && u < dz) return 0.0f;
    // 讓出 deadzone 後重新縮放到 [-1,1]
    if (u > 0)
        return (u - dz) / (1.0f - dz);
    else
        return (u + dz) / (1.0f - dz);
}

static inline float apply_expo_k(float u, float expo) {
    float a = (u >= 0) ? u : -u;
    float y = (1.0f - expo) * a + expo * (a * a);  // 線性+平方混合
    return (u >= 0) ? y : -y;
}

static inline float lpf(float y, float x, float alpha) {  // alpha=0.1~0.3
    return y + alpha * (x - y);
}