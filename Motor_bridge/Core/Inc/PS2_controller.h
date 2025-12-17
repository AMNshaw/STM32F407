#ifndef PS2_CONTROLLER_H_
#define PS2_CONTROLLER_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define PSB_SELECT 0x0001
#define PSB_L3 0x0002
#define PSB_R3 0x0004
#define PSB_START 0x0008
#define PSB_PAD_UP 0x0010
#define PSB_PAD_RIGHT 0x0020
#define PSB_PAD_DOWN 0x0040
#define PSB_PAD_LEFT 0x0080
#define PSB_L2 0x0100
#define PSB_R2 0x0200
#define PSB_L1 0x0400
#define PSB_R1 0x0800
#define PSB_TRIANGLE 0x1000
#define PSB_CIRCLE 0x2000
#define PSB_CROSS 0x4000
#define PSB_SQUARE 0x8000

#define VX_MAX 1.0f  // m/s
#define VY_MAX 1.0f  // m/s
#define VYAW_MAX 1.0f

#define VX_MIN 0.1f  // m/s
#define VY_MIN 0.1f  // m/s
#define VYAW_MIN 0.1f

typedef struct {
    float vx;
    float vy;
    float vyaw;
    uint8_t reset;
} Ps2Cmd;

typedef struct {
    SPI_HandleTypeDef* hspi;
    const GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
} Ps2CommCfg;

void PS2_init(const Ps2CommCfg* cfg);
void PS2_update(Ps2Cmd* cmd);

#endif  // PS2_CONTROLLER_H_
