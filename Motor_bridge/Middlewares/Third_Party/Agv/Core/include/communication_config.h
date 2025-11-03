#ifndef AGV_CORE__COMMUNICATION_CONFIG_H_
#define AGV_CORE__COMMUNICATION_CONFIG_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* ---------- Electrical 層 ---------- */

typedef struct {
    UART_HandleTypeDef* huart;
    uint32_t interframe_chars_x10;  // Modbus RTU: 3.5 chars = 35
} AgvUartCfg;

/* ---------- Format 層 ---------- */

typedef struct {
    uint8_t newline;
} AgvCsvFmtCfg;

typedef struct {
    uint8_t placeholder;
} AgvModbusRtuFmtCfg;

/* ---------- Protocol 層 ---------- */

typedef struct {
    uint8_t placeholder;
} AgvHostCsvProtoCfg;

typedef struct {
    uint8_t addr[4];
    float vel_scale;
} AgvBlvrProtoCfg;

#endif  // AGV_CORE__COMMUNICATION_CONFIG_H_