#ifndef AGV_COMMUNICATION_PACK__COMM_LINK_CONFIG_H_
#define AGV_COMMUNICATION_PACK__COMM_LINK_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"

typedef enum {
    AGV_COMM_LINK_UART_TTL,
    AGV_COMM_LINK_UART_RS485
} AgvCommLinkType;

typedef struct {
    UART_HandleTypeDef* huart;
    uint32_t baudrate;
    size_t max_data_len;

    uint32_t operation_timeout_ms;

    bool auto_DE;
    GPIO_TypeDef* DE_port;
    uint16_t DE_pin;

    size_t queue_len;
} AgvCommLnkUartRs485Cfg;

typedef struct {
    UART_HandleTypeDef* huart;
    uint32_t baudrate;
    size_t max_data_len;

    uint32_t operation_timeout_ms;

    size_t queue_len;
} AgvCommLnkUartTtlCfg;

#endif  // AGV_COMMUNICATION_PACK__COMM_LINK_CONFIG_H_