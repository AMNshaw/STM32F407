#ifndef AGV_COMMUNICATION_PACK__COMM_LINK_CONFIG_H_
#define AGV_COMMUNICATION_PACK__COMM_LINK_CONFIG_H_

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
    size_t buffer_size;

    uint32_t char_time_10x_us;
} AgvCommUartCfg;

typedef struct {
    AgvCommLinkType type;
    union {
        AgvCommUartCfg uart;
        /* data */
    } u;

} AgvCommLinkCfg;

#endif  // AGV_COMMUNICATION_PACK__COMM_LINK_CONFIG_H_