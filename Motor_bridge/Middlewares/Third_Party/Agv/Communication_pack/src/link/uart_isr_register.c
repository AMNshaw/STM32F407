#include "Agv_communication_pack/link/uart_isr_register.h"

#include <stdio.h>

#ifndef TTL_MAX_LINKS
#define TTL_MAX_LINKS 4
#endif

typedef struct {
    UART_HandleTypeDef* huart;
    UartIdleHandler handler;
    void* ctx;
} LinkRegistryItem;

static LinkRegistryItem s_link_reg[TTL_MAX_LINKS] = {0};

int UartIsr_Register(UART_HandleTypeDef* huart, UartIdleHandler handler,
                     void* ctx) {
    if (!huart || !handler) return -1;
    for (int i = 0; i < TTL_MAX_LINKS; ++i) {
        if (s_link_reg[i].huart == NULL) {
            s_link_reg[i].huart = huart;
            s_link_reg[i].handler = handler;
            s_link_reg[i].ctx = ctx;
            return 0;
        }
    }
    return -2;  // 滿了
}

int UartIsr_Unregister(UART_HandleTypeDef* huart, UartIdleHandler handler,
                       void* ctx) {
    if (!huart || !handler) return -1;
    for (int i = 0; i < TTL_MAX_LINKS; ++i) {
        if (s_link_reg[i].huart == huart && s_link_reg[i].handler == handler &&
            s_link_reg[i].ctx == ctx) {
            s_link_reg[i].huart = NULL;
            s_link_reg[i].handler = NULL;
            s_link_reg[i].ctx = NULL;
            return 0;
        }
    }
    return -3;  // 找不到
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    for (int i = 0; i < TTL_MAX_LINKS; ++i) {
        if (s_link_reg[i].huart &&
            s_link_reg[i].huart->Instance == huart->Instance &&
            s_link_reg[i].handler) {
            s_link_reg[i].handler(s_link_reg[i].ctx, huart, Size);
        }
    }
}
