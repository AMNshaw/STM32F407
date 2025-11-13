#ifndef AGV_COMMUNICATION_PACK__UART_ISR_REGISTER_H_
#define AGV_COMMUNICATION_PACK__UART_ISR_REGISTER_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UartIdleHandler)(void* ctx, UART_HandleTypeDef* huart,
                                uint16_t size);

// 註冊（huart 對應的一個處理器；允許多路 huart）
int UartIsr_Register(UART_HandleTypeDef* huart, UartIdleHandler handler,
                     void* ctx);
// 解除註冊
int UartIsr_Unregister(UART_HandleTypeDef* huart, UartIdleHandler handler,
                       void* ctx);

#ifdef __cplusplus
}
#endif
#endif  // AGV_COMMUNICATION_PACK__UART_ISR_REGISTER_H_
