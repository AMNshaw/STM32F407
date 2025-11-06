#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/link/uart_rs485.h"
#include "Agv_communication_pack/link/uart_ttl.h"
#include "Agv_core/core.h"
#include "Agv_core/modules/host_communication_base.h"
#include "Agv_core/modules/motor_communication_base.h"
#include "Agv_host_communication/host_communication_uart.h"
#include "Agv_motor_communication/blvr.h"
#include "usart.h"

// 對 core / comm 的全域參考（先設成 NULL，之後由 init 函數設定）
static AgvCore* g_agv_core = NULL;
static AgvHostCommunicationBase* g_host_comm = NULL;
static AgvMotorCommunicationBase* g_motor_comm = NULL;

// 提供給 main / init 呼叫，把 core 傳進來
void UartIsrBridge_AttachCore(AgvCore* core) {
    g_agv_core = core;
    g_host_comm = core ? core->host_comm : NULL;
    g_motor_comm = core ? core->motors_comm : NULL;
}

// 啟動 RX DMA+IDLE：在 core 初始化完、host/motor_comm ready 後呼叫
void UartIsrBridge_Start(void) {
    if (g_host_comm) {
        CommHostUartImpl* host_impl = (CommHostUartImpl*)g_host_comm->impl;
        if (host_impl) {
            AgvCommLinkIface* host_link = &host_impl->link;
            UartTtlImpl* host_uart_impl = (UartTtlImpl*)host_link->impl;
            if (host_uart_impl && host_uart_impl->cfg &&
                host_uart_impl->cfg->huart) {
                HAL_UARTEx_ReceiveToIdle_DMA(host_uart_impl->cfg->huart,
                                             host_uart_impl->rx_buf,
                                             sizeof(host_uart_impl->rx_buf));
                __HAL_DMA_DISABLE_IT(host_uart_impl->cfg->huart->hdmarx,
                                     DMA_IT_HT);
            }
        }
    }

    if (g_motor_comm) {
        CommBlvrMotorImpl* motor_impl = (CommBlvrMotorImpl*)g_motor_comm->impl;
        if (motor_impl) {
            AgvCommLinkIface* motor_link = &motor_impl->link;
            UartRs485Impl* motor_uart_impl = (UartRs485Impl*)motor_link->impl;
            if (motor_uart_impl && motor_uart_impl->cfg &&
                motor_uart_impl->cfg->huart) {
                HAL_UARTEx_ReceiveToIdle_DMA(motor_uart_impl->cfg->huart,
                                             motor_uart_impl->rx_buf,
                                             sizeof(motor_uart_impl->rx_buf));
                __HAL_DMA_DISABLE_IT(motor_uart_impl->cfg->huart->hdmarx,
                                     DMA_IT_HT);
            }
        }
    }
}

// 這是給 ReceiveToIdle_DMA 用的 callback
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    // Host / TTL
    if (g_host_comm) {
        CommHostUartImpl* host_impl = (CommHostUartImpl*)g_host_comm->impl;
        if (host_impl) {
            AgvCommLinkIface* host_link = &host_impl->link;
            UartTtlImpl* host_uart_impl = (UartTtlImpl*)host_link->impl;

            if (host_uart_impl && host_uart_impl->cfg &&
                host_uart_impl->cfg->huart &&
                huart->Instance == host_uart_impl->cfg->huart->Instance) {
                // Size = 這次實際收到的 byte 數
                if (host_link->on_rx_rcv) {
                    host_link->on_rx_rcv(host_link, host_uart_impl->rx_buf,
                                         Size);
                }

                // 重啟下一輪 DMA+IDLE
                HAL_UARTEx_ReceiveToIdle_DMA(host_uart_impl->cfg->huart,
                                             host_uart_impl->rx_buf,
                                             sizeof(host_uart_impl->rx_buf));
                __HAL_DMA_DISABLE_IT(host_uart_impl->cfg->huart->hdmarx,
                                     DMA_IT_HT);
                return;
            }
        }
    }

    // Motor / RS485
    if (g_motor_comm) {
        CommBlvrMotorImpl* motor_impl = (CommBlvrMotorImpl*)g_motor_comm->impl;
        if (motor_impl) {
            AgvCommLinkIface* motor_link = &motor_impl->link;
            UartRs485Impl* motor_uart_impl = (UartRs485Impl*)motor_link->impl;

            if (motor_uart_impl && motor_uart_impl->cfg &&
                motor_uart_impl->cfg->huart &&
                huart->Instance == motor_uart_impl->cfg->huart->Instance) {
                if (motor_link->on_rx_rcv) {
                    motor_link->on_rx_rcv(motor_link, motor_uart_impl->rx_buf,
                                          Size);
                }

                HAL_UARTEx_ReceiveToIdle_DMA(motor_uart_impl->cfg->huart,
                                             motor_uart_impl->rx_buf,
                                             sizeof(motor_uart_impl->rx_buf));
                __HAL_DMA_DISABLE_IT(motor_uart_impl->cfg->huart->hdmarx,
                                     DMA_IT_HT);
                return;
            }
        }
    }
}
