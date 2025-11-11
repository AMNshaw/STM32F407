#include "Agv_communication_pack/link/uart_ttl.h"
#include "Agv_host_communication/ros_host.h"
#include "agv_app.h"
#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    CommHostRosImpl* host_impl =
        (CommHostRosImpl*)agv_core.host_communication_base.impl;
    if (!host_impl) return;

    AgvCommLinkIface* link = &host_impl->link;

    UartTtlImpl* uart_ttl_impl = (UartTtlImpl*)link->impl;
    if (!uart_ttl_impl) return;

    const AgvCommLnkUartTtlCfg* cfg = uart_ttl_impl->cfg;
    if (!cfg) return;
    if (huart != cfg->huart) return;

    uint8_t* buf = uart_ttl_impl->rx_buf;
    if (!buf) return;
    size_t max_data_len = cfg->max_data_len;

    if (link->on_data_rcv) {
        link->on_data_rcv(link, Size);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, max_data_len);
}