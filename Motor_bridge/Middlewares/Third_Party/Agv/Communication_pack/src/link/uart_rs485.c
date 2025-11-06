#include "link/uart_rs485.h"

int Link_uart_rs485_create(AgvCommLinkIface* out, const AgvCommUartCfg* cfg) {
    if (!out || !cfg || !cfg->huart) return -1;

    UartRs485Impl* impl = (UartRs485Impl*)malloc(sizeof(UartRs485Impl));
    if (!impl) return -2;

    impl->cfg = cfg;

    out->impl = impl;
    out->send_bytes = send_bytes_rs485;
    out->recv_bytes = recv_bytes_rs485;
    out->destroy = destroy_rs485;

    return 0;
}

static int send_bytes_rs485(AgvCommLinkIface* iface, const uint8_t* data,
                            size_t len) {
    if (!iface || !iface->impl || !data || len == 0) return -1;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl->cfg || !impl->cfg->huart) return -2;

    UART_HandleTypeDef* huart = impl->cfg->huart;

    HAL_StatusTypeDef st =
        HAL_UART_Transmit(huart, (uint8_t*)data, (uint16_t)len, 1000);

    if (st != HAL_OK) return -3;

    return (int)len;
}

static int recv_bytes_rs485(AgvCommLinkIface* iface, uint8_t* buf, size_t len) {
    if (!iface || !iface->impl || !buf || len == 0) return -1;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl->cfg || !impl->cfg->huart) return -2;

    UART_HandleTypeDef* huart = impl->cfg->huart;

    HAL_StatusTypeDef st = HAL_UART_Receive(huart, buf, (uint16_t)len,
                                            1000  // timeout 同樣可做成參數
    );
    if (st != HAL_OK) return -3;

    // 這裡是 blocking 收，會一直等到收滿 len bytes 或 timeout。
    return (int)len;
}

static int destroy_rs485(AgvCommLinkIface* iface) {
    if (!iface) return -1;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (impl) {
        free(impl);
    }

    iface->impl = NULL;
    iface->send_bytes = NULL;
    iface->recv_bytes = NULL;
    iface->destroy = NULL;
    return 0;
}