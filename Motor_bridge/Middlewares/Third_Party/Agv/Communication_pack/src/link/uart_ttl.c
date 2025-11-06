#include "link/uart_ttl.h"

int Link_uart_ttl_create(AgvCommLinkIface* out, const AgvCommUartCfg* cfg) {
    if (!out || !cfg || !cfg->huart) return -1;

    UartTtlImpl* impl = (UartTtlImpl*)malloc(sizeof(UartTtlImpl));
    if (!impl) return -2;

    impl->cfg = cfg;

    out->impl = impl;
    out->send_bytes = send_bytes_ttl;
    out->recv_bytes = recv_bytes_ttl;
    out->destroy = destroy_ttl;

    return 0;
}

static int send_bytes_ttl(AgvCommLinkIface* iface, const uint8_t* data,
                          size_t len) {
    if (!iface || !iface->impl || !data || len == 0) return -1;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (!impl->cfg || !impl->cfg->huart) return -2;

    UART_HandleTypeDef* huart = impl->cfg->huart;

    HAL_StatusTypeDef st =
        HAL_UART_Transmit(huart, (uint8_t*)data, (uint16_t)len, 1000);

    if (st != HAL_OK) return -3;

    return (int)len;
}

static int recv_bytes_ttl(AgvCommLinkIface* iface, uint8_t* buf, size_t len) {
    if (!iface || !iface->impl || !buf || len == 0) return -1;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (!impl->cfg || !impl->cfg->huart) return -2;

    UART_HandleTypeDef* huart = impl->cfg->huart;

    HAL_StatusTypeDef st = HAL_UART_Receive(huart, buf, (uint16_t)len, 1000);
    if (st != HAL_OK) return -3;

    return (int)len;
}

static int destroy_ttl(AgvCommLinkIface* iface) {
    if (!iface) return -1;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (impl) {
        free(impl);
    }

    iface->impl = NULL;
    iface->send_bytes = NULL;
    iface->recv_bytes = NULL;
    iface->destroy = NULL;
    return 0;
}