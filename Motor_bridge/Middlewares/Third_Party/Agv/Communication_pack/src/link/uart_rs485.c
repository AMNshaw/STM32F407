#include "Agv_communication_pack/link/uart_rs485.h"

#include "Agv_communication_pack/exception_codes.h"

int Link_uart_rs485_create(AgvCommLinkIface* out, const AgvCommUartCfg* cfg) {
    if (!out || !cfg) return AGV_COMM_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)malloc(sizeof(UartRs485Impl));
    if (!impl) return AGV_COMM_ERR_NO_MEMORY;

    impl->cfg = cfg;
    impl->rx_buf = (uint8_t*)malloc(impl->cfg->max_buffer_size);
    if (!impl->rx_buf) {
        free(impl);
        return AGV_COMM_ERR_NO_MEMORY;
    }
    impl->rx_len = 0;
    impl->rx_mutex = xSemaphoreCreateMutex();
    if (impl->rx_mutex == NULL) {
        Free(impl->rx_buf);
        Free(impl);
        return AGV_COMM_ERR_MUTEX_FAIL;
    }
    impl->last_rx_time_us = 0;

    out->impl = impl;
    out->send_bytes = send_bytes_rs485;
    out->recv_bytes = recv_bytes_rs485;
    out->on_buf_rcv = on_rx_rcv_rs485;
    out->read_buf = read_rx_buff_rs485;
    out->destroy = destroy_rs485;

    if (!cfg->huart) return AGV_COMM_ERR_NO_MEMORY;

    HAL_UARTEx_ReceiveToIdle_DMA(cfg->huart, impl->rx_buf,
                                 cfg->max_buffer_size);

    return AGV_COMM_OK;
}

static int destroy_rs485(AgvCommLinkIface* iface) {
    if (!iface) return AGV_COMM_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl) return AGV_COMM_ERR_NO_MEMORY;

    if (impl->rx_buf) {
        free(impl->rx_buf);
    }
    if (impl->rx_mutex) {
        vSemaphoreDelete(impl->rx_mutex);
    }
    free(impl);

    iface->impl = NULL;
    iface->send_bytes = NULL;
    iface->recv_bytes = NULL;
    iface->on_buf_rcv = NULL;
    iface->read_buf = NULL;
    iface->destroy = NULL;

    return AGV_COMM_OK;
}

static int send_bytes_rs485(AgvCommLinkIface* iface, const uint8_t* data_in,
                            size_t data_len) {
    if (!iface || !data_in || data_len == 0) return AGV_COMM_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl || !impl->cfg || impl->cfg->huart) return AGV_COMM_ERR_NO_MEMORY;

    UART_HandleTypeDef* huart = cfg->huart;

    HAL_StatusTypeDef st =
        HAL_UART_Transmit(huart, (uint8_t*)data_in, (uint16_t)data_len, 1000);

    if (st != HAL_OK) return AGV_COMM_ERR_LINK_HAL;

    return (int)data_len;
}

static int recv_bytes_rs485(AgvCommLinkIface* iface, uint8_t* data_out,
                            size_t data_size) {
    if (!iface || !data_out || data_size == 0) return AGV_COMM_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl || !impl->cfg || !impl->cfg->huart) return AGV_COMM_ERR_NO_MEMORY;

    UART_HandleTypeDef* huart = impl->cfg->huart;

    HAL_StatusTypeDef st =
        HAL_UART_Receive(huart, data_out, (uint16_t)data_size, 1000);

    if (st != HAL_OK) return AGV_COMM_ERR_LINK_HAL;

    return (int)data_size;
}

static int on_rx_rcv_rs485(AgvCommLinkIface* iface, uint8_t* buf_in,
                           size_t buf_len) {
    if (!iface || !buf_in || buf_len == 0) return AGV_COMM_ERR_INVALID_ARG;
    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl || !impl->cfg) return AGV_COMM_ERR_NO_MEMORY;

    if (xSemaphoreTake(impl->rx_mutex, portMAX_DELAY) != pdTRUE) {
        return AGV_COMM_ERR_MUTEX_FAIL;
    }

    if (buf_len > impl->cfg->max_buffer_size) {
        impl->rx_len = 0;
        xSemaphoreGive(impl->rx_mutex);
        return AGV_COMM_ERR_LINK_BUFFER_OVERFLOW;
    }
    memcpy(impl->rx_buf, buf_in, buf_len);
    impl->rx_len = buf_len;

    xSemaphoreGive(impl->rx_mutex);
    return (int)buf_len;
}

static int read_rx_buff_rs485(AgvCommLinkIface* iface, uint8_t* out_buf,
                              size_t max_out_size) {
    if (!iface || !out_buf || max_out_size == 0)
        return AGV_COMM_ERR_INVALID_ARG;
    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl || !impl->cfg) return AGV_COMM_ERR_NO_MEMORY;

    if (xSemaphoreTake(impl->rx_mutex, portMAX_DELAY) != pdTRUE) {
        return AGV_COMM_ERR_MUTEX_FAIL;
    }
    size_t out_len = impl->rx_len;
    if (out_len == 0) {
        xSemaphoreGive(impl->rx_mutex);
        return AGV_COMM_ERR_LINK_RX_EMPTY;  // 沒資料
    }

    if (out_len > max_out_size) return AGV_COMM_ERR_LINK_BUFFER_OVERFLOW;

    memcpy(out_buf, impl->rx_buf, out_len);
    impl->rx_len = 0;

    xSemaphoreGive(impl->rx_mutex);
    return (int)out_len;
}
