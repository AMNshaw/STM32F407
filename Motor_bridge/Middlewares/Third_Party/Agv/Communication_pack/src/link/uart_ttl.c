#include <stdlib.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

/**
 * private declarations
 */

typedef struct {
    uint32_t timestamp;
    size_t len;
    uint8_t data[];
} TtlFrame;

typedef struct {
    const AgvCommLnkUartTtlCfg* cfg;

    uint8_t* rx_buf;
    size_t rx_len;
    QueueHandle_t rx_data_queue;

    size_t frame_item_size;
    size_t num_dropped_data;

    // SemaphoreHandle_t rx_mutex; FreeRTOS will handle the semaphore of queue
} UartTtlImpl;

static int send_bytes_ttl(AgvCommLinkIface* iface, const uint8_t* data_in,
                          size_t data_len);

static int recv_bytes_ttl(AgvCommLinkIface* iface, uint8_t* data_out,
                          size_t* data_len);

static int on_rx_rcv_ttl(AgvCommLinkIface* iface, size_t data_len);

static int pop_rx_queue_ttl(AgvCommLinkIface* iface, uint8_t* buf_out,
                            size_t* buf_len_out, uint32_t* timestamp_out);

static int destroy_ttl(AgvCommLinkIface* iface);

/**
 * Private definitions
 */

int Link_uart_ttl_create(AgvCommLinkIface* out,
                         const AgvCommLnkUartTtlCfg* cfg) {
    if (!out || !cfg || !cfg->huart) return AGV_ERR_INVALID_ARG;

    UartTtlImpl* impl = (UartTtlImpl*)malloc(sizeof(UartTtlImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->cfg = cfg;
    impl->rx_buf = (uint8_t*)malloc(cfg->max_data_len * sizeof(uint8_t));
    if (!impl->rx_buf) {
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }
    impl->rx_len = 0;

    impl->frame_item_size = sizeof(TtlFrame) + cfg->max_data_len;
    impl->rx_data_queue = xQueueCreate(cfg->queue_len, impl->frame_item_size);
    if (!impl->rx_data_queue) {
        free(impl->rx_buf);
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }
    impl->num_dropped_data = 0;

    out->impl = impl;
    out->send_bytes = send_bytes_ttl;
    out->recv_bytes = recv_bytes_ttl;
    out->on_data_rcv = on_rx_rcv_ttl;
    out->read_buf = pop_rx_queue_ttl;
    out->destroy = destroy_ttl;

    if (HAL_UARTEx_ReceiveToIdle_DMA(cfg->huart, impl->rx_buf,
                                     cfg->max_data_len) != HAL_OK) {
        vQueueDelete(impl->rx_data_queue);
        free(impl->rx_buf);
        free(impl);
        return AGV_ERR_COMM_LINK_HAL;
    }

    return AGV_OK;
}

static int destroy_ttl(AgvCommLinkIface* iface) {
    if (!iface) return AGV_OK;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (impl) {
        if (impl->rx_buf) {
            free(impl->rx_buf);
            impl->rx_buf = NULL;
        }
        if (impl->rx_data_queue != NULL) {
            vQueueDelete(impl->rx_data_queue);
            impl->rx_data_queue = NULL;
        }
        impl->cfg = NULL;
        free(impl);
    }

    iface->impl = NULL;
    iface->send_bytes = NULL;
    iface->recv_bytes = NULL;
    iface->on_data_rcv = NULL;
    iface->read_buf = NULL;
    iface->destroy = NULL;
    return AGV_OK;
}

static int send_bytes_ttl(AgvCommLinkIface* iface, const uint8_t* data_in,
                          size_t data_len) {
    if (!iface || !iface->impl || !data_in || data_len == 0)
        return AGV_ERR_INVALID_ARG;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    const AgvCommLnkUartTtlCfg* cfg = impl->cfg;
    if (!cfg || !cfg->huart) return AGV_ERR_NO_MEMORY;

    UART_HandleTypeDef* huart = cfg->huart;

    HAL_StatusTypeDef st = HAL_UART_Transmit(huart, data_in, (uint16_t)data_len,
                                             cfg->operation_timeout_ms);

    if (st != HAL_OK) return AGV_ERR_COMM_LINK_HAL;

    return AGV_OK;
}

static int recv_bytes_ttl(AgvCommLinkIface* iface, uint8_t* data_out,
                          size_t* data_len) {
    if (!iface || !iface->impl || !data_out || !data_len || *data_len == 0)
        return AGV_ERR_INVALID_ARG;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    const AgvCommLnkUartTtlCfg* cfg = impl->cfg;
    if (!cfg || !cfg->huart) return AGV_ERR_NO_MEMORY;

    UART_HandleTypeDef* huart = cfg->huart;

    HAL_StatusTypeDef st = HAL_UART_Receive(
        huart, data_out, (uint16_t)(*data_len), cfg->operation_timeout_ms);

    if (st != HAL_OK) return AGV_ERR_COMM_LINK_HAL;

    return AGV_OK;
}

static int on_rx_rcv_ttl(AgvCommLinkIface* iface, size_t data_len) {
    if (!iface || data_len == 0) return AGV_ERR_INVALID_ARG;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (data_len > impl->cfg->max_data_len)
        return AGV_ERR_COMM_LINK_BUFFER_OVERFLOW;

    impl->rx_len = data_len;

    uint8_t raw[impl->frame_item_size];
    TtlFrame* frame = (TtlFrame*)raw;

    frame->timestamp = xTaskGetTickCountFromISR();
    frame->len = data_len;
    memcpy(frame->data, impl->rx_buf, data_len);

    BaseType_t hpw = pdFALSE;
    if (xQueueSendFromISR(impl->rx_data_queue, frame, &hpw) != pdPASS) {
        ++impl->num_dropped_data;
        return AGV_ERR_COMM_LINK_BUFFER_OVERFLOW;
    }

    portYIELD_FROM_ISR(hpw);
    return AGV_OK;
}

static int pop_rx_queue_ttl(AgvCommLinkIface* iface, uint8_t* buf_out,
                            size_t* buf_len, uint32_t* timestamp_out) {
    if (!iface || !buf_out || !buf_len || !timestamp_out)
        return AGV_ERR_INVALID_ARG;

    UartTtlImpl* impl = (UartTtlImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    if (!impl->rx_data_queue) return AGV_ERR_NO_MEMORY;

    uint8_t raw[impl->frame_item_size];
    TtlFrame* frame = (TtlFrame*)raw;

    if (xQueueReceive(impl->rx_data_queue, frame, portMAX_DELAY) != pdPASS)
        return AGV_ERR_COMM_LINK_RX_EMPTY;

    if (*buf_len < frame->len) return AGV_ERR_OUTPUT_OVERFLOW;

    *timestamp_out = frame->timestamp;
    *buf_len = frame->len;
    memcpy(buf_out, frame->data, *buf_len);

    return AGV_OK;
}
