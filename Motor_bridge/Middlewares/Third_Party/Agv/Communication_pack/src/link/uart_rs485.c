#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/link/uart_isr_register.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"
#include "Agv_core/utils.h"
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
} DataFrame;

typedef struct {
    const AgvCommLnkUartRs485Cfg* cfg;

    SemaphoreHandle_t tx_mutex;

    uint8_t* rx_buf;
    size_t rx_len;
    QueueHandle_t rx_data_queue;

    size_t frame_item_size;
    size_t num_dropped_data;

} UartRs485Impl;

static int LnkRs485_send_bytes(AgvCommLinkIface* iface, const uint8_t* data_in,
                               size_t data_len);

static int LnkRs485_recv_bytes(AgvCommLinkIface* iface, uint8_t* data_out,
                               size_t* data_len);

static int LnkRs485_pop_rx_queue(AgvCommLinkIface* iface, uint8_t* buf_out,
                                 size_t* buf_len_out, uint32_t* timestamp_out);

static int LnkRs485_destroy(AgvCommLinkIface* iface);

// DMA Idle callback

static void LnkRs485_idle_handler(void* ctx, UART_HandleTypeDef* huart,
                                  uint16_t size);

int LnkRs485_on_rx_rcv(AgvCommLinkIface* iface, size_t data_len);

// helpers

int set_tx_mode(const AgvCommLnkUartRs485Cfg* cfg);

int set_rx_mode(const AgvCommLnkUartRs485Cfg* cfg);

/**
 * Private definitions
 */

int Link_uart_rs485_create(AgvCommLinkIface* out,
                           const AgvCommLnkUartRs485Cfg* cfg) {
    if (!out || !cfg || !cfg->huart) return AGV_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)malloc(sizeof(UartRs485Impl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (!cfg->auto_DE && cfg->DE_port == NULL) {
        free(impl);
        return AGV_ERR_INVALID_ARG;
    }

    impl->cfg = cfg;
    impl->rx_buf = (uint8_t*)malloc(cfg->max_data_len * sizeof(uint8_t));
    if (!impl->rx_buf) {
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }
    impl->rx_len = 0;

    impl->frame_item_size = sizeof(DataFrame) + cfg->max_data_len;
    impl->rx_data_queue = xQueueCreate(cfg->queue_len, impl->frame_item_size);
    if (!impl->rx_data_queue) {
        free(impl->rx_buf);
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }
    impl->num_dropped_data = 0;

    impl->tx_mutex = xSemaphoreCreateMutex();
    if (!impl->tx_mutex) {
        vQueueDelete(impl->rx_data_queue);
        free(impl->rx_buf);
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }

    out->impl = impl;
    out->send_bytes = LnkRs485_send_bytes;
    out->recv_bytes = LnkRs485_recv_bytes;
    out->read_buf = LnkRs485_pop_rx_queue;
    out->destroy = LnkRs485_destroy;

    if (UartIsr_Register(cfg->huart, LnkRs485_idle_handler, out) != 0) {
        vSemaphoreDelete(impl->tx_mutex);
        vQueueDelete(impl->rx_data_queue);
        free(impl->rx_buf);
        free(impl);
        return AGV_ERR_COMM_LINK_HAL;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(cfg->huart, impl->rx_buf,
                                     cfg->max_data_len) != HAL_OK) {
        vSemaphoreDelete(impl->tx_mutex);
        vQueueDelete(impl->rx_data_queue);
        free(impl->rx_buf);
        free(impl);
        return AGV_ERR_COMM_LINK_HAL;
    }
    set_rx_mode(cfg);

    return AGV_OK;
}

static int LnkRs485_destroy(AgvCommLinkIface* iface) {
    if (!iface) return AGV_OK;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (impl) {
        if (impl->rx_buf) {
            free(impl->rx_buf);
            impl->rx_buf = NULL;
        }
        if (impl->rx_data_queue != NULL) {
            vQueueDelete(impl->rx_data_queue);
            impl->rx_data_queue = NULL;
        }
        if (impl->tx_mutex) {
            vSemaphoreDelete(impl->tx_mutex);
            impl->tx_mutex = NULL;
        }

        impl->cfg = NULL;
        free(impl);
    }

    iface->impl = NULL;
    iface->send_bytes = NULL;
    iface->recv_bytes = NULL;
    iface->read_buf = NULL;
    iface->destroy = NULL;

    return AGV_OK;
}

static int LnkRs485_send_bytes(AgvCommLinkIface* iface, const uint8_t* data_in,
                               size_t data_len) {
    if (!iface || !data_in || data_len == 0) return AGV_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl || !impl->cfg || !impl->cfg->huart) return AGV_ERR_NO_MEMORY;

    UART_HandleTypeDef* huart = impl->cfg->huart;

    xSemaphoreTake(impl->tx_mutex, portMAX_DELAY);
    set_tx_mode(impl->cfg);
    HAL_StatusTypeDef st =
        HAL_UART_Transmit(huart, (uint8_t*)data_in, (uint16_t)data_len,
                          impl->cfg->operation_timeout_ms);
    // set_rx_mode(impl->cfg);
    xSemaphoreGive(impl->tx_mutex);
    if (st != HAL_OK) return AGV_ERR_COMM_LINK_HAL;

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    return AGV_OK;
}

static int LnkRs485_recv_bytes(AgvCommLinkIface* iface, uint8_t* data_out,
                               size_t* data_len) {
    if (!iface || !data_out || !data_len || *data_len == 0)
        return AGV_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl || !impl->cfg || !impl->cfg->huart) return AGV_ERR_NO_MEMORY;

    UART_HandleTypeDef* huart = impl->cfg->huart;

    HAL_StatusTypeDef st =
        HAL_UART_Receive(huart, data_out, (uint16_t)(*data_len),
                         impl->cfg->operation_timeout_ms);

    size_t rx_done = huart->RxXferSize - huart->RxXferCount;
    *data_len = rx_done;

    LOG("rs485", "buf size: %d buf count: %d, done: %d", huart->RxXferSize,
        huart->RxXferCount, rx_done);

    if (st == HAL_TIMEOUT) return AGV_ERR_COMM_LINK_TIMEOUT;

    if (st != HAL_OK) return AGV_ERR_COMM_LINK_HAL;

    return AGV_OK;
}

static int LnkRs485_pop_rx_queue(AgvCommLinkIface* iface, uint8_t* buf_out,
                                 size_t* buf_len, uint32_t* timestamp_out) {
    if (!iface || !buf_out || !buf_len || !timestamp_out)
        return AGV_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    if (!impl->rx_data_queue) return AGV_ERR_NO_MEMORY;

    uint8_t raw[impl->frame_item_size];
    DataFrame* frame = (DataFrame*)raw;

    if (xQueueReceive(impl->rx_data_queue, frame,
                      pdMS_TO_TICKS(impl->cfg->operation_timeout_ms)) != pdPASS)
        return AGV_ERR_COMM_LINK_TIMEOUT;

    if (*buf_len < frame->len) return AGV_ERR_OUTPUT_OVERFLOW;

    *timestamp_out = frame->timestamp;
    *buf_len = frame->len;
    memcpy(buf_out, frame->data, *buf_len);

    return AGV_OK;
}

static void LnkRs485_idle_handler(void* ctx, UART_HandleTypeDef* huart,
                                  uint16_t size) {
    AgvCommLinkIface* link = (AgvCommLinkIface*)ctx;
    if (!link) return;
    UartRs485Impl* impl = (UartRs485Impl*)link->impl;
    if (!impl || !impl->cfg) return;

    LnkRs485_on_rx_rcv(link, size);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, impl->rx_buf, impl->cfg->max_data_len);
}

int LnkRs485_on_rx_rcv(AgvCommLinkIface* iface, size_t data_len) {
    if (!iface || data_len == 0) return AGV_ERR_INVALID_ARG;

    UartRs485Impl* impl = (UartRs485Impl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (data_len > impl->cfg->max_data_len)
        return AGV_ERR_COMM_LINK_BUFFER_OVERFLOW;

    impl->rx_len = data_len;

    uint8_t raw[impl->frame_item_size];
    DataFrame* frame = (DataFrame*)raw;

    frame->timestamp = xTaskGetTickCountFromISR();
    frame->len = data_len;
    memcpy(frame->data, impl->rx_buf, data_len);

    BaseType_t hpw = pdFALSE;
    if (xQueueSendFromISR(impl->rx_data_queue, frame, &hpw) != pdPASS) {
        ++impl->num_dropped_data;
        LOG("TaskQUeue", "dropped: %d", impl->num_dropped_data);
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        return AGV_ERR_COMM_LINK_BUFFER_OVERFLOW;
    }
    portYIELD_FROM_ISR(hpw);

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

    return AGV_OK;
}

int set_tx_mode(const AgvCommLnkUartRs485Cfg* cfg) {
    if (!cfg) return AGV_ERR_INVALID_ARG;

    if (!cfg->auto_DE)
        HAL_GPIO_WritePin(cfg->DE_port, cfg->DE_pin, GPIO_PIN_SET);

    return AGV_OK;
}

int set_rx_mode(const AgvCommLnkUartRs485Cfg* cfg) {
    if (!cfg) return AGV_ERR_INVALID_ARG;

    if (!cfg->auto_DE)
        HAL_GPIO_WritePin(cfg->DE_port, cfg->DE_pin, GPIO_PIN_RESET);

    return AGV_OK;
}