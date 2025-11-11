#ifndef AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_
#define AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_

#include <stdlib.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

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

#endif  // AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_