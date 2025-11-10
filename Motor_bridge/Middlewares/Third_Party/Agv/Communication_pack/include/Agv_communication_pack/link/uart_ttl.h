#ifndef AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_
#define AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_

#include <stdlib.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
    const AgvCommLnkUartCfg* cfg;

    uint8_t* rx_buf;
    size_t rx_len;

    SemaphoreHandle_t rx_mutex;
} UartTtlImpl;

static int send_bytes_ttl(AgvCommLinkIface*, const uint8_t*, size_t);

static int receive_bytes_ttl(AgvCommLinkIface*, uint8_t*, size_t);

static int on_rx_rcv_ttl(AgvCommLinkIface* iface, uint8_t* buf, size_t len);

static int read_rx_buff_ttl(AgvCommLinkIface* iface, uint8_t* out_buf);

static int destroy_ttl(AgvCommLinkIface* iface);

#endif  // AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_