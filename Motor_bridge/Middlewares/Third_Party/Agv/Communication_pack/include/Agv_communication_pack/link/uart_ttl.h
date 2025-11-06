#ifndef AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_
#define AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_

#include <stdlib.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_link_config.h"

typedef struct uart_ttl {
    const AgvCommUartCfg* cfg;
} UartTtlImpl;

static int send_bytes_ttl(AgvCommLinkIface*, const uint8_t*, size_t);

static int receive_bytes_ttl(AgvCommLinkIface*, uint8_t*, size_t);

static int destroy_ttl(AgvCommLinkIface* iface);

#endif  // AGV_COMMUNICATION_PACK__LINK_UART_TTL_H_