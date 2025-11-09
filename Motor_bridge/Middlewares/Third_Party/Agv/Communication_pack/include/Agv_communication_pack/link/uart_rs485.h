#ifndef AGV_COMMUNICATION_PACK__LINK_UART_RS485_H_
#define AGV_COMMUNICATION_PACK__LINK_UART_RS485_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

typedef struct uart_rs485 {
    const AgvCommLnkUartCfg* cfg;

    uint8_t* rx_buf;
    size_t rx_len;
    uint32_t last_rx_time_us;

    SemaphoreHandle_t rx_mutex;
} UartRs485Impl;

static int send_bytes_rs485(AgvCommLinkIface* iface, const uint8_t* data,
                            size_t len);

static int recv_bytes_rs485(AgvCommLinkIface* iface, uint8_t* buf, size_t len);

static int destroy_rs485(AgvCommLinkIface* iface);

#endif  // AGV_COMMUNICATION_PACK__LINK_UART_RS485_H_