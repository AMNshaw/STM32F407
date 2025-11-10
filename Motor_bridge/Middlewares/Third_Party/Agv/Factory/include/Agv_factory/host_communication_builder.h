#ifndef AGV_FACTORY__COMMUNICATION_BUILDER_H_
#define AGV_FACTORY__COMMUNICATION_BUILDER_H_

#include "Agv_core/modules/host_communication_base.h"
#include "Agv_host_communication/host_uart_config.h"

int Host_communication_uart_create(AgvHostCommunicationBase* out,
                                   const AgvHostUartCfg* cfg);

int Host_communication_uart_destroy(AgvHostCommunicationBase* base);

#endif  // AGV_FACTORY__COMMUNICATION_BUILDER_H_