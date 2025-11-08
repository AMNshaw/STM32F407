#ifndef AGV_MODULE_FACTORY__COMMUNICATION_BUILDER_H_
#define AGV_MODULE_FACTORY__COMMUNICATION_BUILDER_H_

#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "Agv_core/modules/host_communication_base.h"

int Host_communication_uart_create(
    AgvHostCommunicationBase* out, const AgvCommUartCfg* uart_cfg,
    const AgvCommFmtCsvCfg* csv_cfg,
    const AgvCommPrtclHostCsvCfg* host_prtcl_cfg);

int Host_communication_uart_destroy(AgvHostCommunicationBase* base);

#endif  // AGV_MODULE_FACTORY__COMMUNICATION_BUILDER_H_