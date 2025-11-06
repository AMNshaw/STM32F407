#ifndef AGV_MODULE_FACTORY__COMMUNICATION_BUILDER_H_
#define AGV_MODULE_FACTORY__COMMUNICATION_BUILDER_H_

#include "communication_iface.h"
#include "configs/communication_config.h"
#include "modules/host_communication_base.h"

int Communication_host_create(AgvHostCommunicationBase* out,
                              const AgvCommUartCfg* uart_cfg,
                              const AgvCommFmtCsvCfg* csv_cfg,
                              const AgvCommPrtclHostCsvCfg* host_prtcl_cfg);

int Communication_host_destroy(AgvHostCommunicationBase* base);

#endif  // AGV_MODULE_FACTORY__COMMUNICATION_BUILDER_H_