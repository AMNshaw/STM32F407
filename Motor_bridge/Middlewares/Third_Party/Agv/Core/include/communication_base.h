#ifndef AGV_CORE__COMMUNICATION_BASE_H_
#define AGV_CORE__COMMUNICATION_BASE_H_

#include "communication_config.h"
#include "types.h"

typedef struct AgvCommunicationBase {
    int (*receive)(struct AgvCommunicationBase* self);
    int (*send)(struct AgvCommunicationBase* self);
    void* impl;
} AgvCommunicationBase;

int Communication_host_create(AgvCommunicationBase* out,
                              const AgvUartCfg* uart_cfg,
                              const AgvCsvFmtCfg* csv_cfg,
                              const AgvHostCsvProtoCfg* host_prtcl_cfg);

int Communication_motor_create(AgvCommunicationBase* out,
                               const AgvUartCfg* link_cfg,
                               const AgvModbusRtuFmtCfg* format_cfg,
                               const AgvBlvrProtoCfg* protocol_cfg);

#endif  // AGV_CORE__HOST_COMMUNICATION_BASE_H_