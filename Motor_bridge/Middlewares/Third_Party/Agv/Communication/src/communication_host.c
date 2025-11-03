#include "communication_host.h"

int Communication_host_create(AgvCommunicationBase* out,
                              const AgvUartCfg* uart_cfg,
                              const AgvCsvFmtCfg* csv_cfg,
                              const AgvHostCsvProtoCfg* host_prtcl_cfg) {
    static Host_comm_impl impl;

    int code;
    code = Link_uart_ttl_create(&impl.link_, uart_cfg);
    code = Format_csv_create(&impl.fmt_, csv_cfg);
    code = Protocol_host_csv_create(&impl.prtcl_, host_prtcl_cfg);

    out->impl = &impl;
    out->receive = receive_cmd;
    out->send = send_odom;
}

static void receive_cmd(AgvCommunicationBase* self) {}

static void send_odom(AgvCommunicationBase* self) {}