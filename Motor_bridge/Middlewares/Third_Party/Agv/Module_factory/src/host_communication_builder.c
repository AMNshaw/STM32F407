#include "host_communication_builder.h"

#include <stdlib.h>

#include "communication_host.h"
#include "communication_motor.h"

int Communication_host_create(AgvHostCommunicationBase* out,
                              const AgvCommUartCfg* uart_cfg,
                              const AgvCommFmtCsvCfg* csv_cfg,
                              const AgvCommPrtclHostCsvCfg* host_prtcl_cfg) {
    if (!out || !uart_cfg || !csv_cfg || !host_prtcl_cfg) return -1;

    CommHostImpl* impl = (CommHostImpl*)malloc(sizeof(CommHostImpl));
    if (!impl) return -2;

    int code;

    code = Link_uart_ttl_create(&impl->link_, uart_cfg);
    if (code != 0) goto fail_link;

    code = Format_csv_create(&impl->fmt_, csv_cfg);
    if (code != 0) goto fail_fmt;

    code = Protocol_host_csv_create(&impl->prtcl_, host_prtcl_cfg);
    if (code != 0) goto fail_prtcl;

    // 成功組裝完
    out->impl = impl;
    out->receive = uart_receive_cmd;
    out->send = uart_send_odom;
    return 0;

fail_prtcl:
    if (impl->fmt_.destroy) impl->fmt_.destroy(&impl->fmt_);
fail_fmt:
    if (impl->link_.destroy) impl->link_.destroy(&impl->link_);
fail_link:
    free(impl);

    return code;
}

int Communication_host_destroy(AgvHostCommunicationBase* base) {
    if (!base || !base->impl) return -1;

    CommHostImpl* impl = (CommHostImpl*)base->impl;

    if (impl->prtcl_.destroy) impl->prtcl_.destroy(&impl->prtcl_);
    if (impl->fmt_.destroy) impl->fmt_.destroy(&impl->fmt_);
    if (impl->link_.destroy) impl->link_.destroy(&impl->link_);

    base->impl = NULL;
    base->receive = NULL;
    base->send = NULL;

    free(impl);
    return 0;
}
