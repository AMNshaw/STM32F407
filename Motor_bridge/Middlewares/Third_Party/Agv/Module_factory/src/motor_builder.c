#include "motor_builder.h"

#include "blvr.h"
#include "communication_iface.h"

int Motor_blvr_create(AgvMotorBase* out, const AgvCommUartCfg* link_cfg,
                      const AgvCommFmtModbusRtuCfg* format_cfg,
                      const AgvCommPrtclBlvrCfg* protocol_cfg) {
    if (!out || !link_cfg || !format_cfg || !protocol_cfg) return -1;

    BlvrMotorImpl* impl = (BlvrMotorImpl*)malloc(sizeof(BlvrMotorImpl));
    if (!impl) return -2;

    int code;

    code = Link_uart_rs485_create(&impl.link_, uart_cfg);
    if (code != 0) goto fail_link;

    code = Format_modbus_create(&impl.fmt_, modbus_cfg);
    if (code != 0) goto fail_fmt;

    code = Protocol_blvr_create(&impl.prtcl_, blvr_prtcl_cfg);
    if (code != 0) goto fail_prtcl;

    out->impl = &impl;
    out->read_feedback = blvr_read;
    out->write_targets = blvr_write;

fail_prtcl:
    if (impl->fmt_.destroy) impl->fmt_.destroy(&impl->fmt_);
fail_fmt:
    if (impl->link_.destroy) impl->link_.destroy(&impl->link_);
fail_link:
    free(impl);

    return code;
}

int Motor_blvr_destroy(AgvMotorBase* base) {
    if (!base || !base->impl) return -1;

    BlvrMotorImpl* impl = (BlvrMotorImpl*)base->impl;

    if (impl->prtcl_.destroy) impl->prtcl_.destroy(&impl->prtcl_);
    if (impl->fmt_.destroy) impl->fmt_.destroy(&impl->fmt_);
    if (impl->link_.destroy) impl->link_.destroy(&impl->link_);

    base->impl = NULL;
    base->receive = NULL;
    base->send = NULL;

    free(impl);
    return 0;
}
