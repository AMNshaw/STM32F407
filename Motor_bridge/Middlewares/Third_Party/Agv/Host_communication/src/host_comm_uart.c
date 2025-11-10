#include "Agv_host_communication/host_comm_uart.h"

#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_communication_pack/format/ros_format.h"
#include "Agv_communication_pack/link/uart_ttl.h"
#include "Agv_communication_pack/protocol/host_protocol.h"

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

static int Communication_host_destroy(AgvHostCommunicationBase* base) {
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

static int uart_get_host_cmd(AgvHostCommunicationBase* self, HostMsg* out_msg) {
    CommHostUartImpl* impl = (CommHostUartImpl*)self->impl;
    if (!impl) return -1;

    // Convenience pointers to the composed communication interfaces
    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    if (!link || !fmt || !prtcl) return -1;

    // Concrete implementations behind the generic interfaces
    UartTtlImpl* uart_impl = (UartTtlImpl*)impl->link.impl;
    RosFmtImpl* rosFmt_impl = (RosFmtImpl*)impl->fmt.impl;
    HostProtoImpl* host_prtcl_impl = (HostProtoImpl*)impl->prtcl.impl;
    if (!uart_impl || !rosFmt_impl || !host_prtcl_impl) return -1;

    // Receive raw bytes from the UART link
    size_t buff_size = uart_impl->cfg->max_buffer_size;
    uint8_t buf[buff_size];
    int len = link->recv_bytes(link, buf, buff_size);
    if (len <= 0) return len;

    // Feed raw bytes into the Modbus frame formatter
    fmt->feed(fmt, buf, (size_t)len);
    // Extract complete Modbus RTU frames from the formatter
    size_t frame_len = rosFmt_impl->cfg->frame_len;
    uint8_t frame[frame_len];
    while (1) {
        if (fmt->pop_frame(fmt, frame, &frame_len) != 0) {
            // No more complete frames available in the buffer
            break;
        }
        // For each decoded frame, feed it into the protocol layer
        prtcl->feed_frame(prtcl, frame, frame_len);
    }

    // Drain protocol messages and look for BLVR RPM command messages
    AgvCommMsg msg;
    int got_cmd = 0;
    while (prtcl->pop_msg(prtcl, &msg) == 0) {
        switch (msg.type) {
            case AGV_COMM_MSG_CMD_VEL:
                *out = msg.data.cmd_vel;
                got_cmd = 1;
                break;

            default:
                break;
        }
    }

    return got_cmd ? 1 : 0;

    return 0;
}

static int uart_send_odom(AgvHostCommunicationBase* self, const Odom* in) {
    if (!in) return -1;
    CommHostUartImpl* impl = (CommHostUartImpl*)self->impl;
    if (!impl) return -1;
    // Convenience pointers to the composed communication interfaces
    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    if (!link || !fmt || !prtcl) return -1;

    // Concrete implementations behind the generic interfaces
    UartTtlImpl* uart_impl = (UartTtlImpl*)impl->link.impl;
    RosFmtImpl* rosFmt_impl = (RosFmtImpl*)impl->fmt.impl;
    HostProtoImpl* host_prtcl_impl = (HostProtoImpl*)impl->prtcl.impl;
    if (!uart_impl || !rosFmt_impl || !host_prtcl_impl) return -1;

    AgvCommMsg msg;
    msg.type = AGV_COMM_MSG_ODOM;
    msg.data.odom = *in;

    uint8_t frame[128];
    size_t frame_len = sizeof(frame);
    int code = prtcl->build_frame(prtcl, &msg, frame, &frame_len);
    if (code != 0) return code;

    uint8_t raw[256];
    size_t raw_len = sizeof(raw);
    code = fmt->make_frame(fmt, frame, frame_len, raw, &raw_len);
    if (code != 0) return code;

    return link->send_bytes(link, raw, raw_len);
}
