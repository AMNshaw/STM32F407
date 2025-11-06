#include "Agv_motor_communication/blvr.h"

#include "Agv_communication_pack/communication_msg.h"
#include "Agv_communication_pack/format/modbus_rtu_format.h"
#include "Agv_communication_pack/link/uart_rs485.h"
#include "Agv_communication_pack/protocol/blvr_protocol.h"

static int blvr_read(AgvMotorCommunicationBase* self, WheelVel* out) {
    // Convenience pointers to the composed communication interfaces
    BlvrMotorImpl* impl = (BlvrMotorImpl*)self->impl;
    if (!impl) return -1;
    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    if (!link || !fmt || !prtcl) return -1;

    // Concrete implementations behind the generic interfaces
    UartRs485Impl* rs485_impl = (UartRs485Impl*)impl->link.impl;
    ModbusRtuFmtImpl* modbus_impl = (ModbusRtuFmtImpl*)impl->fmt.impl;
    BlvrPrtclImpl* blvr_prtcl_impl = (BlvrPrtclImpl*)impl->prtcl.impl;
    if (!rs485_impl || !modbus_impl || !blvr_prtcl_impl) return -1;

    // Receive raw bytes from the UART link
    size_t buff_size = rs485_impl->cfg->buffer_size;
    uint8_t buf[buff_size];
    int len = link->recv_bytes(link, buf, buff_size);
    if (len <= 0) return len;

    uint32_t gap_threshold_us =
        rs485_impl->char_time_10x_us * modbus_impl->cfg->interframe_chars_x10;

    if (rs485_impl->rtu_rx_len > 0) {
        uint32_t dt = now_us - rs485_impl->last_rx_time_us;
        if (dt >= gap_threshold_us) {
            // gap 夠長 => 前面的當作一個 Modbus frame 丟進 formatter
            fmt->feed(fmt, rs485_impl->rtu_rx_buf, rs485_impl->rtu_rx_len);
            rs485_impl->rtu_rx_len = 0;
        }
        if (len > 0) {
            size_t copy_n = (size_t)len;
            // clang-format off
            if (copy_n > sizeof(rs485_impl->rtu_rx_buf) - rs485_impl->rtu_rx_len) {
                copy_n = sizeof(rs485_impl->rtu_rx_buf) - rs485_impl->rtu_rx_len;
            }
            // clang-format on
            memcpy(&rs485_impl->rtu_rx_buf[rs485_impl->rtu_rx_len], buf,
                   copy_n);
            rs485_impl->rtu_rx_len += copy_n;
            rs485_impl->last_rx_time_us = now_us;
        }
    }

    // Extract complete Modbus RTU frames from the formatter
    size_t max_frame_size = modbus_impl->cfg->max_frame_size;
    uint8_t frame[max_frame_size];
    while (1) {
        size_t frame_len = max_frame_size;
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
            case AGV_COMM_MSG_BLVR_RPM:
                for (int i = 0; i < 4; i++) {
                    out->w[i] = msg.data.blvr_op[i].rpm;
                }
                got_cmd = 1;
                break;

            default:
                break;
        }
    }

    return got_cmd ? 1 : 0;
}

static int blvr_write(AgvMotorCommunicationBase* self, const WheelVel* in) {
    if (!in) return -1;

    // Convenience pointers to the composed communication interfaces
    BlvrMotorImpl* impl = (BlvrMotorImpl*)self->impl;
    if (!impl) return -1;
    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    if (!link || !fmt || !prtcl) return -1;

    // Concrete implementations behind the generic interfaces
    UartRs485Impl* rs485_impl = (UartRs485Impl*)impl->link.impl;
    ModbusRtuFmtImpl* modbus_impl = (ModbusRtuFmtImpl*)impl->fmt.impl;
    BlvrPrtclImpl* blvr_prtcl_impl = (BlvrPrtclImpl*)impl->prtcl.impl;
    if (!rs485_impl || !modbus_impl || !blvr_prtcl_impl) return -1;

    // Write the command into the defined msg
    AgvCommMsg msg;
    msg.type = AGV_COMM_MSG_BLVR_SPEED_CTRL;
    for (int i = 0; i < 4; i++) {
        msg.data.blvr_op[i].speed_ctrl = in->w[i];
    }

    // Build the protocol frame from the msg
    uint8_t frame[128];
    size_t frame_len = sizeof(frame);
    int code = prtcl->build_frame(prtcl, &msg, frame, &frame_len);
    if (code != 0) return code;

    // Transform the frame into the raw data and send
    uint8_t raw[256];
    size_t raw_len = sizeof(raw);
    code = fmt->make_frame(fmt, frame, frame_len, raw, &raw_len);
    if (code != 0) return code;

    return link->send_bytes(link, raw, raw_len);
}