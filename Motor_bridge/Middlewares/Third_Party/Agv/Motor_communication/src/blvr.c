#include "Agv_motor_communication/blvr.h"

#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_communication_pack/format/modbus_rtu_format.h"
#include "Agv_communication_pack/link/uart_rs485.h"
#include "Agv_communication_pack/protocol/blvr_protocol.h"

static int blvr_read(AgvMotorCommunicationBase* self, MotorInterMsg* out_msg) {
    // Convenience pointers to the composed communication interfaces
    CommBlvrMotorImpl* impl = (CommBlvrMotorImpl*)self->impl;
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

    // ToDO: Make protocol frame and send first, then wait for response

    // ToDo

    // Receive raw bytes from the UART link
    size_t max_fmt_buff_size = modbus_impl->cfg->max_buf_size;
    uint8_t data_buf[max_fmt_buff_size];
    int data_len = link->read_buf(link, data_buf, max_fmt_buff_size);
    if (data_len <= 0) return data_len;

    uint32_t gap_threshold_us = rs485_impl->cfg->char_time_10x_us *
                                modbus_impl->cfg->interframe_chars_x10;

    if (data_len > 0) {
        uint32_t dt = now_us - rs485_impl->last_rx_time_us;
        if (dt >= gap_threshold_us) {
            // gap 夠長 => 前面的當作一個 Modbus frame 丟進 formatter
            fmt->feed(fmt, data_buf, (size_t)data_len);
        }
    }

    // Extract complete Modbus RTU frames from the formatter
    size_t max_frame_size = blvr_prtcl_impl->cfg->max_frame_size;
    uint8_t frame[max_frame_size];
    while (1) {
        int frame_len = fmt->pop_frame(fmt, frame, &max_frame_size);
        if (frame_len <= 0) break;
        // For each decoded frame, feed it into the protocol layer
        prtcl->feed_frame(prtcl, frame, frame_len);
    }

    // Drain protocol messages and look for BLVR RPM command messages
    MotorInterMsg msg = {0};
    int got_cmd = 0;
    while (prtcl->pop_msg(prtcl, &msg) == 0) {
        switch (msg.rpm) {
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

static int blvr_write_targets(AgvMotorCommunicationBase* self,
                              const WheelVel* in) {
    if (!in) return -1;

    // Convenience pointers to the composed communication interfaces
    CommBlvrMotorImpl* impl = (CommBlvrMotorImpl*)self->impl;
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