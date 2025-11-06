#include "protocol/blvr_protocol.h"

#include "msg/modbus_rtu.h"

int Protocol_blvr_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclBlvrCfg* cfg) {
    if (!out || !cfg) {
        return -1;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)malloc(sizeof(BlvrPrtclImpl));
    if (!impl) {
        return -2;
    }

    memset(impl, 0, sizeof(*impl));
    impl->cfg = cfg;
    impl->last_req_type = AGV_COMM_MSG_NONE;

    memset(out, 0, sizeof(*out));
    out->impl = impl;
    out->feed_frame = BlvrProto_feed_frame;
    out->pop_msg = BlvrProto_pop_msg;
    out->build_frame = BlvrProto_build_frame;
    out->destroy = BlvrProto_destroy;

    return 0;
}

static int BlvrProto_destroy(AgvCommProtocolIface* iface) {
    if (!iface || !iface->impl) {
        return -1;
    }
    free(iface->impl);
    iface->impl = NULL;
    memset(iface, 0, sizeof(*iface));
    return 0;
}

static int BlvrProto_feed_frame(AgvCommProtocolIface* iface,
                                const uint8_t* frame, size_t len) {
    if (!iface || !iface->impl || !frame) {
        return -1;
    }

    if (len < 5) {
        // Addr + Func + 至少一點資料 + CRC(2)，太短就直接丟
        return -2;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return -1;

    uint8_t addr = frame[0];
    uint8_t func = frame[1];

    // 不是我在管的 slave，就忽略
    if (addr != cfg->slave_id) {
        return 0;
    }

    // 去掉最後 2 bytes 的 CRC
    size_t pdu_len = len - 2;
    const uint8_t* p = &frame[2];   // 指向 PDU data 部分
    size_t data_len = pdu_len - 2;  // 去掉 addr+func 之後，只剩 data

    // 例外回應：func | 0x80
    if (func & MODBUS_FC_EXCEPTION_BIT) {
        uint8_t exc_code = p[0];  // Modbus exception code
        (void)exc_code;
        // 這裡你可以選擇印 log 或設一個 error flag
        return -3;
    }

    // 根據 func 及 last_req_type 來 decode
    switch (func) {
        case MODBUS_FC_READ_HOLDING_REGISTERS: {
            // Read Holding Registers response:
            // [Addr][0x03][ByteCount][Data...][CRC_L][CRC_H]
            if (data_len < 1) {
                return -4;
            }
            uint8_t byte_count = p[0];
            if (data_len < 1 + byte_count) {
                return -4;
            }
            const uint8_t* d = &p[1];

            // 檢查是否跟我們預期的一樣大小
            if (impl->last_req_type == AGV_COMM_MSG_BLVR_RPM) {
                uint16_t expected_regs = cfg->axis_count;  // 一軸一個 16 bit
                if (byte_count != expected_regs * 2) {
                    return -5;
                }

                AgvCommMsg msg;
                memset(&msg, 0, sizeof(msg));
                msg.type = AGV_COMM_MSG_BLVR_RPM;

                for (uint8_t i = 0; i < cfg->axis_count; ++i) {
                    int16_t raw = (int16_t)((d[0] << 8) | d[1]);  // big-endian
                    msg.data.blvr_op[i].rpm = (int32_t)raw;  // 存進 int32_t
                    d += 2;
                }

                impl->pending_msg = msg;
                impl->has_pending = 1;
            }

            return 0;
        }

        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            // Write Multiple Registers response:
            // [Addr][0x10][StartAddrHi][StartAddrLo][RegCountHi][RegCountLo]
            // 一般只要當 ack 就好，不需要轉成 AgvCommMsg
            // 你要的話可以在這裡依 last_req_type 做簡單狀態回報
            return 0;

        case MODBUS_FC_READWRITE_MULTIPLE_REGISTERS:
            // 之後如果你要用 Read/Write Multiple Registers，就可以在這裡參考
            // 0x03 的邏輯 Response: [Addr][0x17][ByteCount][Data...][CRC]
            return 0;

        default:
            // 其他 function code 暫時不處理
            return 0;
    }
}

static int BlvrProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* out) {
    if (!iface || !iface->impl || !out) {
        return -1;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;

    if (!impl->has_pending) {
        return -2;  // 沒有可用訊息
    }

    *out = impl->pending_msg;
    impl->has_pending = 0;

    return 0;
}

static int BlvrProto_build_frame(AgvCommProtocolIface* iface,
                                 const AgvCommMsg* msg, uint8_t* out_frame,
                                 size_t* inout_len) {
    if (!iface || !iface->impl || !msg || !out_frame || !inout_len) {
        return -1;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return -1;
    size_t max_len = *inout_len;
    size_t idx = 0;

    switch (msg->type) {
        case AGV_COMM_MSG_BLVR_SPEED_CTRL: {
            uint16_t reg_start = cfg->reg_speed_cmd_base;
            uint16_t reg_count = cfg->axis_count;  // 一軸一個暫存器
            uint8_t byte_count = (uint8_t)(reg_count * 2);

            size_t needed = 1              // addr
                            + 1            // func
                            + 2            // start addr
                            + 2            // reg_count
                            + 1            // byte_count
                            + byte_count;  // data

            if (max_len < needed) {
                return -5;  // 呼叫方給的 buffer 不夠大
            }

            // Addr
            out_frame[idx++] = cfg->slave_id;
            // Function code
            out_frame[idx++] = 0x10;

            // Start address
            out_frame[idx++] = (uint8_t)(reg_start >> 8);
            out_frame[idx++] = (uint8_t)(reg_start & 0xFF);

            // Register count
            out_frame[idx++] = (uint8_t)(reg_count >> 8);
            out_frame[idx++] = (uint8_t)(reg_count & 0xFF);

            // Byte count
            out_frame[idx++] = byte_count;

            // Data：每軸一個 int16 speed_ctrl
            for (uint8_t i = 0; i < cfg->axis_count; ++i) {
                int16_t sp = (int16_t)(msg->data.blvr_op[i].speed_ctrl);
                out_frame[idx++] = (uint8_t)((sp >> 8) & 0xFF);  // hi
                out_frame[idx++] = (uint8_t)(sp & 0xFF);         // lo
            }

            *inout_len = idx;
            impl->last_req_type = msg->type;
            return 0;
        }

        case AGV_COMM_MSG_BLVR_RPM: {
            uint16_t reg_start = cfg->reg_rpm_base;
            uint16_t reg_count =
                cfg->axis_count;  // 先假設一軸一個 16bit RPM（你可以改成 2
                                  // regs/axis）

            size_t needed = 1 + 1 + 2 + 2;  // addr + func + start + count

            if (max_len < needed) {
                return -5;
            }

            out_frame[idx++] = cfg->slave_id;  // Addr
            out_frame[idx++] = 0x03;  // Function = Read Holding Registers

            out_frame[idx++] = (uint8_t)(reg_start >> 8);
            out_frame[idx++] = (uint8_t)(reg_start & 0xFF);

            out_frame[idx++] = (uint8_t)(reg_count >> 8);
            out_frame[idx++] = (uint8_t)(reg_count & 0xFF);

            *inout_len = idx;
            impl->last_req_type = msg->type;
            return 0;
        }

        case AGV_COMM_MSG_BLVR_ACCEL:
            break;
        case AGV_COMM_MSG_BLVR_DECEL:
            break;
        case AGV_COMM_MSG_BLVR_TRIGGER:
            break;
        default:
            return -4;
    }
}