#include "Agv_communication_pack/protocol/blvr_protocol.h"

#include <stdbool.h>

#include "Agv_communication_pack/exception_codes.h"
#include "Agv_communication_pack/protocol_defs/blvr_protocol_defs.h"

int Protocol_blvr_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclBlvrCfg* cfg) {
    if (!out || !cfg) return AGV_COMM_ERR_INVALID_ARG;

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)malloc(sizeof(BlvrPrtclImpl));
    if (!impl) return AGV_COMM_ERR_NO_MEMORY;

    impl->cfg = cfg;
    impl->has_pending = 0;

    out->impl = impl;
    out->feed_frame = BlvrProto_feed_frame;
    out->pop_msg = BlvrProto_pop_msg;
    out->build_frame = BlvrProto_build_frame;
    out->destroy = BlvrProto_destroy;

    return 0;
}

static int BlvrProto_destroy(AgvCommProtocolIface* iface) {
    if (!iface || !iface->impl) return AGV_COMM_ERR_INVALID_ARG;
    free(iface->impl);
    iface->impl = NULL;

    return AGV_COMM_OK;
}

static int BlvrProto_feed_frame(AgvCommProtocolIface* iface,
                                const uint8_t* frame_in, size_t frame_len) {
    if (!iface || !frame_in || frame_len == 0) {
        return AGV_COMM_ERR_INVALID_ARG;
    }

    if (frame_len < 5) {
        // Addr + Func + 至少一點資料 + CRC(2)，太短就直接丟
        return AGV_COMM_ERR_FMT_FRAME_TOO_SHORT;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_COMM_ERR_NO_MEMORY;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return AGV_COMM_ERR_NO_MEMORY;

    uint8_t addr = frame_in[0];
    uint8_t func = frame_in[1];

    // 不是我在管的 slave，就忽略
    int has_slave_id = false;
    for (size_t i = 0; i < sizeof(cfg->slave_ids); ++i) {
        if (addr == cfg->slave_ids[i]) {
            has_slave_id == true;
            break;
        }
    }
    if (!has_slave_id) return AGV_COMM_ERR_PRTCL_UNSUPPORTED_SLAVE_ID;

    // 去掉最後 2 bytes 的 CRC
    size_t pdu_len = frame_len - 2;
    const uint8_t* p = &frame_in[2];  // 指向 PDU data 部分
    size_t data_len = pdu_len - 2;    // 去掉 addr+func 之後，只剩 data

    // 例外回應：func | 0x80
    if (func & MODBUS_FC_EXCEPTION_BIT) {
        uint8_t exc_code = p[0];  // Modbus exception code
        (void)exc_code;
        // 這裡你可以選擇印 log 或設一個 error flag
        return -3;
    }

    // 根據 func 來 decode
    switch (func) {
        case MODBUS_FC_READ_HOLDING_REGISTERS: {
            // Read Holding Registers response:
            // [Addr][0x03][ByteCount][Data...][CRC_L][CRC_H]
            if (data_len < 1) {
                return AGV_COMM_ERR_FMT_FRAME_TOO_SHORT;
            }
            uint8_t byte_count = p[0];
            if (data_len < 1 + byte_count) {
                return AGV_COMM_ERR_FMT_FRAME_TOO_SHORT;
            }
            const uint8_t* d = &p[1];

            // 檢查是否跟我們預期的一樣大小
            uint16_t expected_regs = cfg->axis_count;  // 一軸一個 16 bit
            if (byte_count != expected_regs * 2) {
                return AGV_COMM_ERR_PRTCL_BAD_PAYLOAD;
            }

            AgvCommMsg msg;
            msg.msg_type = MOTOR_MSG;
            msg.u.motors_msg.msg_type = AGV_COMM_MSG_RPM;

            for (uint8_t i = 0; i < cfg->axis_count; ++i) {
                int16_t raw = (int16_t)((d[0] << 8) | d[1]);  // big-endian
                msg.u.motors_msg.msgs[i].rpm = raw;
                d += 2;
            }

            impl->pending_msg = msg;
            impl->has_pending = 1;

            return 0;
        }

        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            // Write Multiple Registers response:
            // [Addr][0x10][StartAddrHi][StartAddrLo][RegCountHi][RegCountLo]
            // 一般只要當 ack 就好，不需要轉成 AgvCommMsg
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

static int BlvrProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* out_msg) {
    if (!iface || !out_msg) return AGV_COMM_ERR_INVALID_ARG;

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_COMM_ERR_NO_MEMORY;

    if (!impl->has_pending)
        return AGV_COMM_ERR_PRTCL_NO_PENDING_MSG;  // 沒有可用訊息

    *out_msg = impl->pending_msg;
    impl->has_pending = 0;

    return 0;
}

static int BlvrProto_build_frame(AgvCommProtocolIface* iface,
                                 const AgvCommMsg* msg, uint8_t* out_frame,
                                 size_t max_out_len) {
    if (!iface || !msg || !out_frame || max_out_len == 0) {
        return AGV_COMM_ERR_INVALID_ARG;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_COMM_ERR_NO_MEMORY;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return AGV_COMM_ERR_NO_MEMORY;

    size_t idx = 0;

    if (msg->msg_type != MOTOR_MSG) return AGV_COMM_ERR_PRTCL_INVALID_MSG_TYPE;

    MotorsMsg motors_msg = msg->u.motors_msg;

    switch (motors_msg.msg_type) {
        case AGV_COMM_MSG_SPEED_CTRL: {
            uint16_t reg_start = cfg->reg_speed_cmd_base;
            uint16_t reg_count = cfg->axis_count;  // 一軸一個暫存器
            uint8_t byte_count = (uint8_t)(reg_count * 2);

            size_t needed = 1              // addr
                            + 1            // func
                            + 2            // start addr
                            + 2            // reg_count
                            + 1            // byte_count
                            + byte_count;  // data

            if (max_out_len < needed) {
                return AGV_COMM_ERR_FMT_FRAME_TOO_SHORT;  // 呼叫方給的 buffer
                                                          // 不夠大
            }

            // Addr
            out_frame[idx++] = cfg->slave_id;
            // Function code
            out_frame[idx++] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;

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
                int16_t sp = (int16_t)(motors_msg.msgs[i].speed_ctrl);
                out_frame[idx++] = (uint8_t)((sp >> 8) & 0xFF);  // hi
                out_frame[idx++] = (uint8_t)(sp & 0xFF);         // lo
            }

            return 0;
        }

        case AGV_COMM_MSG_ACCEL:
            break;
        case AGV_COMM_MSG_DECEL:
            break;
        case AGV_COMM_MSG_TRIGGER:
            break;
        default:
            return AGV_COMM_ERR_PRTCL_INVALID_MSG_TYPE;
    }
    return idx;
}