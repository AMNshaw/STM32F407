#include "Agv_communication_pack/protocol/blvr_protocol.h"

#include <stdbool.h>

#include "Agv_communication_pack/protocol_defs/blvr_protocol_defs.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"

int Protocol_blvr_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclBlvrCfg* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)malloc(sizeof(BlvrPrtclImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->cfg = cfg;
    impl->has_pending = 0;

    out->impl = impl;
    out->feed_frame = BlvrProto_feed_frame;
    out->pop_msg = BlvrProto_pop_msg;
    out->make_payload = BlvrProto_make_payload;
    out->destroy = BlvrProto_destroy;

    return AGV_OK;
}

static int BlvrProto_destroy(AgvCommProtocolIface* iface) {
    if (!iface || !iface->impl) return AGV_ERR_INVALID_ARG;
    free(iface->impl);

    iface->impl = NULL;
    iface->feed_frame = NULL;
    iface->pop_msg = NULL;
    iface->make_payload = NULL;
    iface->destroy = NULL;

    return AGV_OK;
}

static int BlvrProto_feed_frame(AgvCommProtocolIface* iface,
                                const uint8_t* frame_in, size_t frame_len) {
    if (!iface || !frame_in || frame_len == 0) {
        return AGV_ERR_INVALID_ARG;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return AGV_ERR_NO_MEMORY;

    uint8_t addr = frame_in[0];
    uint8_t func = frame_in[1];

    if (func & BLVR_FC_EXCEPTION_BIT) {
        uint8_t exc_code = frame_in[3];  // Modbus exception code
        (void)exc_code;
        // 這裡你可以選擇印 log 或設一個 error flag
        return AGV_ERR_COMM_PRTCL_EXCEPTION;
    }

    if (addr != cfg->shared_id) return AGV_ERR_COMM_PRTCL_UNSUPPORTED_SHARED_ID;

    if (frame_len < 5) {  // [address 1][func 1][data *][crc up1][crc lo1]
        return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
    }

    size_t pdu_len = frame_len - 2;
    const uint8_t* pdu_data = &frame_in[2];  // 指向 PDU data 部分
    size_t data_len = pdu_len - 2;           // 去掉 crc 之後，只剩 data
    size_t reg_len = data_len - 1;           // 第一個 byte 是 byte_count

    // 根據 func 來 decode
    switch (func) {
        case BLVR_FC_READ_HOLDING_REGISTERS: {
            // Read Holding Registers response:
            // [Addr][0x03][ByteCount][Data...][CRC_L][CRC_H]
            if (reg_len <= 0) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
            }
            uint8_t byte_count = pdu_data[0];
            if (reg_len != byte_count) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
            }
            const uint8_t* registers_data = &pdu_data[1];

            // 檢查是否跟我們預期的一樣大小
            uint16_t expected_bytes =
                cfg->axis_count * (cfg->num_read_cmd * cfg->num_rgster_per_cmd *
                                   cfg->byte_per_rgstr);
            if (byte_count != expected_bytes) {
                return AGV_ERR_COMM_PRTCL_BAD_PAYLOAD;
            }
            AgvCommMsg msg = {0};
            msg.msg_type = MOTOR_MSG;
            for (size_t i = 0; i < cfg->axis_count; ++i) {
                size_t axis_byte_idx =
                    i * (cfg->num_rgster_per_cmd * cfg->byte_per_rgstr *
                         cfg->num_read_cmd);
                const uint8_t* p = &registers_data[axis_byte_idx];
                uint16_t error_check = 0;

                p = get_be32(p, &msg.u.motors_msg.msgs[i].driver_st);
                p = get_be32(p, &msg.u.motors_msg.msgs[i].rl_pos);
                p = get_be32(p, &msg.u.motors_msg.msgs[i].rl_rpm);
                p = get_be32(p, &msg.u.motors_msg.msgs[i].alrm);
                error_check |= (int32_t)(*p++) << 8;
                error_check |= (int32_t)(*p++);
            }

            impl->pending_msg = msg;
            impl->has_pending = 1;

            return 0;
        }
        case BLVR_FC_WRITE_MULTIPLE_REGISTERS: {
            return 0;
        }
        case BLVR_FC_READWRITE_MULTIPLE_REGISTERS: {
            return 0;
        }
        default:
            // 其他 function code 暫時不處理
            return 0;
    }
}

static int BlvrProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* msg_out) {
    if (!iface || !msg_out) return AGV_ERR_INVALID_ARG;

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (!impl->has_pending)
        return AGV_ERR_COMM_PRTCL_NO_PENDING_MSG;  // 沒有可用訊息

    *msg_out = impl->pending_msg;
    impl->has_pending = 0;

    return 0;
}

static int BlvrProto_make_payload(AgvCommProtocolIface* iface,
                                  const AgvCommMsg* msg_in, uint8_t* frame_out,
                                  size_t* frame_len) {
    if (!iface || !msg_in || !frame_out || !frame_len) {
        return AGV_ERR_INVALID_ARG;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return AGV_ERR_NO_MEMORY;

    if (msg_in->msg_type != MOTOR_MSG)
        return AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE;

    MotorsCommMsg motors_msg = msg_in->u.motors_msg;

    size_t idx = 0;
    switch (motors_msg.type) {
        case WRITE: {
            const uint16_t reg_start = cfg->reg_address_write.cmd_vel;
            const uint16_t totol_rgstr_count =
                cfg->axis_count *
                (cfg->num_write_cmd * cfg->num_rgster_per_cmd);
            uint16_t totol_byte_count = totol_rgstr_count * cfg->byte_per_rgstr;

            size_t needed = 1                    // addr
                            + 1                  // func
                            + 2                  // start addr
                            + 2                  // reg_count
                            + 1                  // byte_count
                            + totol_byte_count;  // data

            if (*frame_len < needed) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;  // 呼叫方給的 buffer
                                                          // 不夠大
            }

            // Addr
            frame_out[idx++] = cfg->shared_id;
            // Function code

            frame_out[idx++] = BLVR_FC_WRITE_MULTIPLE_REGISTERS;

            // modbus is a 8bit format, but BLVR is using 16bit, so we should
            // split the var to 2 8bit (upper & lower)

            // Start address
            frame_out[idx++] = (uint8_t)(reg_start >> 8);
            frame_out[idx++] = (uint8_t)(reg_start & 0xFF);

            // Register count
            frame_out[idx++] = (uint8_t)(totol_rgstr_count >> 8);
            frame_out[idx++] = (uint8_t)(totol_rgstr_count & 0xFF);

            // Byte count
            frame_out[idx++] = totol_byte_count;

            uint8_t* p = &frame_out[idx];
            for (uint8_t i = 0; i < cfg->axis_count; ++i) {
                p = put_be32(p, motors_msg.msgs[i].des_vel);
                p = put_be32(p, motors_msg.msgs[i].des_acc);
                p = put_be32(p, motors_msg.msgs[i].des_dec);
                p = put_be32(p, motors_msg.msgs[i].spd_ctrl);
                p = put_be32(p, motors_msg.msgs[i].trigger);
            }

            *frame_len = idx;

            return AGV_OK;
        }

        case READ: {
            const uint16_t addr = cfg->reg_address_read.driver_status;
            const uint16_t totol_rgstr_count =
                cfg->axis_count * (cfg->num_read_cmd * cfg->num_rgster_per_cmd);

            size_t needed = 1     // addr
                            + 1   // func
                            + 2   // start addr
                            + 2;  // reg count

            if (*frame_len < needed) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
            }

            frame_out[idx++] = cfg->shared_id;
            frame_out[idx++] = BLVR_FC_READ_HOLDING_REGISTERS;
            frame_out[idx++] = (uint8_t)(addr >> 8);
            frame_out[idx++] = (uint8_t)(addr & 0xFF);
            frame_out[idx++] = (uint8_t)(totol_rgstr_count >> 8);
            frame_out[idx++] = (uint8_t)(totol_rgstr_count & 0xFF);

            *frame_len = idx;
            return AGV_OK;
        }
        case READ_WRITE: {
            const uint16_t reg_start_read = cfg->reg_address_read.driver_status;
            const uint16_t reg_start_write = cfg->reg_address_write.cmd_vel;
            const uint16_t totol_read_rgstr_count =
                cfg->axis_count * cfg->num_read_cmd * cfg->num_rgster_per_cmd;
            const uint16_t totol_write_rgstr_count =
                cfg->axis_count * cfg->num_write_cmd * cfg->num_rgster_per_cmd;
            const uint16_t write_byte_count =
                totol_write_rgstr_count * cfg->byte_per_rgstr;

            size_t needed = 1                    // addr
                            + 1                  // func
                            + 2                  // start addr read
                            + 2                  // regc_ount read
                            + 2                  // start addr write
                            + 2                  // reg_count write
                            + 1                  // byte_count
                            + write_byte_count;  // data
            if (*frame_len < needed) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;  // 呼叫方給的 buffer
                                                          // 不夠大
            }

            // Addr
            frame_out[idx++] = cfg->shared_id;
            // Function code

            frame_out[idx++] = BLVR_FC_READWRITE_MULTIPLE_REGISTERS;

            // modbus is a 8bit format, but BLVR is using 16bit, so we should
            // split the var to 2 8bit (upper & lower)

            // Start address read
            frame_out[idx++] = (uint8_t)(reg_start_read >> 8);
            frame_out[idx++] = (uint8_t)(reg_start_read & 0xFF);

            // Register count read
            frame_out[idx++] = (uint8_t)(totol_read_rgstr_count >> 8);
            frame_out[idx++] = (uint8_t)(totol_read_rgstr_count & 0xFF);

            // Start address write
            frame_out[idx++] = (uint8_t)(reg_start_write >> 8);
            frame_out[idx++] = (uint8_t)(reg_start_write & 0xFF);

            // Register count write
            frame_out[idx++] = (uint8_t)(totol_write_rgstr_count >> 8);
            frame_out[idx++] = (uint8_t)(totol_write_rgstr_count & 0xFF);

            // Byte count
            frame_out[idx++] = write_byte_count;

            uint8_t* p = &frame_out[idx];
            for (uint8_t i = 0; i < cfg->axis_count; ++i) {
                p = put_be32(p, motors_msg.msgs[i].des_vel);
                p = put_be32(p, motors_msg.msgs[i].des_acc);
                p = put_be32(p, motors_msg.msgs[i].des_dec);
                p = put_be32(p, motors_msg.msgs[i].spd_ctrl);
                p = put_be32(p, motors_msg.msgs[i].trigger);
            }
            idx = (size_t)(p - frame_out);

            *frame_len = idx;
            return AGV_OK;
        }

        default:
            return AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE;
    }

    return AGV_OK;
}

static uint8_t* put_be32(uint8_t* p, int32_t v) {
    *p++ = (uint8_t)(v >> 24);
    *p++ = (uint8_t)(v >> 16);
    *p++ = (uint8_t)(v >> 8);
    *p++ = (uint8_t)(v);
    return p;
}

static const uint8_t* get_be32(const uint8_t* p, int32_t* out) {
    int32_t v = 0;

    v |= (int32_t)(*p++) << 24;
    v |= (int32_t)(*p++) << 16;
    v |= (int32_t)(*p++) << 8;
    v |= (int32_t)(*p++);

    *out = v;
    return p;
}
