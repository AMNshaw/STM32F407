#include "Agv_communication_pack/protocol/blvr_protocol.h"

#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "Agv_communication_pack/protocol_defs/blvr_protocol_defs.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"
#include "Agv_core/utils.h"

/**
 * private declarations
 */

typedef struct {
    const AgvCommPrtclBlvrCfg* cfg;

    AgvCommMsg pending_msg;
    int has_pending;

} BlvrPrtclImpl;

static int BlvrProto_feed_payload(AgvCommProtocolIface* iface,
                                  const uint8_t* payload_in,
                                  size_t payload_len);

static int BlvrProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* msg_out);

static int BlvrProto_make_payload(AgvCommProtocolIface* iface,
                                  const AgvCommMsg* msg_in, uint8_t* frame_out,
                                  size_t* frame_len);

static int BlvrProto_destroy(AgvCommProtocolIface* iface);

static uint8_t* put_be32(uint8_t* p, int32_t v);

static const uint8_t* get_be32(const uint8_t* p, int32_t* out);

/**
 * Private definitions
 */

int Protocol_blvr_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclBlvrCfg* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)malloc(sizeof(BlvrPrtclImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->cfg = cfg;
    impl->has_pending = 0;

    out->impl = impl;
    out->feed_payload = BlvrProto_feed_payload;
    out->pop_msg = BlvrProto_pop_msg;
    out->make_payload = BlvrProto_make_payload;
    out->destroy = BlvrProto_destroy;

    return AGV_OK;
}

static int BlvrProto_destroy(AgvCommProtocolIface* iface) {
    if (!iface || !iface->impl) return AGV_ERR_INVALID_ARG;
    free(iface->impl);

    iface->impl = NULL;
    iface->feed_payload = NULL;
    iface->pop_msg = NULL;
    iface->make_payload = NULL;
    iface->destroy = NULL;

    return AGV_OK;
}

static int BlvrProto_feed_payload(AgvCommProtocolIface* iface,
                                  const uint8_t* payload_in,
                                  size_t payload_len) {
    if (!iface || !payload_in || payload_len == 0) {
        return AGV_ERR_INVALID_ARG;
    }

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return AGV_ERR_NO_MEMORY;

    uint8_t addr = payload_in[0];
    uint8_t func = payload_in[1];

    if (func & BLVR_FC_EXCEPTION_BIT) {
        uint8_t exc_code = payload_in[3];  // Modbus exception code
        (void)exc_code;
        return AGV_ERR_COMM_PRTCL_EXCEPTION;
    }

    if (addr != cfg->shared_id) return AGV_ERR_COMM_PRTCL_UNSUPPORTED_SHARED_ID;

    if (payload_len < 3) {  // [address 1][func 1][data *]
        return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
    }

    size_t data_len =
        payload_len - 3;  // addr 1, func 1, data[0] is registers byte count
    if (data_len <= 0) {
        return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
    }

    const uint8_t* data = &payload_in[2];  // 指向 PDU data 部分
    switch (func) {
        case BLVR_FC_READWRITE_MULTIPLE_REGISTERS: {
            // Read Holding Registers response:
            // [Addr][0x03][ByteCount][Data...][CRC_L][CRC_H]

            size_t reg_byte_count = data[0];

            const uint8_t* registers_data = &data[1];
            size_t bytes_per_axis =
                cfg->num_rgster_read_cmd * cfg->byte_per_rgstr +
                2;  // + 2 error check
            uint16_t expected_bytes =
                cfg->axis_count *
                (uint16_t)bytes_per_axis;  // including error check bytes
            if (reg_byte_count != expected_bytes) {
                return AGV_ERR_COMM_PRTCL_BAD_PAYLOAD;
            }
            LOG("blvr", "payload_len=%d, addr=%02X, func=%02X, byteCount=%d",
                (int)payload_len, addr, func, data[0]);
            AgvCommMsg msg = {0};
            msg.msg_type = MOTOR_MSG;
            for (size_t i = 0; i < cfg->axis_count; ++i) {
                size_t axis_byte_idx = i * bytes_per_axis;
                const uint8_t* p = &registers_data[axis_byte_idx];

                // p = get_be32(p, &msg.u.motors_msg.msgs[i].driver_st);
                msg.u.motors_msg.msgs[i].driver_st |= (int32_t)(*p++) << 8;
                msg.u.motors_msg.msgs[i].driver_st |= (int32_t)(*p++);
                p = get_be32(p, &msg.u.motors_msg.msgs[i].rl_pos);
                p = get_be32(p, &msg.u.motors_msg.msgs[i].rl_rpm);
                p = get_be32(p, &msg.u.motors_msg.msgs[i].alrm);
            }

            impl->pending_msg = msg;
            impl->has_pending = 1;

            return AGV_OK;
        }
        case BLVR_FC_WRITE_MULTIPLE_REGISTERS: {
            return 0;
        }
        case BLVR_FC_READ_HOLDING_REGISTERS: {
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
                                  const AgvCommMsg* msg_in,
                                  uint8_t* payload_out, size_t* payload_len) {
    if (!iface || !msg_in || !payload_out || !payload_len) {
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
                cfg->axis_count * cfg->num_rgster_write_cmd;
            uint16_t totol_byte_count = totol_rgstr_count * cfg->byte_per_rgstr;

            size_t needed = 1                    // addr
                            + 1                  // func
                            + 2                  // start addr
                            + 2                  // reg_count
                            + 1                  // byte_count
                            + totol_byte_count;  // data

            if (*payload_len < needed) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;  // 呼叫方給的 buffer
                                                          // 不夠大
            }

            // Addr
            payload_out[idx++] = cfg->shared_id;
            // Function code

            payload_out[idx++] = BLVR_FC_WRITE_MULTIPLE_REGISTERS;

            // modbus is a 8bit format, but BLVR is using 16bit, so we should
            // split the var to 2 8bit (upper & lower)

            // Start address
            payload_out[idx++] = (uint8_t)(reg_start >> 8);
            payload_out[idx++] = (uint8_t)(reg_start & 0xFF);

            // Register count
            payload_out[idx++] = (uint8_t)(totol_rgstr_count >> 8);
            payload_out[idx++] = (uint8_t)(totol_rgstr_count & 0xFF);

            // Byte count
            payload_out[idx++] = totol_byte_count;

            uint8_t* p = &payload_out[idx];
            for (uint8_t i = 0; i < cfg->axis_count; ++i) {
                p = put_be32(p, motors_msg.msgs[i].des_vel);
                p = put_be32(p, motors_msg.msgs[i].des_acc);
                p = put_be32(p, motors_msg.msgs[i].des_dec);
                p = put_be32(p, motors_msg.msgs[i].spd_ctrl);
                p = put_be32(p, motors_msg.msgs[i].trigger);
            }

            *payload_len = idx;

            return AGV_OK;
        }

        case READ: {
            const uint16_t addr = cfg->reg_address_read.driver_status;
            const uint16_t totol_rgstr_count =
                cfg->axis_count * (cfg->num_rgster_read_cmd);

            size_t needed = 1     // addr
                            + 1   // func
                            + 2   // start addr
                            + 2;  // reg count

            if (*payload_len < needed) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
            }

            payload_out[idx++] = cfg->shared_id;
            // payload_out[idx++] = 0x04;
            payload_out[idx++] = BLVR_FC_READ_HOLDING_REGISTERS;
            payload_out[idx++] = (uint8_t)(addr >> 8);
            payload_out[idx++] = (uint8_t)(addr & 0xFF);
            payload_out[idx++] = (uint8_t)(totol_rgstr_count >> 8);
            payload_out[idx++] = (uint8_t)(totol_rgstr_count & 0xFF);

            *payload_len = idx;
            return AGV_OK;
        }
        case READ_WRITE: {
            const uint16_t reg_start_read = cfg->reg_address_read.driver_status;
            const uint16_t reg_start_write = cfg->reg_address_write.cmd_vel;
            const uint16_t totol_read_rgstr_count =
                cfg->axis_count * (cfg->num_rgster_read_cmd +
                                   1);  // +1 depends on the document p281.
            const uint16_t totol_write_rgstr_count =
                cfg->axis_count * cfg->num_rgster_write_cmd;
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
            if (*payload_len < needed) {
                return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;  // 呼叫方給的 buffer
                                                          // 不夠大
            }

            // Addr
            payload_out[idx++] = cfg->shared_id;
            // Function code

            payload_out[idx++] = BLVR_FC_READWRITE_MULTIPLE_REGISTERS;

            // modbus is a 8bit format, but BLVR is using 16bit, so we should
            // split the var to 2 8bit (upper & lower)

            // Start address read
            payload_out[idx++] = (uint8_t)(reg_start_read >> 8);
            payload_out[idx++] = (uint8_t)(reg_start_read & 0xFF);

            // Register count read
            payload_out[idx++] = (uint8_t)(totol_read_rgstr_count >> 8);
            payload_out[idx++] = (uint8_t)(totol_read_rgstr_count & 0xFF);

            // Start address write
            payload_out[idx++] = (uint8_t)(reg_start_write >> 8);
            payload_out[idx++] = (uint8_t)(reg_start_write & 0xFF);

            // Register count write
            payload_out[idx++] = (uint8_t)(totol_write_rgstr_count >> 8);
            payload_out[idx++] = (uint8_t)(totol_write_rgstr_count & 0xFF);

            // Byte count
            payload_out[idx++] = write_byte_count;

            uint8_t* p = &payload_out[idx];
            for (uint8_t i = 0; i < cfg->axis_count; ++i) {
                p = put_be32(p, motors_msg.msgs[i].des_vel);
                p = put_be32(p, motors_msg.msgs[i].des_acc);
                p = put_be32(p, motors_msg.msgs[i].des_dec);
                p = put_be32(p, motors_msg.msgs[i].spd_ctrl);
                p = put_be32(p, motors_msg.msgs[i].trigger);
            }
            idx = (size_t)(p - payload_out);

            *payload_len = idx;
            return AGV_OK;
        }

        default:
            return AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE;
    }

    return AGV_OK;
}

int BlvrProto_get_response_frame_len(AgvCommProtocolIface* iface,
                                     const AgvCommMsg* request,
                                     size_t* frame_len) {
    if (!iface || !request || request->msg_type != MOTOR_MSG || !frame_len)
        return AGV_ERR_INVALID_ARG;

    BlvrPrtclImpl* impl = (BlvrPrtclImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    const AgvCommPrtclBlvrCfg* cfg = impl->cfg;
    if (!cfg) return AGV_ERR_NO_MEMORY;

    size_t expected_len = 0;
    switch (request->u.motors_msg.type) {
        case READ_WRITE:
            size_t total_read_bytes = cfg->axis_count * cfg->byte_per_rgstr *
                                      cfg->num_rgster_read_cmd;
            size_t total_error_check_bytes = 2 * cfg->axis_count;
            expected_len = 1                   // address
                           + 1                 // fucntion code
                           + 1                 // number of read data bytes
                           + total_read_bytes  // read bytes
                           + total_error_check_bytes  // error check
                           + 2                        // crc
                ;
            break;

        default:
            return AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE;
    }
    *frame_len = expected_len;
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
