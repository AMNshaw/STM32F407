#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/protocol_defs/blvr_protocol_defs.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"

/**
 * private declarations
 */

typedef enum {
    ST_WAIT_ADDR,
    ST_WAIT_FUNC,
    ST_WAIT_DATA,
    ST_WAIT_CRC_LO,
    ST_WAIT_CRC_HI
} ModbusRtuFmtState;

typedef struct {
    const AgvCommFmtModbusRtuCfg* cfg;
    ModbusRtuFmtState state;

    uint8_t expct_address;
    uint8_t expct_function;
    uint8_t expct_bytes;

    size_t byte_idx;
    uint8_t crc_high;
    uint8_t crc_low;

    uint8_t* payload_buf;
    size_t payload_len;
    int has_payload;
} ModbusRtuFmtImpl;

static int modbusFmt_feed_bytes(AgvCommFormatIface* iface,
                                const uint8_t* btyes_in, size_t bytes_len);

static int modbusFmt_pop_payload(AgvCommFormatIface* iface,
                                 uint8_t* payload_out, size_t* payload_len);

static int modbusFmt_make_frame(AgvCommFormatIface* iface,
                                const uint8_t* payload_in, size_t payload_len,
                                uint8_t* frame_out, size_t* frame_len);

static int modbusFmt_destroy(AgvCommFormatIface* iface);

uint16_t modbus_crc16(const uint8_t* data, size_t len);

/**
 * Private definitions
 */

int Format_modbus_create(AgvCommFormatIface* out,
                         const AgvCommFmtModbusRtuCfg* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl =
        (ModbusRtuFmtImpl*)malloc(sizeof(ModbusRtuFmtImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->cfg = cfg;
    impl->state = ST_WAIT_ADDR;

    impl->payload_buf = (uint8_t*)malloc(cfg->max_frame_len * sizeof(uint8_t));
    if (!impl->payload_buf) {
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }

    impl->payload_len = 0;
    impl->has_payload = 0;

    out->impl = impl;
    out->feed_bytes = modbusFmt_feed_bytes;
    out->pop_payload = modbusFmt_pop_payload;
    out->make_frame = modbusFmt_make_frame;
    out->destroy = modbusFmt_destroy;

    return AGV_OK;
}

static int modbusFmt_destroy(AgvCommFormatIface* iface) {
    if (!iface) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;

    if (!impl) return AGV_ERR_NO_MEMORY;

    if (impl->payload_buf) {
        free(impl->payload_buf);
    }

    free(impl);

    iface->feed_bytes = NULL;
    iface->pop_payload = NULL;
    iface->make_frame = NULL;
    iface->destroy = NULL;

    return AGV_OK;
}

static int modbusFmt_feed_bytes(AgvCommFormatIface* iface,
                                const uint8_t* bytes_in, size_t bytes_len) {
    if (!iface || !bytes_in) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    for (size_t i = 0; i < bytes_len; ++i) {
        uint8_t b = bytes_in[i];
        switch (impl->state) {
            case ST_WAIT_ADDR: {
                impl->byte_idx = 0;
                if (b == impl->expct_address) {
                    impl->state = ST_WAIT_FUNC;
                    impl->payload_buf[impl->byte_idx++] = b;
                }
                break;
            }
            case ST_WAIT_FUNC: {
                if (b == impl->expct_function) {
                    impl->state = ST_WAIT_DATA;
                    impl->payload_buf[impl->byte_idx++] = b;
                } else {
                    impl->state = ST_WAIT_ADDR;
                }
                break;
            }
            case ST_WAIT_DATA: {
                impl->payload_buf[impl->byte_idx++] = b;

                if (impl->byte_idx == impl->expct_bytes - 2) {  //  - crc
                    impl->state = ST_WAIT_CRC_LO;
                }
                break;
            }
            case ST_WAIT_CRC_LO: {
                impl->crc_low = b;
                impl->state = ST_WAIT_CRC_HI;
                break;
            }
            case ST_WAIT_CRC_HI: {
                impl->crc_high = b;
                uint16_t rcv_crc = 0;
                rcv_crc |= (uint16_t)impl->crc_low;
                rcv_crc |= (uint16_t)impl->crc_high << 8;
                uint16_t calc_crc =
                    modbus_crc16(impl->payload_buf, impl->expct_bytes - 2);

                impl->state = ST_WAIT_ADDR;
                if (rcv_crc != calc_crc) {
                    impl->has_payload = 0;
                    impl->payload_len = 0;
                    return AGV_ERR_COMM_FMT_BAD_CRC;
                }
                impl->payload_len = impl->expct_bytes - 2;
                impl->has_payload = 1;
                return AGV_OK;
            }
            default:
                return AGV_ERR_UNKNOWN;
        }
    }
    return AGV_ERR_COMM_FMT_NO_COMPLETE_FRAME;
}

static int modbusFmt_pop_payload(AgvCommFormatIface* iface,
                                 uint8_t* payload_out, size_t* payload_len) {
    if (!iface || !payload_out || !payload_len) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (!impl->has_payload) {
        return AGV_ERR_COMM_FMT_NO_COMPLETE_FRAME;
    }

    if (*payload_len < impl->payload_len) {
        return AGV_ERR_COMM_FMT_FRAME_TOO_LONG;
    }

    memcpy(payload_out, impl->payload_buf, impl->payload_len);
    impl->has_payload = 0;
    *payload_len = impl->payload_len;

    return AGV_OK;
}

static int modbusFmt_make_frame(AgvCommFormatIface* iface,
                                const uint8_t* payload_in, size_t payload_len,
                                uint8_t* frame_out, size_t* frame_len) {
    if (!iface || !payload_in || payload_len == 0 || !frame_out || !frame_len)
        return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;

    // 總長度 = payload + CRC(2)
    size_t frame_len_temp = payload_len + 2;

    if (frame_len_temp > *frame_len) {
        return AGV_ERR_COMM_FMT_FRAME_TOO_LONG;
    }
    // 先拷貝 payload
    memcpy(frame_out, payload_in, payload_len);

    // 計算 CRC

    uint16_t crc = modbus_crc16(frame_out, payload_len);

    // Modbus RTU 是低位在前
    frame_out[payload_len] = (uint8_t)(crc & 0x00FF);             // CRC_L
    frame_out[payload_len + 1] = (uint8_t)((crc >> 8) & 0x00FF);  // CRC_H

    *frame_len = frame_len_temp;

    return AGV_OK;
}

int modbusFmt_set_check_item(AgvCommFormatIface* iface, uint8_t addr,
                             uint8_t func, size_t expected_bytes) {
    if (!iface) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->expct_address = addr;
    impl->expct_function = func;
    impl->expct_bytes = expected_bytes;
    impl->state = ST_WAIT_ADDR;
    impl->has_payload = 0;
    impl->payload_len = 0;
    impl->byte_idx = 0;

    return AGV_OK;
}

uint16_t modbus_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = MODBUS_RTU_CRC_INIT;  // 一般是 0xFFFF

    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];  // 低位對齊 XOR 進來
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^
                      MODBUS_RTU_CRC_POLY;  // LSB = 1：右移 + XOR poly
            } else {
                crc >>= 1;  // 否則只右移
            }
        }
    }

    return crc;  // 注意：Modbus RTU 傳輸時低位在前，高位在後
}