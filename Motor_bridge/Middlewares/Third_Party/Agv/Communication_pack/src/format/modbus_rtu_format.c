#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"

/**
 * private declarations
 */

typedef enum {
    ST_IDLE,
    ST_WAIT_ADDR,
    ST_WAIT_FUNC,
    ST_WAIT_DATA,
    ST_WAIT_CRC_LO,
    ST_WAIT_CRC_HI
} ModbusRtuFmtState;

typedef struct {
    const AgvCommFmtModbusRtuCfg* cfg;
    ModbusRtuFmtState state;

    uint8_t address;
    uint8_t function;

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

uint16_t modbus_crc16(const CrcCfg* cfg, const uint8_t* data, size_t len);

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
    impl->state = ST_IDLE;

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
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;

    const AgvCommFmtModbusRtuCfg* cfg = impl->cfg;

    if (bytes_len == 0) {
        return AGV_OK;  // 沒東西就當沒事
    }

    if (bytes_len < 4) {
        // Addr(1) + Func(1) + CRC(2) 都不夠
        return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
    }

    if (bytes_len > cfg->max_frame_len) {
        return AGV_ERR_COMM_FMT_FRAME_TOO_LONG;
    }

    // 收到的 CRC：最後兩個 byte，Modbus 是低位在前
    uint16_t rcv_crc = (uint16_t)bytes_in[bytes_len - 2] |
                       ((uint16_t)bytes_in[bytes_len - 1] << 8);

    // 計算 CRC（不含最後兩個 CRC byte）
    uint16_t calc_crc = modbus_crc16(&cfg->crc_cfg, bytes_in, bytes_len - 2);

    if (rcv_crc != calc_crc) {
        // CRC 錯誤，這一坨丟掉
        impl->has_payload = 0;
        impl->payload_len = 0;
        impl->state = ST_IDLE;
        return AGV_ERR_COMM_FMT_BAD_CRC;
    }

    impl->address = bytes_in[0];
    impl->function = bytes_in[1];
    memcpy(impl->payload_buf, bytes_in, bytes_len);
    impl->payload_len = bytes_len;
    impl->has_payload = 1;
    impl->state = ST_IDLE;

    return AGV_OK;
}

static int modbusFmt_pop_payload(AgvCommFormatIface* iface,
                                 uint8_t* payload_out, size_t* payload_len) {
    if (!iface || payload_out || !payload_len) return AGV_ERR_INVALID_ARG;

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
    impl->payload_len = *payload_len;

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
    memcpy(frame_out, payload_in, *frame_len);

    // 計算 CRC

    uint16_t crc = modbus_crc16(&impl->cfg->crc_cfg, frame_out, frame_len_temp);

    // Modbus RTU 是低位在前
    frame_out[payload_len] = (uint8_t)(crc & 0x00FF);             // CRC_L
    frame_out[payload_len + 1] = (uint8_t)((crc >> 8) & 0x00FF);  // CRC_H

    *frame_len = frame_len_temp;

    return AGV_OK;
}

uint16_t modbus_crc16(const CrcCfg* cfg, const uint8_t* data, size_t len) {
    uint16_t crc = cfg->crc_init;  // 一般是 0xFFFF

    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];  // 低位對齊 XOR 進來
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ cfg->crc_poly;  // LSB = 1：右移 + XOR poly
            } else {
                crc >>= 1;  // 否則只右移
            }
        }
    }

    return crc;  // 注意：Modbus RTU 傳輸時低位在前，高位在後
}