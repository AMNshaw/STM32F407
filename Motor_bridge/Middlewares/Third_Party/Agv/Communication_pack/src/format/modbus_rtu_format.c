#include "Agv_communication_pack/format/modbus_rtu_format.h"

#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"

int Format_modbus_create(AgvCommFormatIface* out,
                         const AgvCommFmtModbusRtuCfg* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl =
        (ModbusRtuFmtImpl*)malloc(sizeof(ModbusRtuFmtImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->cfg = cfg;
    impl->state = ST_IDLE;

    impl->data_buf = (uint8_t*)malloc(cfg->max_buf_size);
    if (!impl->data_buf) {
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }

    impl->frame_buf = (uint8_t*)malloc(sizeof(impl->data_buf));
    if (!impl->frame_buf) {
        free(impl->data_buf);
        free(impl);
        return AGV_ERR_NO_MEMORY;
    }

    impl->frame_len = 0;
    impl->has_frame = 0;

    out->impl = impl;
    out->feed_frame = modbusFmt_feed_frame;
    out->pop_frame = modbusFmt_pop_frame;
    out->make_frame = modbusFmt_make_frame;

    return AGV_OK;
}

static int modbusFmt_destroy(AgvCommFormatIface* iface) {
    if (!iface) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;

    if (!impl) return AGV_ERR_NO_MEMORY;

    if (impl->data_buf) {
        free(impl->data_buf);
    }

    if (impl->frame_buf) {
        free(impl->frame_buf);
    }

    free(impl);

    iface->feed_frame = NULL;
    iface->pop_frame = NULL;
    iface->make_frame = NULL;
    iface->destroy = NULL;

    return AGV_OK;
}

static int modbusFmt_feed_data(AgvCommFormatIface* iface, const uint8_t* buf,
                               size_t buf_size) {
    if (!iface || !buf) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;

    const AgvCommFmtModbusRtuCfg* cfg = impl->cfg;

    if (buf_size == 0) {
        return AGV_OK;  // 沒東西就當沒事
    }

    if (buf_size < 4) {
        // Addr(1) + Func(1) + CRC(2) 都不夠
        return AGV_ERR_COMM_FMT_FRAME_TOO_SHORT;
    }

    if (buf_size > cfg->max_frame_size || buf_size > cfg->max_buf_size) {
        // 超過允許 frame 長度或 buffer 容量
        return AGV_ERR_COMM_FMT_FRAME_TOO_LONG;
    }

    // 把這次收到的 bytes 先存到工作緩衝區
    memcpy(impl->data_buf, buf, buf_size);
    impl->data_len = buf_size;

    // 收到的 CRC：最後兩個 byte，Modbus 是低位在前
    uint16_t rx_crc =
        (uint16_t)buf[buf_size - 2] | ((uint16_t)buf[buf_size - 1] << 8);

    // 計算 CRC（不含最後兩個 CRC byte）
    uint16_t calc_crc = modbus_crc16(&cfg->crc_cfg, buf, buf_size - 2);

    if (rx_crc != calc_crc) {
        // CRC 錯誤，這一坨丟掉
        impl->has_frame = 0;
        impl->frame_len = 0;
        impl->state = ST_IDLE;
        return AGV_ERR_COMM_FMT_BAD_CRC;
    }

    // CRC OK，存成一個完整 frame 讓上層 pop
    memcpy(impl->frame_buf, buf, buf_size);
    impl->frame_len = buf_size;
    impl->has_frame = 1;
    impl->state = ST_IDLE;

    return AGV_OK;
}

static int modbusFmt_pop_frame(AgvCommFormatIface* iface, uint8_t* out_frame,
                               size_t max_out_size) {
    if (!iface || out_frame || max_out_size == 0) return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (!impl->has_frame) {
        return AGV_ERR_COMM_FMT_NO_COMPLETE_FRAME;
    }

    if (max_out_size < impl->frame_len) {
        return AGV_ERR_COMM_FMT_FRAME_TOO_LONG;
    }

    memcpy(out_frame, impl->frame_buf, impl->frame_len);
    impl->has_frame = 0;

    return impl->frame_len;
}

static int modbusFmt_make_frame(AgvCommFormatIface* iface,
                                const uint8_t* payload, size_t payload_size,
                                uint8_t* frame_out, size_t frame_size) {
    if (!iface || !payload || payload_size == 0 || !frame_out)
        return AGV_ERR_INVALID_ARG;

    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    const AgvCommFmtModbusRtuCfg* cfg = impl->cfg;
    if (!cfg) return AGV_ERR_NO_MEMORY;

    // 總長度 = payload + CRC(2)
    size_t frame_len = payload_size + 2;

    if (frame_len > cfg->max_frame_size || frame_len > frame_size) {
        return AGV_ERR_COMM_FMT_FRAME_TOO_LONG;
    }

    // 先拷貝 payload
    memcpy(frame_out, payload, frame_len);

    // 計算 CRC
    uint16_t crc = modbus_crc16(&cfg->crc_cfg, frame_out, frame_len);

    // Modbus RTU 是低位在前
    frame_out[payload_size] = (uint8_t)(crc & 0x00FF);             // CRC_L
    frame_out[payload_size + 1] = (uint8_t)((crc >> 8) & 0x00FF);  // CRC_H

    return AGV_OK;
}

static uint16_t modbus_crc16(const CrcCfg* cfg, const uint8_t* data,
                             size_t len) {
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