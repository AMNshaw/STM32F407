#include "Agv_communication_pack/format/modbus_rtu_format.h"

int Format_modbus_create(AgvCommFormatIface* out,
                         const AgvCommFmtModbusRtuCfg* cfg) {
    if (!out || !cfg) return -1;

    ModbusRtuFmtImpl* impl =
        (ModbusRtuFmtImpl*)malloc(sizeof(ModbusRtuFmtImpl));
    if (!impl) return -2;

    memset(impl, 0, sizeof(*impl));
    impl->cfg = cfg;
    impl->state = ST_IDLE;
    impl->data_buf = (uint8_t*)malloc(cfg->data_buf_size);
    if (!impl->data_buf) {
        free(impl);
        return -3;
    }

    impl->frame_buf = (uint8_t*)malloc(sizeof(impl->data_buf));
    if (!impl->frame_buf) {
        free(impl->data_buf);
        free(impl);
        return -4;
    }

    impl->frame_len = 0;
    impl->has_frame = 0;

    // 把 out 的其他函式指標先清 0，避免有垃圾值
    memset(out, 0, sizeof(*out));

    out->impl = impl;
    out->feed = modbusFmt_feed;
    out->pop_frame = modbusFmt_pop_frame;
    out->make_frame = modbusFmt_make_frame;

    return 0;
}

static int modbusFmt_feed(AgvCommFormatIface* iface, const uint8_t* bytes,
                          size_t n) {
    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl || !impl->cfg) return -1;

    const AgvCommFmtModbusRtuCfg* cfg = impl->cfg;

    if (n == 0) {
        return 0;  // 沒東西就當沒事
    }

    if (n < 4) {
        // Addr(1) + Func(1) + CRC(2) 都不夠
        return -2;
    }

    if (n > cfg->max_frame_size || n > cfg->data_buf_size) {
        // 超過允許 frame 長度或 buffer 容量
        return -3;
    }

    // 把這次收到的 bytes 先存到工作緩衝區
    memcpy(impl->data_buf, bytes, n);
    impl->data_len = n;

    // 收到的 CRC：最後兩個 byte，Modbus 是低位在前
    uint16_t rx_crc = (uint16_t)bytes[n - 2] | ((uint16_t)bytes[n - 1] << 8);

    // 計算 CRC（不含最後兩個 CRC byte）
    uint16_t calc_crc = modbus_crc16(&cfg->crc_cfg, bytes, n - 2);

    if (rx_crc != calc_crc) {
        // CRC 錯誤，這一坨丟掉
        impl->has_frame = 0;
        impl->frame_len = 0;
        impl->state = ST_IDLE;
        return -4;
    }

    // CRC OK，存成一個完整 frame 讓上層 pop
    memcpy(impl->frame_buf, bytes, n);
    impl->frame_len = n;
    impl->has_frame = 1;
    impl->state = ST_IDLE;

    return 0;
}

static int modbusFmt_pop_frame(AgvCommFormatIface* iface, uint8_t* out,
                               size_t* inout_len) {
    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl || !inout_len) {
        return -1;
    }
    if (!impl->has_frame) {
        return -2;  // 沒有 frame 可讀
    }

    if (*inout_len < impl->frame_len) {
        // 呼叫者給的緩衝區太小
        return -3;
    }

    memcpy(out, impl->frame_buf, impl->frame_len);
    *inout_len = impl->frame_len;
    impl->has_frame = 0;

    return 0;
}

static int modbusFmt_make_frame(AgvCommFormatIface* iface,
                                const uint8_t* payload, size_t len,
                                uint8_t* out, size_t* inout_len) {
    ModbusRtuFmtImpl* impl = (ModbusRtuFmtImpl*)iface->impl;
    if (!impl || !impl->cfg || !inout_len) {
        return -1;
    }

    const AgvCommFmtModbusRtuCfg* cfg = impl->cfg;

    // 總長度 = payload + CRC(2)
    size_t total_len = len + 2;

    if (total_len > cfg->max_frame_size) {
        return -2;
    }
    if (*inout_len < total_len) {
        // 呼叫者 out buffer 不夠大
        return -3;
    }

    // 先拷貝 payload
    memcpy(out, payload, len);

    // 計算 CRC
    uint16_t crc = modbus_crc16(&cfg->crc_cfg, out, len);

    // Modbus RTU 是低位在前
    out[len] = (uint8_t)(crc & 0x00FF);             // CRC_L
    out[len + 1] = (uint8_t)((crc >> 8) & 0x00FF);  // CRC_H

    *inout_len = total_len;
    return 0;
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