#include "Agv_communication_pack/format/ros_format.h"

int Format_ros_create(AgvCommFormatIface* out, AgvCommFmtRosCfg* cfg) {
    if (!out || !cfg) return -1;
    RosFmtImpl* impl = (RosFmtImpl*)malloc(sizeof(RosFmtImpl));
    if (!impl) return -2;

    memset(impl, 0, sizeof(*impl));
    impl->cfg = cfg;
    impl->state = ST_WAIT_H0;

    // data
    impl->data_buf = (uint8_t*)malloc(cfg->max_buf_len * sizeof(uint8_t));
    if (!impl->data_buf) {
        free(impl);
        return -3;
    }
    impl->data_idx = 0;

    // frame
    impl->frame_buf = (uint8_t*)malloc(
        cfg->max_frame_len * sizeof(uint8_t));  // 1 for cmd, 1 for size
    if (!impl->frame_buf) {
        free(impl->data_buf);
        free(impl);
        return -4;
    }
    impl->frame_len = 0;
    impl->has_frame = 0;

    out->impl = impl;
    out->feed_data = rosFmt_feed_data;
    out->pop_frame = rosFmt_pop_frame;
    out->make_frame = rosFmt_make_frame;
    out->destroy = rosFmt_destroy;

    return 0;
}

static int rosFmt_destroy(AgvCommFormatIface* iface) {
    if (!iface || !iface->impl) return 0;

    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;

    if (impl->data_buf) {
        free(impl->data_buf);
        impl->data_buf = NULL;
    }

    if (impl->frame_buf) {
        free(impl->frame_buf);
        impl->frame_buf = NULL;
    }

    free(impl);
    iface->impl = NULL;
    iface->feed_data = NULL;
    iface->pop_frame = NULL;
    iface->make_frame = NULL;
    iface->destroy = NULL;

    return 0;
}

static int rosFmt_feed_data(AgvCommFormatIface* iface, const uint8_t* data,
                            size_t data_len) {
    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;
    if (!impl) return -1;
    const AgvCommFmtRosCfg* cfg = impl->cfg;
    if (!cfg) return -1;

    for (size_t i = 0; i < data_len; ++i) {
        uint8_t b = data[i];

        switch (impl->state) {
            case ST_WAIT_H0:
                if (b == cfg->header0) {
                    impl->state = ST_WAIT_H1;
                }
                break;

            case ST_WAIT_H1:
                if (b == cfg->header1) {
                    impl->state = ST_WAIT_CMD;
                } else {
                    impl->state = ST_WAIT_H0;
                }
                break;

            case ST_WAIT_CMD:
                impl->cmd = b;
                impl->state = ST_WAIT_SIZE;
                break;

            case ST_WAIT_SIZE:
                impl->len = b;
                if (impl->len == 0 || impl->len > sizeof(impl->data_buf)) {
                    impl->state = ST_WAIT_H0;
                } else {
                    impl->data_idx = 0;
                    impl->state = ST_WAIT_DATA;
                }
                break;

            case ST_WAIT_DATA:
                impl->data_buf[impl->data_idx++] = b;
                if (impl->data_idx >= impl->len) {
                    impl->state = ST_WAIT_CRC;
                }
                break;

            case ST_WAIT_CRC: {
                // 計算 CRC: 一般會是 [header0, header1, cmd, size, data...]
                uint8_t tmp[2 + 1 + 1 + sizeof(impl->data_buf)];
                size_t len = 0;
                tmp[len++] = cfg->header0;
                tmp[len++] = cfg->header1;
                tmp[len++] = impl->cmd;
                tmp[len++] = impl->len;
                memcpy(&tmp[len], impl->data_buf, impl->len);
                len += impl->len;

                uint8_t crc = crc8_compute(&cfg->crc_cfg, tmp, len);
                if (crc != b) {
                    // CRC 錯誤，整包丟掉
                    impl->state = ST_WAIT_H0;
                } else {
                    impl->state = ST_WAIT_T0;
                }
                break;
            }

            case ST_WAIT_T0:
                if (b == cfg->tail0) {
                    impl->state = ST_WAIT_T1;
                } else {
                    impl->state = ST_WAIT_H0;
                }
                break;

            case ST_WAIT_T1:
                if (b == cfg->tail1) {
                    // 完整 frame 收到，把 payload 暫存給 pop_frame
                    impl->frame_buf[0] = impl->cmd;
                    impl->frame_buf[1] = impl->len;
                    memcpy(&impl->frame_buf[2], impl->data_buf, impl->len);
                    impl->frame_len = 2 + impl->len;
                    impl->has_frame = 1;
                }
                // 無論成功失敗，都回到等 header
                impl->state = ST_WAIT_H0;
                break;
        }
    }

    return 0;
}

static int rosFmt_pop_frame(AgvCommFormatIface* iface, uint8_t* out,
                            size_t* inout_len) {
    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;
    if (!impl || !out || !inout_len) return -1;

    if (!impl->has_frame) return -2;

    if (*inout_len < impl->frame_len) {
        return -3;  // buffer 太小
    }

    memcpy(out, impl->frame_buf, impl->frame_len);
    *inout_len = impl->frame_len;
    impl->has_frame = 0;
    return 0;
}

static int rosFmt_make_frame(AgvCommFormatIface* iface, const uint8_t* payload,
                             size_t len, uint8_t* out, size_t* inout_len) {
    if (!iface || !payload || !out || !inout_len) return -1;
    if (len < 2) return -2;

    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;
    if (!impl) return -1;
    const AgvCommFmtRosCfg* cfg = impl->cfg;
    if (!cfg) return -1;

    uint8_t cmd = payload[0];
    uint8_t size = payload[1];
    if ((size_t)(2 + size) != len) return -3;

    // 檢查 buffer
    size_t need = 2 + len + 1 + 2;  // header2 + payload(len) + crc + tail2
    if (*inout_len < need) return -4;

    size_t idx = 0;
    out[idx++] = cfg->header0;
    out[idx++] = cfg->header1;
    out[idx++] = cmd;
    out[idx++] = size;
    memcpy(&out[idx], &payload[2], size);
    idx += size;

    // 計算 CRC
    uint8_t crc = crc8_compute(&cfg->crc_cfg, out,
                               idx);  // [header0, header1, cmd, size, data...]
    out[idx++] = crc;
    out[idx++] = cfg->tail0;
    out[idx++] = cfg->tail1;

    *inout_len = idx;
    return 0;
}

static uint8_t crc8_compute(const CrcCfg* cfg, const uint8_t* data,
                            size_t len) {
    uint8_t crc = cfg->crc_init;
    const uint8_t poly = cfg->crc_poly;  // 你可以換成實際用的
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}