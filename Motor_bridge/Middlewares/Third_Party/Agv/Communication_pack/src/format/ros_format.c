
#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"

/**
 * private declarations
 */

typedef enum {
    ST_WAIT_H0 = 0,
    ST_WAIT_H1,
    ST_WAIT_CMD,
    ST_WAIT_SIZE,
    ST_WAIT_DATA,
    ST_WAIT_CRC,
    ST_WAIT_T0,
    ST_WAIT_T1,
} RosFmtState;

typedef struct {
    const AgvCommFmtRosCfg* cfg;
    RosFmtState state;
    uint8_t cmd;
    uint8_t data_len;
    size_t data_idx;

    uint8_t* payload_buf;
    size_t payload_len;
    int has_payload;
} RosFmtImpl;

static int rosFmt_destroy(AgvCommFormatIface* iface);

static int rosFmt_feed_bytes(AgvCommFormatIface* iface, const uint8_t* bytes_in,
                             size_t bytes_len);
static int rosFmt_pop_payload(AgvCommFormatIface* iface, uint8_t* payload_out,
                              size_t* payload_len);
static int rosFmt_make_frame(AgvCommFormatIface* iface,
                             const uint8_t* payload_in, size_t payload_len,
                             uint8_t* frame_out, size_t* frame_len);
static uint8_t crc8_compute(const CrcCfg* cfg, const uint8_t* data, size_t len);

/**
 * Private definitions
 */

int Format_ros_create(AgvCommFormatIface* out, AgvCommFmtRosCfg* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;
    RosFmtImpl* impl = (RosFmtImpl*)malloc(sizeof(RosFmtImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;
    impl->cfg = cfg;

    impl->state = ST_WAIT_H0;
    impl->data_idx = 0;

    // frame
    size_t max_payload_len = cfg->max_frame_len - 5;  // 2h, 2t, 1c
    impl->payload_buf = (uint8_t*)malloc(max_payload_len * sizeof(uint8_t));
    if (!impl->payload_buf) return AGV_ERR_NO_MEMORY;

    impl->payload_len = 0;
    impl->has_payload = 0;

    out->impl = impl;
    out->feed_bytes = rosFmt_feed_bytes;
    out->pop_payload = rosFmt_pop_payload;
    out->make_frame = rosFmt_make_frame;
    out->destroy = rosFmt_destroy;

    return 0;
}

static int rosFmt_destroy(AgvCommFormatIface* iface) {
    if (!iface) return AGV_OK;

    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;
    if (impl) {
        if (impl->payload_buf) {
            free(impl->payload_buf);
            impl->payload_buf = NULL;
        }
        impl->cfg = NULL;
        free(impl);
    }

    iface->impl = NULL;
    iface->feed_bytes = NULL;
    iface->pop_payload = NULL;
    iface->make_frame = NULL;
    iface->destroy = NULL;

    return AGV_OK;
}

static int rosFmt_feed_bytes(AgvCommFormatIface* iface, const uint8_t* bytes,
                             size_t bytes_len) {
    if (!iface || !bytes || bytes_len == 0) return AGV_ERR_INVALID_ARG;
    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    const AgvCommFmtRosCfg* cfg = impl->cfg;
    if (!cfg) return AGV_ERR_NO_MEMORY;

    size_t max_data_len = impl->cfg->max_frame_len -
                          7;  // -2 headers, 1 cmd, 1 size, -2 tails, -1 crc
    uint8_t data[max_data_len];

    for (size_t i = 0; i < bytes_len; ++i) {
        uint8_t b = bytes[i];

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
                impl->data_len = b;
                if (impl->data_len > max_data_len) {
                    impl->state = ST_WAIT_H0;
                } else {
                    impl->data_idx = 0;
                    impl->state = ST_WAIT_DATA;
                }
                break;

            case ST_WAIT_DATA:
                data[impl->data_idx++] = b;
                if (impl->data_idx >= impl->data_len) {
                    impl->state = ST_WAIT_CRC;
                }
                break;

            case ST_WAIT_CRC: {
                uint8_t payload[2 + impl->data_idx];
                size_t payload_idx = 0;
                payload[payload_idx++] = impl->cmd;
                payload[payload_idx++] = impl->data_len;
                memcpy(&payload[payload_idx], data, impl->data_len);
                payload_idx += impl->data_len;
                uint8_t crc = crc8_compute(&cfg->crc_cfg, payload, payload_idx);
                if (crc != b) {
                    impl->state = ST_WAIT_H0;
                } else {
                    impl->state = ST_WAIT_T0;
                }
                break;
            }

            case ST_WAIT_T0: {
                if (b == cfg->tail0) {
                    impl->state = ST_WAIT_T1;
                } else {
                    impl->state = ST_WAIT_H0;
                }
                break;
            }

            case ST_WAIT_T1: {
                if (b == cfg->tail1) {
                    impl->payload_buf[0] = impl->cmd;
                    impl->payload_buf[1] = impl->data_len;
                    memcpy(&impl->payload_buf[2], data, impl->data_len);
                    impl->payload_len = 2 + impl->data_len;
                    impl->has_payload = 1;
                }
                impl->state = ST_WAIT_H0;
                break;
            }
        }
    }

    return AGV_OK;
}

static int rosFmt_pop_payload(AgvCommFormatIface* iface, uint8_t* payload_out,
                              size_t* payload_len) {
    if (!iface || !payload_out || !payload_len) return AGV_ERR_INVALID_ARG;
    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (!impl->has_payload) return AGV_ERR_COMM_FMT_NO_COMPLETE_FRAME;

    if (*payload_len < impl->payload_len) {
        return AGV_ERR_OUTPUT_OVERFLOW;  // buffer 太小
    }

    memcpy(payload_out, impl->payload_buf, impl->payload_len);
    *payload_len = impl->payload_len;
    impl->has_payload = 0;

    return AGV_OK;
}

static int rosFmt_make_frame(AgvCommFormatIface* iface, const uint8_t* payload,
                             size_t payload_len, uint8_t* frame_out,
                             size_t* frame_len) {
    if (!iface || !payload || payload_len == 0 || !frame_out || !frame_len)
        return AGV_ERR_INVALID_ARG;

    RosFmtImpl* impl = (RosFmtImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    const AgvCommFmtRosCfg* cfg = impl->cfg;
    if (!cfg) return AGV_ERR_NO_MEMORY;

    uint8_t cmd = payload[0];
    uint8_t data_len = payload[1];
    if ((size_t)(2 + data_len) != payload_len) return AGV_ERR_INVALID_ARG;

    size_t need =
        2 + payload_len + 1 + 2;  // header2 + payload(data_len) + crc + tail2
    if (*frame_len < need) return AGV_ERR_OUTPUT_OVERFLOW;

    size_t idx = 0;
    frame_out[idx++] = cfg->header0;
    frame_out[idx++] = cfg->header1;
    frame_out[idx++] = cmd;
    frame_out[idx++] = data_len;
    memcpy(&frame_out[idx], &payload[2], data_len);
    idx += data_len;

    uint8_t crc = crc8_compute(&cfg->crc_cfg, frame_out,
                               idx);  // [header0, header1, cmd, size, data...]
    frame_out[idx++] = crc;
    frame_out[idx++] = cfg->tail0;
    frame_out[idx++] = cfg->tail1;

    *frame_len = idx;
    return 0;
}

static uint8_t crc8_compute(const CrcCfg* cfg, const uint8_t* data,
                            size_t data_len) {
    uint8_t crc = cfg->crc_init;
    const uint8_t poly = cfg->crc_poly;  // 你可以換成實際用的
    for (size_t i = 0; i < data_len; ++i) {
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