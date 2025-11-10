#ifndef AGV_COMMUNICATION_PACK__ROS_FORMAT_H_
#define AGV_COMMUNICATION_PACK__ROS_FORMAT_H_

#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_format_config.h"

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
    uint8_t len;
    uint8_t* data_buf;
    size_t data_idx;

    uint8_t* frame_buf;
    size_t frame_len;
    int has_frame;
} RosFmtImpl;

static int rosFmt_destroy(AgvCommFormatIface* iface);

static int rosFmt_feed_data(AgvCommFormatIface* iface, const uint8_t* data,
                            size_t data_len);
static int rosFmt_pop_frame(AgvCommFormatIface* iface, uint8_t* out_frame,
                            size_t* frame_len);
static int rosFmt_make_frame(AgvCommFormatIface* iface, const uint8_t* payload,
                             size_t payload_len, uint8_t* out_frame,
                             size_t* frame_len);
static uint8_t crc8_compute(const CrcCfg* cfg, const uint8_t* data, size_t len);
#endif  // AGV_COMMUNICATION_PACK__FORMAT_ROS_FORMAT_H_