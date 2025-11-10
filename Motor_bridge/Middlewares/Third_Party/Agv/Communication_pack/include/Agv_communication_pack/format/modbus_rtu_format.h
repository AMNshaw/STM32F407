#ifndef AGV_COMMUNICATION_PACK__MODBUS_RTU_FORMAT_H_
#define AGV_COMMUNICATION_PACK__MODBUS_RTU_FORMAT_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_format_config.h"

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

    uint8_t* frame_buf;
    size_t frame_len;
    int has_frame;
} ModbusRtuFmtImpl;

static int modbusFmt_feed_data(AgvCommFormatIface* iface,
                               const uint8_t* data_in, size_t data_len);

static int modbusFmt_pop_frame(AgvCommFormatIface* iface, uint8_t* frame_out,
                               size_t* frame_len);

static int modbusFmt_make_frame(AgvCommFormatIface* iface,
                                const uint8_t* payload_in, size_t payload_len,
                                uint8_t* frame_out, size_t* frame_len);

static int modbusFmt_destroy(AgvCommFormatIface* iface);

uint16_t modbus_crc16(const CrcCfg* cfg, const uint8_t* data, size_t len);

#endif  // AGV_COMMUNICATION_PACK__MODBUS_RTU_FORMAT_H_