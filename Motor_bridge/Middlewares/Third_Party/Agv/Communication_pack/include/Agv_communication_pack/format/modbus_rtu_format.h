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
    uint8_t* data_buf;
    size_t data_len;

    uint8_t* frame_buf;
    size_t frame_len;
    int has_frame;
} ModbusRtuFmtImpl;

// Modbus状态
static enum { SUCCESS = 0, ERROR } ModbusStatus;

static int modbusFmt_feed(AgvCommFormatIface* iface, const uint8_t* bytes,
                          size_t n);
static int modbusFmt_pop_frame(AgvCommFormatIface* iface, uint8_t* out,
                               size_t* inout_len);
static int modbusFmt_make_frame(AgvCommFormatIface* iface,
                                const uint8_t* payload, size_t len,
                                uint8_t* out, size_t* inout_len);

static uint16_t modbus_crc16(const CrcCfg* cfg, const uint8_t* data,
                             size_t len);

#endif  // AGV_COMMUNICATION_PACK__MODBUS_RTU_FORMAT_H_