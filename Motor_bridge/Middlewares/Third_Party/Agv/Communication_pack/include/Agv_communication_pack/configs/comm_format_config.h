#ifndef AGV_COMMUNICATION_PACK__COMM_FORMAT_CONFIG_H_
#define AGV_COMMUNICATION_PACK__COMM_FORMAT_CONFIG_H_

#include <stdint.h>
#include <stdlib.h>

typedef enum { AGV_COMM_FMT_CSV, AGV_COMM_FMT_MODBUS } AgvCommFmtType;

typedef struct {
    uint16_t crc_poly;
    uint16_t crc_init;
} CrcCfg;

typedef struct {
    uint16_t header0;
    uint16_t header1;
    uint16_t tail0;
    uint16_t tail1;
    size_t max_frame_len;
    CrcCfg crc_cfg;
} AgvCommFmtRosCfg;

typedef struct {
    size_t max_frame_len;
    CrcCfg crc_cfg;
    uint16_t interframe_chars_x10;  // Modbus RTU: 3.5 chars = 35
} AgvCommFmtModbusRtuCfg;

typedef struct {
    AgvCommFmtType type;
    union {
        AgvCommFmtRosCfg ros;
        AgvCommFmtModbusRtuCfg mdbsRtu;
    } u;

} AgvCommFmtCfg;

#endif  // AGV_COMMUNICATION_PACK__COMM_FORMAT_CONFIG_H_