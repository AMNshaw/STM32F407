#ifndef AGV_COMMUNICATION_PACK__COMM_FORMAT_CONFIG_H_
#define AGV_COMMUNICATION_PACK__COMM_FORMAT_CONFIG_H_

#include <stdint.h>
#include <stdlib.h>

typedef enum { AGV_COMM_FMT_CSV, AGV_COMM_FMT_MODBUS } AgvCommFmtType;

typedef struct {
    size_t max_frame_len;
} AgvCommFmtRosCfg;

typedef struct {
    size_t max_frame_len;
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