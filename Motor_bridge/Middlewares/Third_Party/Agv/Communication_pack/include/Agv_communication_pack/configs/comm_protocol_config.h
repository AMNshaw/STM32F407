#ifndef AGV_COMMUNICATION_PACK__COMM_PROTOCOL_CONFIG_H_
#define AGV_COMMUNICATION_PACK__COMM_PROTOCOL_CONFIG_H_

#include <stdint.h>
#include <stdlib.h>

typedef enum { AGV_COMM_PRTCL_HOST, AGV_COMM_PRTCL_BLVR } AgvCommPrtclType;

typedef struct {
    uint8_t placeholder;
} AgvCommPrtclHostCfg;

typedef struct {
    uint8_t slave_id;
    uint8_t axis_count;

    uint16_t reg_rpm_base;
    uint16_t regs_per_axis_rpm;

    uint16_t reg_speed_cmd_base;
} AgvCommPrtclBlvrCfg;

typedef struct {
    AgvCommPrtclType type;
    union {
        AgvCommPrtclHostCfg host_csv;
        AgvCommPrtclBlvrCfg blvr;
    } u;
} AgvCommPrtclCfg;

#endif  // AGV_COMMUNICATION_PACK__COMM_PROTOCOL_CONFIG_H_