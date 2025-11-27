#ifndef AGV_COMMUNICATION_PACK__COMM_PROTOCOL_CONFIG_H_
#define AGV_COMMUNICATION_PACK__COMM_PROTOCOL_CONFIG_H_

#include <stdint.h>
#include <stdlib.h>

typedef enum { AGV_COMM_PRTCL_HOST, AGV_COMM_PRTCL_BLVR } AgvCommPrtclType;
/**
 * Host
 */
typedef struct {
    size_t max_payload_len;
} AgvCommPrtclHostCfg;

/**
 * Motor
 */

typedef struct {
    size_t max_payload_len;
    size_t axis_count;
} AgvCommPrtclBlvrCfg;

typedef struct {
    AgvCommPrtclType type;
    union {
        AgvCommPrtclHostCfg host_csv;
        AgvCommPrtclBlvrCfg blvr;
    } u;
} AgvCommPrtclCfg;

#endif  // AGV_COMMUNICATION_PACK__COMM_PROTOCOL_CONFIG_H_