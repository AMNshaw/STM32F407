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
    uint16_t shared_id;
    size_t max_payload_len;

    size_t axis_count;

    size_t byte_per_rgstr;
    size_t num_rgster_per_cmd;
    size_t num_read_cmd;
    size_t num_write_cmd;

    int32_t operation_type;
    int32_t operation_trigger;

    struct {
        uint16_t driver_status;
        uint16_t real_vel;
        uint16_t real_pos;
        uint16_t present_alarm;
    } reg_address_read;

    struct {
        uint16_t cmd_vel;
        uint16_t cmd_acc;
        uint16_t cmd_dec;
        uint16_t cmd_op;
        uint16_t cmd_trg;
    } reg_address_write;

} AgvCommPrtclBlvrCfg;

typedef struct {
    AgvCommPrtclType type;
    union {
        AgvCommPrtclHostCfg host_csv;
        AgvCommPrtclBlvrCfg blvr;
    } u;
} AgvCommPrtclCfg;

#endif  // AGV_COMMUNICATION_PACK__COMM_PROTOCOL_CONFIG_H_