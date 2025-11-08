#ifndef AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_
#define AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_

#include "stdlib.h"

typedef struct {
    size_t link_max_data_size;
    size_t format_max_buffer_size;
    size_t format_max_frame_size;
    size_t protocol_max_frame_size;

    int32_t operation_type;
    int32_t operation_trigger;
} Agv_Blvr_config;

#endif  // AGV_MOTOR_COMMUNICATION__BLVR_CONFIG_H_