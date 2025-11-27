#ifndef AGV_COMMUNICATION_PACK__BLVR_MOTOR_MSGS_H_
#define AGV_COMMUNICATION_PACK__BLVR_MOTOR_MSGS_H_

#include <stdlib.h>

#include "Agv_core/agv_types.h"

/**
 * Motor msg
 */
typedef enum { READ, WRITE, READ_WRITE } BlvrMsgType;
typedef enum { SET_DRIVER, SET_MOVE } BlvrWriteType;

typedef struct {
    struct {
        int32_t driver_st;
        int32_t rl_pos;
        int32_t rl_rpm;
        int32_t alrm;
    } msgs[4];
} BlvrReadMsg;

typedef struct {
    BlvrWriteType write_type;
    struct {
        int32_t des_vel;  // vel of wheel
        int32_t des_acc;
        int32_t des_dec;
        int32_t spd_ctrl;
        int32_t trigger;
        int32_t driver_cmd;
    } msgs[4];
} BlvrWriteMsg;

typedef struct {
    BlvrMsgType msg_type;
    union {
        BlvrReadMsg read_msg;
        BlvrWriteMsg write_msg;
    } u;
} BlvrMsg;

#endif  // AGV_COMMUNICATION_PACK__BLVR_MOTOR_MSGS_H_