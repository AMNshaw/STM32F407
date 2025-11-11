#ifndef AGV_MOTOR_COMMUNICATION__BLVR_H_
#define AGV_MOTOR_COMMUNICATION__BLVR_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_core/agv_types.h"
#include "Agv_core/modules/motor_communication_base.h"
#include "Agv_motor_communication/blvr_config.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct {
    int32_t des_vel;
    int32_t des_acc;
    int32_t des_dec;
    int32_t spd_ctrl;
    int32_t trigger;

    int32_t driver_st;
    int32_t rl_pos;
    int32_t rl_rpm;
    int32_t alrm;
} BlvrBuff;

typedef struct {
    const AgvMotorBlvrConfig* cfg;

    AgvCommLinkIface link;
    AgvCommFormatIface fmt;
    AgvCommProtocolIface prtcl;

    BlvrBuff* buffer;

    SemaphoreHandle_t mutex_buf;
} MotorCommBlvrImpl;

static int Motor_comm_blvr_destroy(AgvMotorCommunicationBase* base);

static int blvr_set_des_vel(AgvMotorCommunicationBase* base,
                            const WheelsVel* in);

static int blvr_get_curr_vel(AgvMotorCommunicationBase* base, WheelsVel* out);

static int blvr_get_state(AgvMotorCommunicationBase* base);

static int blvr_read_and_write(AgvMotorCommunicationBase* base);

int32_t rad_s_to_regVel(float rad_s, float unit_rpm);

float regVel_to_rad_s(int32_t reg_val, float unit_rpm);

#endif  // AGV_MOTOR_COMMUNICATION__BLVR_H_