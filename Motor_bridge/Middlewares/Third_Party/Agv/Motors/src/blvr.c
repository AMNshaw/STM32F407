#include "Agv_communication_pack/communication_builder.h"
#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/format/modbus_rtu_format.h"
#include "Agv_communication_pack/msg/blvr_motor_msgs.h"
#include "Agv_communication_pack/protocol/blvr_protocol.h"
#include "Agv_communication_pack/protocol_defs/blvr_protocol_defs.h"
#include "Agv_core/agv_types.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"
#include "Agv_core/modules/motors_base.h"
#include "Agv_core/utils.h"
#include "Agv_motors/blvr_config.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifndef M_PI
#define M_PI (float)3.1415926535897932384626433832
#endif

/**
 * private declarations
 */

typedef enum { DRIVER, MOVE } PendingCmd;

typedef struct {
    int32_t driver_st;
    int32_t rl_pos;
    int32_t rl_rpm;
    int32_t alrm;
} BlvrReadBuff;

typedef struct {
    int32_t des_vel;
    int32_t des_acc;
    int32_t des_dec;
    int32_t spd_ctrl;
    int32_t trigger;
    int32_t driver_cmd;
} BlvrWriteBuff;

typedef struct {
    const AgvMotorBlvrConfig* cfg;

    AgvCommLinkIface link;
    AgvCommFormatIface fmt;
    AgvCommProtocolIface prtcl;

    PendingCmd pending_cmd;

    BlvrReadBuff* read_buf;
    BlvrWriteBuff* write_buf;

    SemaphoreHandle_t mutex_buf_read;
    SemaphoreHandle_t mutex_buf_write;
} MotorsBlvrImpl;

// Private impl
static int Motors_blvr_destroy(AgvMotorsBase* base);

static int blvr_reset(AgvMotorsBase* base);

static int blvr_on_off(AgvMotorsBase* base, bool state);

static int blvr_set_des_vel(AgvMotorsBase* base, const WheelsVel* vel_in);

static int blvr_get_curr_vel(AgvMotorsBase* base, WheelsVel* vel_out);

static int blvr_get_curr_ang(AgvMotorsBase* base, WheelsAng* ang_out);

static int blvr_get_state(AgvMotorsBase* base);

static int blvr_read_and_write(AgvMotorsBase* base);

// Helper
int32_t rad_s_to_regVelUnit(float rad_s, float unit_rpm);

float regVelUnit_to_rad_s(int32_t reg_val, float unit_rpm);

int32_t rad_to_regAngUnit(float rad, float unit_degree);

float regAngUnit_to_rad(int32_t reg_val, float unit_degree);

bool is_servo_on(int32_t driver_st);

bool is_alarm(int32_t driver_st);

bool is_moving(int32_t driver_st);

/**
 * Private definitions
 */

int Motors_blvr_create(AgvMotorsBase* out, const AgvMotorBlvrConfig* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;
    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)malloc(sizeof(MotorsBlvrImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;
    impl->cfg = cfg;

    char* name = "BLV-R motor";

    impl->read_buf =
        (BlvrReadBuff*)malloc(cfg->axis_count * sizeof(BlvrReadBuff));
    if (!impl->read_buf) return AGV_ERR_NO_MEMORY;
    impl->write_buf =
        (BlvrWriteBuff*)malloc(cfg->axis_count * sizeof(BlvrWriteBuff));
    if (!impl->write_buf) return AGV_ERR_NO_MEMORY;

    impl->pending_cmd = MOVE;

    for (size_t i = 0; i < cfg->axis_count; ++i) {
        BlvrReadBuff* read_buf = &impl->read_buf[i];
        BlvrWriteBuff* write_buf = &impl->write_buf[i];

        read_buf->alrm = 0;
        read_buf->driver_st = 0;
        read_buf->rl_pos = 0;
        read_buf->rl_rpm = 0;

        write_buf->des_acc = 60;
        write_buf->des_dec = 60;
        write_buf->des_vel = 0;
        write_buf->spd_ctrl = 0;
        write_buf->trigger = 0;
        write_buf->driver_cmd = 0;
    }

    int code;
    LOG(name, "Creating uart_rs485 link...");
    code = Link_uart_rs485_create(&impl->link, &cfg->uart_cfg);
    if (code < 0) {
        impl->link.destroy(&impl->link);
        free(impl->read_buf);
        free(impl->write_buf);
        free(impl);
        return code;
    }
    LOG(name, "Creating modbus format...");
    code = Format_modbus_create(&impl->fmt, &cfg->modbus_cfg);
    if (code < 0) {
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        free(impl->read_buf);
        free(impl->write_buf);
        free(impl);
        return code;
    }
    LOG(name, "Creating BLV-R protocol...");
    code = Protocol_blvr_create(&impl->prtcl, &cfg->prtcl_blvr_cfg);
    if (code < 0) {
        impl->prtcl.destroy(&impl->prtcl);
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        free(impl->read_buf);
        free(impl->write_buf);
        free(impl);
        return code;
    }

    impl->mutex_buf_read = xSemaphoreCreateMutex();
    impl->mutex_buf_write = xSemaphoreCreateMutex();
    if (impl->mutex_buf_read == NULL || impl->mutex_buf_write == NULL) {
        free(impl->read_buf);
        free(impl->write_buf);
        free(impl);
        return AGV_ERR_MUTEX_FAIL;
    }

    out->name = name;
    out->impl = impl;
    out->reset = blvr_reset;
    out->on_off = blvr_on_off;
    out->set_des_wheel_vel = blvr_set_des_vel;
    out->get_curr_wheels_ang = blvr_get_curr_ang;
    out->get_curr_wheels_vel = blvr_get_curr_vel;
    out->get_state = blvr_get_state;
    out->readTo_and_writeFrom_buffer = blvr_read_and_write;
    out->destroy = Motors_blvr_destroy;

    LOG(out->name, "Motors module created");
    return AGV_OK;
}

static int Motors_blvr_destroy(AgvMotorsBase* base) {
    if (!base) return AGV_OK;

    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (impl) {
        if (impl->read_buf) {
            free(impl->read_buf);
            impl->read_buf = NULL;
        }
        if (impl->write_buf) {
            free(impl->write_buf);
            impl->write_buf = NULL;
        }
        if (impl->mutex_buf_read) {
            vSemaphoreDelete(impl->mutex_buf_read);
            impl->mutex_buf_read = NULL;
        }
        if (impl->mutex_buf_write) {
            vSemaphoreDelete(impl->mutex_buf_write);
            impl->mutex_buf_write = NULL;
        }
        if (impl->prtcl.destroy) impl->prtcl.destroy(&impl->prtcl);
        if (impl->fmt.destroy) impl->fmt.destroy(&impl->fmt);
        if (impl->link.destroy) impl->link.destroy(&impl->link);
        free(impl);
    }

    base->impl = NULL;
    base->set_des_wheel_vel = NULL;
    base->get_curr_wheels_ang = NULL;
    base->get_curr_wheels_vel = NULL;
    base->get_state = NULL;
    base->readTo_and_writeFrom_buffer = NULL;
    base->destroy = NULL;

    return AGV_OK;
}

static int blvr_reset(AgvMotorsBase* base) {
    if (!base) return AGV_ERR_INVALID_ARG;
    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;
    size_t axis_count = impl->cfg->axis_count;

    int32_t curr_st[axis_count];
    xSemaphoreTake(impl->mutex_buf_read, portMAX_DELAY);
    for (size_t i = 0; i < axis_count; ++i) {
        curr_st[i] = impl->read_buf[i].driver_st;
    }
    xSemaphoreGive(impl->mutex_buf_read);
    xSemaphoreTake(impl->mutex_buf_write, portMAX_DELAY);

    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        if (is_alarm(curr_st[i])) {
            impl->pending_cmd = DRIVER;
            impl->write_buf[i].driver_cmd = BLVR_DRIVER_RESET_ALARM;
        }
    }
    xSemaphoreGive(impl->mutex_buf_write);

    return AGV_OK;
}

static int blvr_on_off(AgvMotorsBase* base, bool state) {
    if (!base) return AGV_ERR_INVALID_ARG;
    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;
    const AgvMotorBlvrConfig* cfg = impl->cfg;
    size_t axis_count = cfg->axis_count;

    int32_t curr_st[axis_count];
    xSemaphoreTake(impl->mutex_buf_read, portMAX_DELAY);
    for (size_t i = 0; i < axis_count; ++i) {
        curr_st[i] = impl->read_buf[i].driver_st;
    }
    xSemaphoreGive(impl->mutex_buf_read);
    xSemaphoreTake(impl->mutex_buf_write, portMAX_DELAY);
    for (size_t i = 0; i < axis_count; ++i) {
        bool curr_on = is_servo_on(curr_st[i]);
        bool should_cmd = !(state && curr_on);
        if (should_cmd) {
            impl->pending_cmd = DRIVER;
            impl->write_buf[i].driver_cmd =
                state ? BLVR_DRIVER_SERVO_ON : BLVR_DRIVER_SERVO_OFF;
        }
    }
    xSemaphoreGive(impl->mutex_buf_write);

    return AGV_OK;
}

static int blvr_set_des_vel(AgvMotorsBase* base, const WheelsVel* vel_in) {
    if (!base || !vel_in) return AGV_ERR_INVALID_ARG;
    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;

    float unit_rpm = impl->cfg->unit_vel_rpm;
    float gear_ratio = impl->cfg->gearRatio_motor_to_wheel;

    xSemaphoreTake(impl->mutex_buf_write, portMAX_DELAY);
    if (impl->pending_cmd != DRIVER) {
        impl->pending_cmd = MOVE;
    }
    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        float omega_motor = vel_in->data[i] * gear_ratio;
        impl->write_buf[i].des_vel = rad_s_to_regVelUnit(omega_motor, unit_rpm);
    }
    xSemaphoreGive(impl->mutex_buf_write);

    return AGV_OK;
}

static int blvr_get_curr_vel(AgvMotorsBase* base, WheelsVel* vel_out) {
    if (!base || !vel_out) return AGV_ERR_INVALID_ARG;
    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;
    float unit_rpm = impl->cfg->unit_vel_rpm;
    float gear_ratio = impl->cfg->gearRatio_motor_to_wheel;

    xSemaphoreTake(impl->mutex_buf_read, portMAX_DELAY);
    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        float omega_motor =
            regVelUnit_to_rad_s(impl->read_buf[i].rl_rpm, unit_rpm);
        vel_out->data[i] = omega_motor / gear_ratio;
    }
    xSemaphoreGive(impl->mutex_buf_read);

    return AGV_OK;
}

static int blvr_get_curr_ang(AgvMotorsBase* base, WheelsAng* ang_out) {
    if (!base || !ang_out) return AGV_ERR_INVALID_ARG;
    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;
    float unit_degree = impl->cfg->unit_step_degree;
    float gear_ratio = impl->cfg->gearRatio_motor_to_wheel;

    xSemaphoreTake(impl->mutex_buf_read, portMAX_DELAY);
    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        ang_out->data[i] =
            regAngUnit_to_rad(impl->read_buf[i].rl_pos, unit_degree) /
            gear_ratio;
    }
    xSemaphoreGive(impl->mutex_buf_read);

    return AGV_OK;
}

static int blvr_get_state(AgvMotorsBase* base) {
    if (!base) return AGV_ERR_INVALID_ARG;
    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (!impl || !impl->cfg) return AGV_ERR_NO_MEMORY;

    size_t axis_count = impl->cfg->axis_count;
    int32_t curr_state[axis_count];
    xSemaphoreTake(impl->mutex_buf_read, portMAX_DELAY);
    for (size_t i = 0; i < axis_count; ++i) {
        curr_state[i] = impl->read_buf[i].driver_st;
    }
    xSemaphoreGive(impl->mutex_buf_read);

    return AGV_OK;
}

static int blvr_read_and_write(AgvMotorsBase* base) {
    // Convenience pointers to the composed communication interfaces
    if (!base) return AGV_ERR_INVALID_ARG;

    MotorsBlvrImpl* impl = (MotorsBlvrImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    if (!link || !fmt || !prtcl) return AGV_ERR_NO_MEMORY;

    // Send read_write request
    // LOG(base->name, "Copying msg");
    BlvrMsg msg_send = {0};
    msg_send.msg_type = READ_WRITE;

    xSemaphoreTake(impl->mutex_buf_write, portMAX_DELAY);
    if (impl->pending_cmd == MOVE) {
        msg_send.u.write_msg.write_type = SET_MOVE;
        for (size_t i = 0; i < impl->cfg->axis_count; i++) {
            msg_send.u.write_msg.msgs[i].des_vel = impl->write_buf[i].des_vel;
            msg_send.u.write_msg.msgs[i].des_acc = impl->write_buf[i].des_acc;
            msg_send.u.write_msg.msgs[i].des_dec = impl->write_buf[i].des_dec;
            msg_send.u.write_msg.msgs[i].spd_ctrl =
                BLVR_OPERATION_TYPE_CONTINUOUS_SPD_CTRL_MOTION_EXT;
            msg_send.u.write_msg.msgs[i].trigger =
                BLVR_OPERATION_TRIGGER_NORMAL_START;
        }
    } else if (impl->pending_cmd == DRIVER) {
        msg_send.u.write_msg.write_type = SET_DRIVER;
        for (size_t i = 0; i < impl->cfg->axis_count; i++) {
            msg_send.u.write_msg.msgs[i].driver_cmd =
                impl->write_buf[i].driver_cmd;
        }
    }
    xSemaphoreGive(impl->mutex_buf_write);

    int code = AGV_OK;

    // LOG(base->name, "Making payload");
    size_t blvr_payload_len = impl->cfg->prtcl_blvr_cfg.max_payload_len;
    uint8_t payload_built[blvr_payload_len];
    code = prtcl->make_payload(prtcl, &msg_send, sizeof(msg_send),
                               payload_built, &blvr_payload_len);
    // LOG(base->name, "Payload len: %d", blvr_payload_len);
    if (code != AGV_OK) return code;
    size_t frame_len = impl->cfg->modbus_cfg.max_frame_len;
    uint8_t frame_made[frame_len];
    code = fmt->make_frame(fmt, payload_built, blvr_payload_len, frame_made,
                           &frame_len);
    if (code != AGV_OK) return code;
    // LOG(base->name, "Frame len: %d, Frame:", frame_len);
    // for (size_t i = 0; i < frame_len; i++) {
    //     printf("%02X ", frame_made[i]);
    // }
    // printf("\n");

    // LOG(base->name, "Sending frame");
    code = link->send_bytes(link, frame_made, frame_len);
    if (code != AGV_OK) return code;

    // We have no module, skip this part first

    // Receive read_write response
    // LOG(base->name, "Receiving bytes");
    size_t expected_bytes_len;
    code =
        BlvrProto_get_response_frame_len(prtcl, &msg_send, &expected_bytes_len);
    code = modbusFmt_set_check_item(fmt, frame_made[0], frame_made[1],
                                    expected_bytes_len);
    code = AGV_ERR_COMM_FMT_NO_COMPLETE_FRAME;
    while (code == AGV_ERR_COMM_FMT_NO_COMPLETE_FRAME) {
        size_t data_len = impl->cfg->uart_cfg.max_data_len;
        uint8_t data_rcv[frame_len];
        uint32_t data_timeStamp;
        code = link->read_buf(link, data_rcv, &data_len, &data_timeStamp);
        if (code != AGV_OK) return code;
        // LOG(base->name, "Bytes received, len: %d", data_len);
        // for (size_t i = 0; i < data_len; i++) {
        //     printf("%02X ", data_rcv[i]);
        // }
        // printf("\n");

        // LOG(base->name, "feeding bytes");
        code = fmt->feed_bytes(fmt, data_rcv, data_len);
    }
    if (code != AGV_OK) return code;
    // LOG(base->name, "popping payload");
    size_t payload_len = impl->cfg->modbus_cfg.max_frame_len;
    uint8_t payload[payload_len];
    code = fmt->pop_payload(fmt, payload, &payload_len);
    if (code != AGV_OK) return code;
    // LOG(base->name, "payload len: %d", payload_len);

    // LOG(base->name, "feeding payload");
    code = prtcl->feed_payload(prtcl, payload, payload_len);
    if (code != AGV_OK) return code;

    // LOG(base->name, "popping msg");
    BlvrMsg msg_rcv;
    code = prtcl->pop_msg(prtcl, &msg_rcv, sizeof(msg_rcv));
    if (code != AGV_OK) return code;
    xSemaphoreTake(impl->mutex_buf_read, portMAX_DELAY);
    for (size_t i = 0; i < impl->cfg->axis_count; i++) {
        impl->read_buf[i].driver_st = msg_rcv.u.read_msg.msgs[i].driver_st;
        impl->read_buf[i].rl_pos = msg_rcv.u.read_msg.msgs[i].rl_pos;
        impl->read_buf[i].rl_rpm = msg_rcv.u.read_msg.msgs[i].rl_rpm;
        impl->read_buf[i].alrm = msg_rcv.u.read_msg.msgs[i].alrm;
    }
    xSemaphoreGive(impl->mutex_buf_read);

    xSemaphoreTake(impl->mutex_buf_write, portMAX_DELAY);
    if (impl->pending_cmd == DRIVER) {
        for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
            impl->write_buf[i].driver_cmd = 0;
        }
        impl->pending_cmd = MOVE;
    }
    xSemaphoreGive(impl->mutex_buf_write);

    return AGV_OK;
}

int32_t rad_s_to_regVelUnit(float rad_s, float unit_rpm) {
    float rpm_f = rad_s * 60.0f / (2.0f * M_PI);
    return (int32_t)lroundf(rpm_f / unit_rpm);
}

float regVelUnit_to_rad_s(int32_t reg_val, float unit_rpm) {
    float rpm = (float)reg_val * unit_rpm;
    return rpm * 2.0f * M_PI / 60.0f;
}

int32_t rad_to_regAngUnit(float rad, float unit_degree) {
    float degree = rad * 180.0f / M_PI;
    return (int32_t)lroundf(degree / unit_degree);
}

float regAngUnit_to_rad(int32_t reg_val, float unit_degree) {
    float degree = (float)reg_val * unit_degree;

    return degree * M_PI / 180.0f;
}

bool is_servo_on(int32_t driver_st) {
    return (driver_st & BLVR_DRIVER_STATE_ON) != 0;
}

bool is_alarm(int32_t driver_st) {
    return (driver_st & BLVR_DRIVER_BLVR_STATE_ALARM) != 0;
}

bool is_moving(int32_t driver_st) {
    return (driver_st & BLVR_DRIVER_BLVR_STATE_MOVING) != 0;
}