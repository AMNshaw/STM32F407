#include "Agv_motor_communication/blvr.h"

#include "Agv_communication_pack/communication_builder.h"
#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_core/error_codes/error_common.h"

#ifndef M_PI
#define M_PI (float)3.1415926535897932384626433832
#endif

int Motor_comm_blvr_create(AgvMotorCommunicationBase* out,
                           const AgvBlvrConfig* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    MotorCommBlvrImpl* impl =
        (MotorCommBlvrImpl*)malloc(sizeof(MotorCommBlvrImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;
    impl->cfg = cfg;

    impl->buffer = (BlvrBuff*)malloc(cfg->axis_count * sizeof(BlvrBuff));
    if (!impl->buffer) return AGV_ERR_NO_MEMORY;

    impl->sem = xSemaphoreCreateMutex();
    if (impl->sem == NULL) {
        Free(impl->buffer);
        Free(impl);
        return AGV_ERR_MUTEX_FAIL;
    }

    int code;
    code = Link_uart_rs485_create(&impl->link, &cfg->uart_cfg);
    if (code < 0) {
        impl->link.destroy(&impl->link);
        Free(impl->buffer);
        Free(impl);
        return code;
    }
    code = Format_modbus_create(&impl->fmt, &cfg->modbus_cfg);
    if (code < 0) {
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        Free(impl->buffer);
        Free(impl);
        return code;
    }
    code = Protocol_blvr_create(&impl->prtcl, &cfg->prtcl_blvr_cfg);
    if (code < 0) {
        impl->prtcl.destroy(&impl->prtcl);
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        Free(impl->buffer);
        Free(impl);
        return code;
    }

    out->impl = impl;
    out->set_desired_vel_to_buffer = blvr_set_des_vel;
    out->get_vel_from_buffer = blvr_get_curr_vel;
    out->get_state_from_buffer = blvr_get_state;
    out->readTo_and_writeFrom_buffer = blvr_read_and_write;
    out->destroy = Motor_comm_blvr_destroy;

    return AGV_OK;
}

int Motor_comm_blvr_destroy(AgvMotorCommunicationBase* base) {
    if (!base || !base->impl) return -1;

    MotorCommBlvrImpl* impl = (MotorCommBlvrImpl*)base->impl;

    if (impl->prtcl.destroy) impl->prtcl.destroy(&impl->prtcl);
    if (impl->fmt.destroy) impl->fmt.destroy(&impl->fmt);
    if (impl->link.destroy) impl->link.destroy(&impl->link);

    free(impl);

    base->impl = NULL;
    base->set_desired_vel_to_buffer = NULL;
    base->get_vel_from_buffer = NULL;
    base->get_state_from_buffer = NULL;
    base->readTo_and_writeFrom_buffer = NULL;
    base->destroy = NULL;

    return 0;
}

static int blvr_set_des_vel(AgvMotorCommunicationBase* base,
                            const WheelsVel* in) {
    if (!base || !in) return AGV_ERR_INVALID_ARG;
    MotorCommBlvrImpl* impl = (MotorCommBlvrImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    float unit_rpm = impl->cfg->prtcl_blvr_cfg.unit_rpm;

    xSemaphoreTake(impl->sem, portMAX_DELAY);
    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        impl->buffer[i].des_vel = rad_s_to_regVel(in->w[i], unit_rpm);
    }
    xSemaphoreGive(impl->sem);

    return AGV_OK;
}

static int blvr_get_curr_vel(AgvMotorCommunicationBase* base, WheelsVel* out) {
    if (!base || !out) return AGV_ERR_INVALID_ARG;
    MotorCommBlvrImpl* impl = (MotorCommBlvrImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    float unit_rpm = impl->cfg->prtcl_blvr_cfg.unit_rpm;

    xSemaphoreTake(impl->sem, portMAX_DELAY);
    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        out->w[i] = regVel_to_rad_s(impl->buffer[i].des_vel, unit_rpm);
    }
    xSemaphoreGive(impl->sem);
}

static int blvr_get_state(AgvMotorCommunicationBase* base) {
    if (!base) return AGV_ERR_INVALID_ARG;
    MotorCommBlvrImpl* impl = (MotorCommBlvrImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    xSemaphoreTake(impl->sem, portMAX_DELAY);
    // TODO: get the state back depneds on the state code defined in the file
    xSemaphoreGive(impl->sem);
}

static int blvr_read_and_write(AgvMotorCommunicationBase* base) {
    // Convenience pointers to the composed communication interfaces
    if (!base) return AGV_ERR_INVALID_ARG;

    MotorCommBlvrImpl* impl = (MotorCommBlvrImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    if (!link || !fmt || !prtcl) return AGV_ERR_NO_MEMORY;

    // Send read_write request
    AgvCommMsg msg_send;
    msg_send.msg_type = MOTOR_MSG;
    msg_send.u.motors_msg.type = READ_WRITE;

    xSemaphoreTake(impl->sem, portMAX_DELAY);
    for (size_t i = 0; i < 4; i++) {
        msg_send.u.motors_msg.msgs[i].des_vel = impl->buffer[i].des_vel;
        msg_send.u.motors_msg.msgs[i].des_acc = impl->buffer[i].des_acc;
        msg_send.u.motors_msg.msgs[i].des_dec = impl->buffer[i].des_dec;
        msg_send.u.motors_msg.msgs[i].spd_ctrl =
            impl->cfg->prtcl_blvr_cfg.operation_type;
        msg_send.u.motors_msg.msgs[i].trigger =
            impl->cfg->prtcl_blvr_cfg.operation_trigger;
    }
    xSemaphoreGive(impl->sem);

    size_t blvr_payload_len = impl->cfg->prtcl_blvr_cfg.max_payload_len;
    uint8_t payload_built[blvr_payload_len];
    prtcl->make_payload(prtcl, &msg_send, payload_built, &blvr_payload_len);
    size_t frame_len = impl->cfg->modbus_cfg.max_frame_len;
    uint8_t frame_made[frame_len];
    fmt->make_frame(fmt, payload_built, blvr_payload_len, frame_made,
                    &frame_len);
    link->send_bytes(link, frame_made, frame_len);

    // Receive read write response
    size_t data_rcv_len = impl->cfg->modbus_cfg.max_buf_len;
    uint8_t data_rcv[data_rcv_len];
    link->recv_bytes(link, data_rcv, data_rcv_len);

    size_t modbus_frame_len = impl->cfg->modbus_cfg.max_frame_len;
    uint8_t frame_popped[modbus_frame_len];
    fmt->feed_data(fmt, data_rcv, data_rcv_len);
    fmt->pop_frame(fmt, frame_popped, &modbus_frame_len);
    prtcl->feed_frame(prtcl, frame_popped, modbus_frame_len);

    AgvCommMsg msg_rcv;
    msg_rcv.msg_type = MOTOR_MSG;
    prtcl->pop_msg(prtcl, &msg_rcv);

    xSemaphoreTake(impl->sem, portMAX_DELAY);
    for (size_t i = 0; i < 4; i++) {
        impl->buffer[i].driver_st = msg_rcv.u.motors_msg.msgs[i].driver_st;
        impl->buffer[i].rl_pos = msg_rcv.u.motors_msg.msgs[i].rl_pos;
        impl->buffer[i].rl_rpm = msg_rcv.u.motors_msg.msgs[i].rl_rpm;
        impl->buffer[i].alrm = msg_rcv.u.motors_msg.msgs[i].alrm;
    }
    xSemaphoreGive(impl->sem);

    return AGV_OK;
}

int32_t rad_s_to_regVel(float rad_s, float unit_rpm) {
    float rpm_f = rad_s * 60.0f / (2.0f * M_PI);
    return (int32_t)lroundf(rpm_f / unit_rpm);
}

float regVel_to_rad_s(int32_t reg_val, float unit_rpm) {
    float rpm = (float)reg_val * unit_rpm;
    return rpm * 2.0f * M_PI / 60.0f;
}