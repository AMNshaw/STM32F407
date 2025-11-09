#include "Agv_motor_communication/blvr.h"

#include "Agv_communication_pack/communication_builder.h"
#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_core/error_codes/error_common.h"

#ifndef M_PI
#define M_PI (float)3.1415926535897932384626433832
#endif

int Motor_comm_blvr_create(AgvMotorCommunicationBase* out,
                           const Agv_Blvr_config* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;

    MotorCommBlvrImpl* impl =
        (MotorCommBlvrImpl*)malloc(sizeof(MotorCommBlvrImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;
    impl->buffer = (BlvrBuff*)malloc(cfg->axis_count * sizeof(BlvrBuff));

    impl->sem = xSemaphoreCreateMutex();
    if (impl->sem == NULL) {
        Free(impl->buffer);
        Free(impl);
        return AGV_ERR_MUTEX_FAIL;
    }

    int code;

    code = Link_uart_rs485_create(&impl->link, &cfg->uart_cfg);
    if (code != 0) {
        impl->link.destroy(&impl->link);
        Free(impl->buffer);
        Free(impl);
        return code;
    }
    code = Format_modbus_create(&impl->fmt, &cfg->modbus_cfg);
    if (code != 0) {
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        Free(impl->buffer);
        Free(impl);
        return code;
    }
    code = Protocol_blvr_create(&impl->prtcl, &cfg->prtcl_blvr_cfg);
    if (code != 0) {
        impl->prtcl.destroy(&impl->prtcl);
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        Free(impl->buffer);
        Free(impl);
        return code;
    }

    out->impl = &impl;
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

    xSemaphoreTake(impl->sem, portMAX_DELAY);
    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        impl->buffer[i].des_rpm = rad_s_to_rpm(&in->w[i]);
    }
    xSemaphoreGive(impl->sem);

    return AGV_OK;
}

static int blvr_get_curr_vel(AgvMotorCommunicationBase* base, WheelsVel* out) {
    if (!base || !out) return AGV_ERR_INVALID_ARG;
    MotorCommBlvrImpl* impl = (MotorCommBlvrImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    xSemaphoreTake(impl->sem, portMAX_DELAY);
    for (size_t i = 0; i < impl->cfg->axis_count; ++i) {
        out->w[i] = rpm_to_rad_s(&impl->buffer[i].des_rpm);
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
        msg_send.u.motors_msg.msgs[i].des_rpm = impl->buffer[i].des_rpm;
        msg_send.u.motors_msg.msgs[i].des_acc = impl->buffer[i].des_acc;
        msg_send.u.motors_msg.msgs[i].des_dec = impl->buffer[i].des_dec;
        msg_send.u.motors_msg.msgs[i].spd_ctrl =
            impl->cfg->prtcl_blvr_cfg.operation_type;
        msg_send.u.motors_msg.msgs[i].trigger =
            impl->cfg->prtcl_blvr_cfg.operation_trigger;
    }
    xSemaphoreGive(impl->sem);

    size_t blvr_frame_size = impl->cfg->prtcl_blvr_cfg.max_frame_size;
    uint8_t frame_built[blvr_frame_size];
    prtcl->build_frame(prtcl, &msg_send, frame_built, &blvr_frame_size);
    fmt->feed_frame(fmt, frame_built, blvr_frame_size);
    size_t data_send_size = impl->cfg->uart_cfg.max_data_size;
    uint8_t data_send[data_send_size];
    fmt->pop_frame(fmt, data_send, &data_send_size);
    link->send_bytes(link, data_send, data_send_size);

    // Receive read write request
    size_t data_rcv_size = impl->cfg->modbus_cfg.max_buf_size;
    uint8_t data_rcv[data_rcv_size];
    link->recv_bytes(link, data_rcv, data_rcv_size);

    size_t modbus_frame_size = impl->cfg->modbus_cfg.max_frame_size;
    uint8_t frame_popped[modbus_frame_size];
    fmt->feed_frame(fmt, data_rcv, data_rcv_size);
    fmt->pop_frame(fmt, frame_popped, &modbus_frame_size);
    prtcl->feed_frame(prtcl, frame_popped, modbus_frame_size);

    AgvCommMsg msg_rcv;
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

int32_t rad_s_to_rpm(float* rad_s) {
    return (int32_t)lroundf((*rad_s) * 60.0f / (2.0f * M_PI));
}

float rpm_to_rad_s(int32_t* rpm) { return (float)(*rpm) * 2.0f * M_PI / 60.0f; }