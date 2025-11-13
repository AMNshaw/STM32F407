#include <stdio.h>

#include "Agv_communication_pack/communication_builder.h"
#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_core/agv_types.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/modules/host_communication_base.h"
#include "Agv_host_communication/ros_host_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/**
 * private declarations
 */

typedef struct {
    const AgvHostRosCfg* cfg;

    AgvCommLinkIface link;
    AgvCommFormatIface fmt;
    AgvCommProtocolIface prtcl;

    struct {
        TickType_t timestamp;
        Twist2D cmd_vel;
    } cmd_vel_buf;

    SemaphoreHandle_t mutex_buf;

} CommHostRosImpl;

static int Host_communication_ros_destroy(AgvHostCommunicationBase* base);

static int ros_get_des_vel_from_buf(AgvHostCommunicationBase* base,
                                    Twist2D* twist_out);

static int ros_send_odom(AgvHostCommunicationBase* base,
                         const Odometry* odom_in);

static int ros_send_heartbeat(AgvHostCommunicationBase* base);  // TODO

static int ros_process_pending_msg_to_buf(AgvHostCommunicationBase* base);

/**
 * Private definitions
 */

int Host_communication_ros_create(AgvHostCommunicationBase* out,
                                  const AgvHostRosCfg* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;
    CommHostRosImpl* impl = (CommHostRosImpl*)malloc(sizeof(CommHostRosImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;
    impl->cfg = cfg;

    int code;
    printf("Creating uart_ttl link...\n");
    code = Link_uart_ttl_create(&impl->link, &cfg->uart_cfg);
    if (code < 0) {
        impl->link.destroy(&impl->link);
        free(impl);
        return code;
    }
    printf("Creating ros format...\n");
    code = Format_ros_create(&impl->fmt, &cfg->rosFmt_cfg);
    if (code < 0) {
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        free(impl);
        return code;
    }
    printf("Creating host protocol...\n");
    code = Protocol_host_create(&impl->prtcl, &cfg->prtcl_host_cfg);
    if (code < 0) {
        impl->prtcl.destroy(&impl->prtcl);
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        free(impl);
        return code;
    }

    impl->mutex_buf = xSemaphoreCreateMutex();
    if (impl->mutex_buf == NULL) {
        impl->prtcl.destroy(&impl->prtcl);
        impl->fmt.destroy(&impl->fmt);
        impl->link.destroy(&impl->link);
        free(impl);
        return AGV_ERR_MUTEX_FAIL;
    }

    out->impl = impl;
    out->get_des_vel_from_buffer = ros_get_des_vel_from_buf;
    out->process_pending_msg_to_buffer = ros_process_pending_msg_to_buf;
    out->send_odom = ros_send_odom;
    out->send_heartbeat = ros_send_heartbeat;
    out->destroy = Host_communication_ros_destroy;

    printf("Host communication module created\n");
    return AGV_OK;
}

static int Host_communication_ros_destroy(AgvHostCommunicationBase* base) {
    if (!base) return AGV_OK;

    CommHostRosImpl* impl = (CommHostRosImpl*)base->impl;
    if (impl) {
        if (impl->mutex_buf) {
            vSemaphoreDelete(impl->mutex_buf);
            impl->mutex_buf = NULL;
        }
        if (impl->prtcl.destroy) impl->prtcl.destroy(&impl->prtcl);
        if (impl->fmt.destroy) impl->fmt.destroy(&impl->fmt);
        if (impl->link.destroy) impl->link.destroy(&impl->link);
        impl->cfg = NULL;
        free(impl);
    }

    base->impl = NULL;
    base->get_des_vel_from_buffer = NULL;
    base->process_pending_msg_to_buffer = NULL;
    base->send_odom = NULL;
    base->send_heartbeat = NULL;
    base->destroy = NULL;

    return AGV_OK;
}

static int ros_get_des_vel_from_buf(AgvHostCommunicationBase* base,
                                    Twist2D* twist_out) {
    if (!base || !twist_out) return AGV_ERR_INVALID_ARG;
    CommHostRosImpl* impl = (CommHostRosImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    xSemaphoreTake(impl->mutex_buf, portMAX_DELAY);
    twist_out->x = impl->cmd_vel_buf.cmd_vel.x;
    twist_out->y = impl->cmd_vel_buf.cmd_vel.y;
    twist_out->yaw = impl->cmd_vel_buf.cmd_vel.yaw;
    xSemaphoreGive(impl->mutex_buf);
    return AGV_OK;
}

static int ros_send_odom(AgvHostCommunicationBase* base,
                         const Odometry* odom_in) {
    if (!base || !odom_in) return AGV_ERR_INVALID_ARG;
    CommHostRosImpl* impl = (CommHostRosImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    const AgvHostRosCfg* cfg = impl->cfg;
    if (!link || !fmt || !prtcl || !cfg) return AGV_ERR_NO_MEMORY;

    int code = AGV_OK;

    AgvCommMsg msg;
    msg.msg_type = HOST_MSG;
    msg.u.host_msg.type = ODOMETRY;
    msg.u.host_msg.msg.odom.pose.x = odom_in->pose.x;
    msg.u.host_msg.msg.odom.pose.y = odom_in->pose.y;
    msg.u.host_msg.msg.odom.pose.yaw = odom_in->pose.yaw;
    msg.u.host_msg.msg.odom.twist.x = odom_in->twist.x;
    msg.u.host_msg.msg.odom.twist.y = odom_in->twist.y;
    msg.u.host_msg.msg.odom.twist.yaw = odom_in->twist.yaw;

    size_t payload_len = cfg->prtcl_host_cfg.max_payload_len;
    uint8_t payload[payload_len];
    code = prtcl->make_payload(prtcl, &msg, payload, &payload_len);
    if (code != AGV_OK) return code;

    size_t frame_len = cfg->rosFmt_cfg.max_frame_len;
    uint8_t frame[frame_len];
    code = fmt->make_frame(fmt, payload, payload_len, frame, &frame_len);
    if (code != AGV_OK) return code;

    code = link->send_bytes(link, frame, frame_len);

    return code;
}

static int ros_send_heartbeat(AgvHostCommunicationBase* base) { return 0; }

static int ros_process_pending_msg_to_buf(AgvHostCommunicationBase* base) {
    if (!base) return AGV_ERR_INVALID_ARG;
    CommHostRosImpl* impl = (CommHostRosImpl*)base->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    AgvCommLinkIface* link = &impl->link;
    AgvCommFormatIface* fmt = &impl->fmt;
    AgvCommProtocolIface* prtcl = &impl->prtcl;
    const AgvHostRosCfg* cfg = impl->cfg;
    if (!link || !fmt || !prtcl || !cfg) return AGV_ERR_NO_MEMORY;

    int code = AGV_OK;

    printf("Trying to read buf...\n");
    size_t data_len = cfg->uart_cfg.max_data_len;
    uint8_t data[data_len];
    uint32_t timestamp;
    code = link->read_buf(link, data, &data_len, &timestamp);
    if (code != AGV_OK) return code;
    TickType_t now = xTaskGetTickCount();
    if (now - (TickType_t)timestamp > cfg->data_expiration_threshold)
        return 0;  // TODO: def a error, data expired;
    printf("New data bytes_len: %d \n", data_len);

    code = fmt->feed_bytes(fmt, data, data_len);
    if (code != AGV_OK) return code;
    size_t ros_payload_len = cfg->rosFmt_cfg.max_frame_len;
    uint8_t payload[ros_payload_len];
    code = fmt->pop_payload(fmt, payload, &ros_payload_len);
    printf("Payload len: %d \n", ros_payload_len);
    if (code != AGV_OK) return code;

    code = prtcl->feed_payload(prtcl, payload, ros_payload_len);
    if (code != AGV_OK) return code;
    AgvCommMsg msg_popped;
    msg_popped.msg_type = HOST_MSG;
    code = prtcl->pop_msg(prtcl, &msg_popped);
    if (code != AGV_OK) return code;

    switch (msg_popped.u.host_msg.type) {
        case VEL_CMD: {
            xSemaphoreTake(impl->mutex_buf, portMAX_DELAY);
            impl->cmd_vel_buf.cmd_vel.x = msg_popped.u.host_msg.msg.vel.x;
            impl->cmd_vel_buf.cmd_vel.y = msg_popped.u.host_msg.msg.vel.y;
            impl->cmd_vel_buf.cmd_vel.yaw = msg_popped.u.host_msg.msg.vel.yaw;
            impl->cmd_vel_buf.timestamp = (TickType_t)timestamp;
            xSemaphoreGive(impl->mutex_buf);
            printf("cmd_vel msg: %f %f %f\n", impl->cmd_vel_buf.cmd_vel.x,
                   impl->cmd_vel_buf.cmd_vel.y, impl->cmd_vel_buf.cmd_vel.yaw);
            return AGV_OK;
        }
        default:
            return AGV_ERR_UNKNOWN;
    }

    return AGV_OK;
}