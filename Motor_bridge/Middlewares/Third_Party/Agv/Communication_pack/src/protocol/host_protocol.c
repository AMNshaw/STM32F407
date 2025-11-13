
#include <stdlib.h>
#include <string.h>

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/communication_msgs.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"
#include "Agv_communication_pack/protocol_defs/host_protocol_defs.h"
#include "Agv_core/error_codes/error_common.h"
#include "Agv_core/error_codes/error_communication.h"

/**
 * private declarations
 */

typedef struct {
    const AgvCommPrtclHostCfg* cfg;

    AgvCommMsg pending_msg;
    int has_pending;
} HostProtoImpl;

static int hostProto_feed_payload(AgvCommProtocolIface* iface,
                                  const uint8_t* payload_in,
                                  size_t payload_len);

static int hostProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* msg_out);

static int hostProto_make_payload(AgvCommProtocolIface* iface,
                                  const AgvCommMsg* msg_in,
                                  uint8_t* payload_out, size_t* payload_len);

static int hostProto_destroy(AgvCommProtocolIface* iface);

uint8_t* put_f32_le(uint8_t* p, float v);

const uint8_t* get_f32_le(const uint8_t* p, float* out);

/**
 * Private definitions
 */

int Protocol_host_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclHostCfg* cfg) {
    if (!out || !cfg) return AGV_ERR_INVALID_ARG;
    HostProtoImpl* impl = (HostProtoImpl*)malloc(sizeof(HostProtoImpl));
    if (!impl) return AGV_ERR_NO_MEMORY;

    impl->cfg = cfg;

    out->impl = impl;
    out->feed_payload = hostProto_feed_payload;
    out->pop_msg = hostProto_pop_msg;
    out->make_payload = hostProto_make_payload;
    out->destroy = hostProto_destroy;
    return AGV_OK;
}

static int hostProto_destroy(AgvCommProtocolIface* iface) {
    if (!iface) return AGV_OK;
    HostProtoImpl* impl = (HostProtoImpl*)iface->impl;
    if (impl) {
        impl->cfg = NULL;
        free(impl);
    };

    iface->impl = NULL;
    iface->feed_payload = NULL;
    iface->pop_msg = NULL;
    iface->make_payload = NULL;
    iface->destroy = NULL;

    return AGV_OK;
}

static int hostProto_feed_payload(AgvCommProtocolIface* iface,
                                  const uint8_t* payload_in,
                                  size_t payload_len) {
    if (!iface || !payload_in || payload_len == 0) return AGV_ERR_INVALID_ARG;
    HostProtoImpl* impl = (HostProtoImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;
    if (payload_len <= 2) return AGV_ERR_COMM_PRTCL_BAD_PAYLOAD;

    size_t idx = 0;
    uint8_t cmd = payload_in[idx++];
    uint8_t len = payload_in[idx++];

    if (2 + len != payload_len) {
        return AGV_ERR_COMM_PRTCL_BAD_PAYLOAD;
    }

    AgvCommMsg msg;
    msg.msg_type = HOST_MSG;
    msg.u.host_msg.type = VEL_CMD;

    switch (cmd) {
        case HOST_COMM_CMD_SET_VEL: {
            size_t expected_len = 3 * sizeof(float);  // float * (vx, vy, vyaw)
            if (len != expected_len) return AGV_ERR_COMM_PRTCL_BAD_PAYLOAD;
            uint8_t* p = &payload_in[idx];

            p = get_f32_le(p, &msg.u.host_msg.msg.vel.x);
            p = get_f32_le(p, &msg.u.host_msg.msg.vel.y);
            p = get_f32_le(p, &msg.u.host_msg.msg.vel.yaw);

            impl->pending_msg = msg;
            impl->has_pending = 1;

            return AGV_OK;
        }
        default:
            return AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE;
    }

    return AGV_OK;
}

static int hostProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* msg_out) {
    if (!iface || !msg_out) return AGV_ERR_INVALID_ARG;

    HostProtoImpl* impl = (HostProtoImpl*)iface->impl;
    if (!impl) return AGV_ERR_NO_MEMORY;

    if (!impl->has_pending) return AGV_ERR_COMM_PRTCL_NO_PENDING_MSG;

    *msg_out = impl->pending_msg;
    impl->has_pending = 0;

    return 0;
}

static int hostProto_make_payload(AgvCommProtocolIface* iface,
                                  const AgvCommMsg* msg_in,
                                  uint8_t* payload_out, size_t* payload_len) {
    if (!msg_in || !payload_out || !payload_len) return AGV_ERR_INVALID_ARG;

    if (msg_in->msg_type != HOST_MSG)
        return AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE;
    HostCommMsg* host_msg = &msg_in->u.host_msg;

    uint8_t cmd = 0;
    uint8_t data_len = 0;

    size_t idx = 0;
    switch (host_msg->type) {
        case ODOMETRY: {
            cmd = HOST_COMM_CMD_ODOMETRY_FEEDBACK;
            data_len = 6 * sizeof(float);  // cmd + len + float * (x, y, yaw,
                                           // vx, vy, vyaw)
            if (*payload_len < data_len) return AGV_ERR_OUTPUT_OVERFLOW;

            payload_out[idx++] = cmd;
            payload_out[idx++] = data_len;
            uint8_t* p = &payload_out[idx];

            p = put_f32_le(p, host_msg->msg.odom.pose.x);
            p = put_f32_le(p, host_msg->msg.odom.pose.y);
            p = put_f32_le(p, host_msg->msg.odom.pose.yaw);
            p = put_f32_le(p, host_msg->msg.odom.twist.x);
            p = put_f32_le(p, host_msg->msg.odom.twist.y);
            p = put_f32_le(p, host_msg->msg.odom.twist.yaw);

            idx = (size_t)(p - payload_out);
            *payload_len = idx;

            return AGV_OK;
        }
        case HOST_COMM_CMD_HEARTBEAT: {
            return AGV_OK;
        }
        default:
            return AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE;
    }
    return AGV_OK;
}

uint8_t* put_f32_le(uint8_t* p, float v) {
    uint32_t u;
    memcpy(&u, &v, 4);  // float -> uint32_t (bit pattern 不變)

    *p++ = (uint8_t)(u & 0xFF);
    *p++ = (uint8_t)((u >> 8) & 0xFF);
    *p++ = (uint8_t)((u >> 16) & 0xFF);
    *p++ = (uint8_t)((u >> 24) & 0xFF);
    return p;
}

const uint8_t* get_f32_le(const uint8_t* p, float* out) {
    uint32_t u = 0;
    u |= (uint32_t)(*p++);
    u |= (uint32_t)(*p++) << 8;
    u |= (uint32_t)(*p++) << 16;
    u |= (uint32_t)(*p++) << 24;

    memcpy(out, &u, 4);  // uint32_t -> float
    return p;
}