#include "protocol/host_protocol.h"

int Protocol_host_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclHostCfg* cfg) {
    if (!out || !cfg) return -1;
    HostProtoImpl* impl = (HostProtoImpl*)malloc(sizeof(HostProtoImpl));
    if (!impl) return -2;
    memset(impl, 0, sizeof(*impl));

    impl->cfg = cfg;

    out->impl = impl;
    out->feed_frame = hostProto_feed_frame;
    out->pop_msg = hostProto_pop_msg;
    out->build_frame = hostProto_build_frame;
    out->destroy = [](AgvCommProtocolIface* iface) {
        if (!iface || !iface->impl) return 0;
        free(iface->impl);
        iface->impl = NULL;
        return 0;
    };
    return 0;
}

static int hostProto_feed_frame(AgvCommProtocolIface* iface,
                                const uint8_t* frame, size_t len) {
    HostProtoImpl* impl = (HostProtoImpl*)iface->impl;
    if (!impl || !frame || len < 2) return -1;

    uint8_t cmd = frame[0];
    uint8_t size = frame[1];
    const uint8_t* data = &frame[2];

    if (2 + size != len) {
        return -2;
    }

    AgvCommMsg msg;
    memset(&msg, 0, sizeof(msg));

    switch (cmd) {
        case CMD_SET_VEL: {
            if (size != 2 * sizeof(float)) return -3;
            float v, w;
            memcpy(&v, &data[0], sizeof(float));
            memcpy(&w, &data[sizeof(float)], sizeof(float));
            msg.type = AGV_COMM_MSG_CMD_VEL;
            msg.data.cmd_vel.linear = v;
            msg.data.cmd_vel.angular = w;
            break;
        }
        case CMD_FEEDBACK: {
            if (size != 3 * sizeof(float)) return -3;
            float x, y, theta;
            memcpy(&x, &data[0], sizeof(float));
            memcpy(&y, &data[sizeof(float)], sizeof(float));
            memcpy(&theta, &data[2 * sizeof(float)], sizeof(float));
            msg.type = AGV_COMM_MSG_ODOM;
            msg.data.odom.x = x;
            msg.data.odom.y = y;
            msg.data.odom.theta = theta;
            break;
        }
        default:
            // 不認識的 command：直接忽略
            return 0;
    }

    impl->last_msg = msg;
    impl->has_msg = 1;
    return 0;
}

static int hostProto_pop_msg(AgvCommProtocolIface* iface, AgvCommMsg* out) {
    HostProtoImpl* impl = (HostProtoImpl*)iface->impl;
    if (!impl || !out) return -1;
    if (!impl->has_msg) return -2;

    *out = impl->last_msg;
    impl->has_msg = 0;
    return 0;
}

static int hostProto_build_frame(AgvCommProtocolIface* iface,
                                 const AgvCommMsg* msg, uint8_t* out_frame,
                                 size_t* inout_len) {
    (void)iface;
    if (!msg || !out_frame || !inout_len) return -1;

    uint8_t cmd;
    uint8_t size;
    uint8_t data[64];

    switch (msg->type) {
        case AGV_COMM_MSG_ODOM: {
            cmd = CMD_FEEDBACK;
            size = 3 * sizeof(float);
            if (*inout_len < (size_t)(2 + size)) return -2;

            float x = msg->data.odom.x;
            float y = msg->data.odom.y;
            float th = msg->data.odom.theta;
            memcpy(&data[0], &x, sizeof(float));
            memcpy(&data[sizeof(float)], &y, sizeof(float));
            memcpy(&data[2 * sizeof(float)], &th, sizeof(float));
            break;
        }
        default:
            return -3;  // host 目前只需要 encode ODOM
    }

    size_t idx = 0;
    out_frame[idx++] = cmd;
    out_frame[idx++] = size;
    memcpy(&out_frame[idx], data, size);
    idx += size;

    *inout_len = idx;
    return 0;
}
