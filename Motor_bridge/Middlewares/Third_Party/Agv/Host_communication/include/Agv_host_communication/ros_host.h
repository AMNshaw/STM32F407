#ifndef AGV_HOST_COMMUNICATION__ROS_HOST_H_
#define AGV_HOST_COMMUNICATION__ROS_HOST_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_core/agv_types.h"
#include "Agv_core/modules/host_communication_base.h"
#include "Agv_host_communication/ros_host_config.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

typedef struct {
    const AgvHostRosCfg* cfg;

    AgvCommLinkIface link;
    AgvCommFormatIface fmt;
    AgvCommProtocolIface prtcl;

    struct {
        TickType_t timestamp;
        VelCmd cmd_vel;
    } cmd_vel_buf;

    SemaphoreHandle_t mutex_buf;

} CommHostRosImpl;

static int Host_communication_ros_destroy(AgvHostCommunicationBase* base);

static int ros_get_des_vel_from_buf(AgvHostCommunicationBase* base,
                                    VelCmd* out);

static int ros_send_odom(AgvHostCommunicationBase* base, const Odom* in);

static int ros_send_heartbeat(AgvHostCommunicationBase* base);  // TODO

static int ros_process_pending_msg_to_buf(AgvHostCommunicationBase* base);

#endif  // AGV_HOST_COMMUNICATION__ROS_HOST_H_