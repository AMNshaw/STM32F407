#ifndef AGV_FACTORY__COMMUNICATION_BUILDER_H_
#define AGV_FACTORY__COMMUNICATION_BUILDER_H_

#include "Agv_core/modules/host_communication_base.h"
#include "Agv_host_communication/ros_host_config.h"

int Host_communication_ros_create(AgvHostCommunicationBase* out,
                                  const AgvHostRosCfg* cfg);

#endif  // AGV_FACTORY__COMMUNICATION_BUILDER_H_