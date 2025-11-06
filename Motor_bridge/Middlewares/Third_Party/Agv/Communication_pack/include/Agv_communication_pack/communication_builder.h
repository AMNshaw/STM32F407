#ifndef AGV_COMMUNICATION_PACK__COMMUNICATION_BUILDER_H_
#define AGV_COMMUNICATION_PACK__COMMUNICATION_BUILDER_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/configs/comm_format_config.h"
#include "Agv_communication_pack/configs/comm_link_config.h"
#include "Agv_communication_pack/configs/comm_protocol_config.h"

int Link_uart_ttl_create(AgvCommLinkIface* out, const AgvCommUartCfg* cfg);
int Link_uart_rs485_create(AgvCommLinkIface* out, const AgvCommUartCfg* cfg);

int Format_ros_create(AgvCommFormatIface* out, const AgvCommFmtRosCfg* cfg);
int Format_modbus_create(AgvCommFormatIface* out,
                         const AgvCommFmtModbusRtuCfg* cfg);

int Protocol_host_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclHostCfg* cfg);
int Protocol_blvr_create(AgvCommProtocolIface* out,
                         const AgvCommPrtclBlvrCfg* cfg);

#endif  // AGV_COMMUNICATION_PACK__COMMUNICATION_BUILDER_H_