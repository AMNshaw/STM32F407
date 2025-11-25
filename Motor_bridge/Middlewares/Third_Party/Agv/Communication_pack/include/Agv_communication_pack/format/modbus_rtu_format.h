#ifndef AGV_COMMUNICATION_PACK__MODBUS_RTU_FORMAT_H_
#define AGV_COMMUNICATION_PACK__MODBUS_RTU_FORMAT_H_

#include "Agv_communication_pack/communication_iface.h"
#include "Agv_communication_pack/communication_msgs.h"

int modbusFmt_set_check_item(AgvCommFormatIface* iface, uint8_t addr,
                             uint8_t func, size_t expected_bytes);

#endif  // AGV_COMMUNICATION_PACK__MODBUS_RTU_FORMAT_H_