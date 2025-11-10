#ifndef AGV_COMMUNICATION_PACK__HOST_PROTOCOL_DEFS_H_
#define AGV_COMMUNICATION_PACK__HOST_PROTOCOL_DEFS_H_

// On-wire frame layout (ROS):
// [Header0][Header1][Command][Size][Data...][Crc][Tail0][Tail1]

#define HOST_COMM_HEADER0 0x55
#define HOST_COMM_HEADER1 0xAA
#define HOST_COMM_TAIL0 0x0D
#define HOST_COMM_TAIL1 0x0A

#define HOST_COMM_CMD_SET_VEL 0x01
#define HOST_COMM_CMD_FEEDBACK 0x02

// CRC-8 settings (這些不會出現在封包裡，但定義「怎麼算 Crc」)
#define ROSFMT_CRC_INIT 0x00  // 初始值
#define ROSFMT_CRC_POLY 0x07  // x^8 + x^2 + x + 1（去掉最高位）

#endif  // AGV_COMMUNICATION_PACK__HOST_PROTOCOL_DEFS_H_
