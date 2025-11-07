#ifndef AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_DEFS_H_
#define AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_DEFS_H_

#include <stdint.h>

// On-wire frame layout (Modbus RTU):
// [SlaveId][FunctionCode][PduData...][CrcLo][CrcHi]

/****************************************************
 * Operation types
 ****************************************************/
#define BLVR_OPERATION_TYPE_CONTINUOUS_SPD_CTRL_NORMAL 0x10
#define BLVR_OPERATION_TYPE_CONTINUOUS_SPD_CTRL_MOTION_EXT 0x30

/****************************************************
 * Operation Trigger
 ****************************************************/

#define BLVR_OPERATION_TRIGGER_DISABLE 0x00
#define BLVR_OPERATION_TRIGGER_NORMAL_START 0x01

/****************************************************
 * Shared operations
 ****************************************************/

#define BLVR_SHARED_ID 0x0F

#define BLVR_NUM_RGSTR_PER_CMD 2
#define BLVR_RGSTR_BYTE 2
/**
 * Read
 */

// Function code
#define BLVR_FC_READ_HOLDING_REGISTERS 0x03  // 你現在在用的 0x03

// Data
#define BLVR_DATA_WRITE_CMD_COUNT 4

#define BLVR_DATA_REG_ADDRESS_DRIVER_OUT_ST 10
#define BLVR_DATA_REG_ADDRESS_REAL_POS 12
#define BLVR_DATA_REG_ADDRESS_REAL_VEL 14
#define BLVR_DATA_REG_ADDRESS_PRESENT_ALRM 16

// Data type

typedef uint32_t value_type;

/**
 * Write
 */

// Function code
#define BLVR_FC_WRITE_MULTIPLE_REGISTERS 0x10  // 你現在在用的 0x10

// Data
#define BLVR_DATA_WRITE_CMD_COUNT 5

#define BLVR_DATA_REG_ADDRESS_CMD_VEL 0
#define BLVR_DATA_REG_ADDRESS_CMD_ACC 2
#define BLVR_DATA_REG_ADDRESS_CMD_DEC 4
#define BLVR_DATA_REG_ADDRESS_CMD_OP_TYPE 6
#define BLVR_DATA_REG_ADDRESS_CMD_TRIG 8

/**
 * Read & Write
 */

// Function code
#define BLVR_FC_READWRITE_MULTIPLE_REGISTERS 0x17  // 你提到的 0x17

// Data
/****************************************************
 * Exceptions / Error
 ****************************************************/

// 若回應的 FunctionCode = RequestFunctionCode | 0x80 → 表示 exception
#define BLVR_FC_EXCEPTION_BIT 0x80

// Exception Codes (PDU 第 3 個 byte)
#define BLVR_EX_ILLEGAL_FUNCTION 0x01
#define BLVR_EX_ILLEGAL_DATA_ADDRESS 0x02
#define BLVR_EX_ILLEGAL_DATA_VALUE 0x03
#define BLVR_EX_SLAVE_DEVICE_FAILURE 0x04
#define BLVR_EX_ACKNOWLEDGE 0x05
#define BLVR_EX_SLAVE_DEVICE_BUSY 0x06
#define BLVR_EX_MEMORY_PARITY_ERROR 0x08
#define BLVR_EX_GATEWAY_PATH_UNAVAILABLE 0x0A
#define BLVR_EX_GATEWAY_TARGET_FAILED 0x0B

// ---------------------------------------------------------------------------
// CRC 設定 (給 format/modbus_rtu_format 用)
// ---------------------------------------------------------------------------

#define MODBUS_RTU_CRC_INIT ((uint16_t)0xFFFFu)
#define MODBUS_RTU_CRC_POLY ((uint16_t)0xA001u)

#endif  // AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_DEFS_H_
