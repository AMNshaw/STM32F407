#ifndef AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_DEFS_H_
#define AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_DEFS_H_

#include <stdint.h>

// On-wire frame layout (Modbus RTU):
// [SlaveId][FunctionCode][PduData...][CrcLo][CrcHi]

// ---------------------------------------------------------------------------
// Function Codes
// ---------------------------------------------------------------------------

// 讀取類
#define MODBUS_FC_READ_COILS 0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS 0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS 0x03  // 你現在在用的 0x03
#define MODBUS_FC_READ_INPUT_REGISTERS 0x04

// 寫入類
#define MODBUS_FC_WRITE_SINGLE_COIL 0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER 0x06
#define MODBUS_FC_WRITE_MULTIPLE_COILS 0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10  // 你現在在用的 0x10

// 讀寫複合
#define MODBUS_FC_READWRITE_MULTIPLE_REGISTERS 0x17  // 你提到的 0x17

// ---------------------------------------------------------------------------
// Exception / Error
// ---------------------------------------------------------------------------

// 若回應的 FunctionCode = RequestFunctionCode | 0x80 → 表示 exception
#define MODBUS_FC_EXCEPTION_BIT 0x80

// Exception Codes (PDU 第 3 個 byte)
#define MODBUS_EX_ILLEGAL_FUNCTION 0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS 0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE 0x03
#define MODBUS_EX_SLAVE_DEVICE_FAILURE 0x04
#define MODBUS_EX_ACKNOWLEDGE 0x05
#define MODBUS_EX_SLAVE_DEVICE_BUSY 0x06
#define MODBUS_EX_MEMORY_PARITY_ERROR 0x08
#define MODBUS_EX_GATEWAY_PATH_UNAVAILABLE 0x0A
#define MODBUS_EX_GATEWAY_TARGET_FAILED 0x0B

// ---------------------------------------------------------------------------
// CRC 設定 (給 format/modbus_rtu_format 用)
// ---------------------------------------------------------------------------

#define MODBUS_RTU_CRC_INIT ((uint16_t)0xFFFFu)
#define MODBUS_RTU_CRC_POLY ((uint16_t)0xA001u)

#endif  // AGV_COMMUNICATION_PACK__BLVR_PROTOCOL_DEFS_H_
