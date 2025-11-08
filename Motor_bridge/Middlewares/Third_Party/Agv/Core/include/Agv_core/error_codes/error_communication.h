#ifndef AGV_COMMUNICATION_PACK__EXCEPTION_CODES_H_
#define AGV_COMMUNICATION_PACK__EXCEPTION_CODES_H_

/* ===== Link 層錯誤 (UART / RS485 / I2C / SPI...) ===== */
#define AGV_ERR_COMM_LINK_HAL \
    -10  ///< HAL_* API 回傳錯誤 (HAL_ERROR/HAL_TIMEOUT...)
#define AGV_ERR_COMM_LINK_TIMEOUT -11  ///< 傳輸逾時
#define AGV_ERR_COMM_LINK_BUFFER_OVERFLOW \
    -12                                 ///< 收到資料超過 buffer 可容納大小
#define AGV_ERR_COMM_LINK_RX_EMPTY -13  ///< 沒有收到任何資料

/* ===== Format 層錯誤 (frame parser) ===== */
#define AGV_ERR_COMM_FMT_FRAME_TOO_SHORT \
    -20                                      ///< frame 長度太短，不足基本頭/CRC
#define AGV_ERR_COMM_FMT_FRAME_TOO_LONG -21  ///< frame 長度超過允許
#define AGV_ERR_COMM_FMT_BAD_CRC -22         ///< CRC 檢查失敗
#define AGV_ERR_COMM_FMT_BAD_HEADER -23      ///< header 不符合期望
#define AGV_ERR_COMM_FMT_BAD_TAIL -24        ///< tail 不符合期望
#define AGV_ERR_COMM_FMT_NO_COMPLETE_FRAME -25  ///< buffer 裡沒有完整 frame

/* ===== Protocol 層錯誤 (command 解譯/邏輯) ===== */
#define AGV_ERR_COMM_PRTCL_UNSUPPORTED_CMD \
    -30  ///< 不支援的 command / function code
#define AGV_ERR_COMM_PRTCL_UNSUPPORTED_SHARED_ID -31  ///< shared_id 不符
#define AGV_ERR_COMM_PRTCL_BAD_PAYLOAD -32  ///< payload 格式不符或長度不對
#define AGV_ERR_COMM_PRTCL_STATE -33        ///< 狀態機不在合法狀態
#define AGV_ERR_COMM_PRTCL_NO_PENDING_MSG -34
#define AGV_ERR_COMM_PRTCL_INVALID_MSG_TYPE -35
#define AGV_ERR_COMM_PRTCL_EXCEPTION -36

#endif  // AGV_COMMUNICATION_PACK__EXCEPTION_CODES_H_
