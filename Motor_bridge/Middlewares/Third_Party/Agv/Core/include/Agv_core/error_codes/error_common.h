#ifndef AGV_CORE__ERROR_CODES_COMMON_H_
#define AGV_CORE__ERROR_CODES_COMMON_H_

#define AGV_OK 0  ///< 成功

#define AGV_ERR_INVALID_ARG -1      ///< 參數無效 (NULL、長度為 0...等)
#define AGV_ERR_NO_MEMORY -2        ///< malloc / new 失敗
#define AGV_ERR_MUTEX_FAIL -3       ///< Mutex 建立或操作失敗
#define AGV_ERR_OUTPUT_OVERFLOW -4  ///<
#define AGV_ERR_UNKNOWN -9          ///<

#endif  // AGV_CORE__ERROR_CODES_COMMON_H_