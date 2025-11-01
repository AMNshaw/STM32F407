#ifndef AGV_CORE__COMMUNICATION_BASE_H_
#define AGV_CORE__COMMUNICATION_BASE_H_

#include "types.h"

typedef struct AgvCommunicationBase {
    int (*recv_cmd)(struct AgvCommunicationBase* self,
                    VelCmd* cmd_out);  // 沒新資料回 0
    int (*publish_odom)(struct AgvCommunicationBase* self, const Odom* odom);
    void* impl;
} AgvCommunicationBase;

typedef struct {
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* de_port;
    uint16_t de_pin;  //
    enum { HOST_FMT_CSV, HOST_FMT_MAVLINK } fmt;
    enum { PHY_UART_TTL, PHY_UART_RS485 } phy;
} AgvCommConfig;

// void UART_link_create(AgvCommunicationBase* out /*, UART handle, queues ...
// */);

void Link_create(AgvCommunicationBase* out, const AgvCommConfig* cfg);

#endif  // AGV_CORE__COMMUNICATION_BASE_H_