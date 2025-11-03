#include "communication_motor.h"

int Communication_motor_create(AgvCommunicationBase* out,
                               const AgvUartCfg* uart_cfg,
                               const AgvModbusRtuFmtCfg* modbus_cfg,
                               const AgvBlvrProtoCfg* blvr_prtcl_cfg) {
    static Motor_comm_impl impl;

    int code;
    code = Link_uart_rs485_create(&impl.link_, uart_cfg);
    code = Format_modbus_create(&impl.fmt_, modbus_cfg);
    code = Protocol_blvr_create(&impl.prtcl_, blvr_prtcl_cfg);

    out->impl = &impl;
    out->receive = receive_encoder;
    out->send = send_ctrl;
}

static void receive_encoder(AgvCommunicationBase* self) {}

static void send_ctrl(AgvCommunicationBase* self) {}