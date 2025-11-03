#ifndef AGV_CORE__COMMUNICATION_IFACE_H_
#define AGV_CORE__COMMUNICATION_IFACE_H_

#include <stddef.h>
#include <stdint.h>

#include "communication_config.h"

/*
 * Link
 **/

typedef struct AgvCommLinkIface {
    int (*send_bytes)(struct AgvCommLinkIface*, const uint8_t*, size_t);
    int (*recv_bytes)(struct AgvCommLinkIface*, uint8_t*, size_t);
    int (*destroy)(struct AgvCommLinkIface*);
    void* impl;
} AgvCommLinkIface;

int Link_uart_ttl_create(AgvCommLinkIface* out, const AgvUartCfg* cfg);
int Link_uart_rs485_create(AgvCommLinkIface* out, const AgvUartCfg* cfg);

/*
 * Physical
 **/

// typedef struct AgvCommPhyIface {
//     void* impl;
// } AgvCommPhyIface;

// int Phy_ttl_create(AgvCommPhyIface* out, const AgvPhyTtlCfg* cfg);
// int Phy_rs485_create(AgvCommPhyIface* out, const AgvPhyRs485Cfg* cfg);

/*
 * Format
 **/

typedef struct AgvCommFormatIface {
    int (*feed)(struct AgvCommFormatIface*, const uint8_t* bytes, size_t n);
    int (*pop_frame)(struct AgvCommFormatIface*, uint8_t* out,
                     size_t* inout_len);
    int (*make_frame)(struct AgvCommFormatIface*, const uint8_t* payload,
                      size_t len, uint8_t* out, size_t* inout_len);
    int (*destroy)(struct AgvCommFormatIface*);
    void* impl;
} AgvCommFormatIface;

int Format_csv_create(AgvCommFormatIface* out, const AgvCsvFmtCfg* cfg);
int Format_modbus_create(AgvCommFormatIface* out,
                         const AgvModbusRtuFmtCfg* cfg);

/*
 * Portocol
 **/

typedef struct AgvCommProtocolIface {
    int (*parse)(struct AgvCommProtocolIface*);
    int (*format)(struct AgvCommProtocolIface*);
    int (*destroy)(struct AgvCommProtocolIface*);
    void* impl;
} AgvCommProtocolIface;

int Protocol_host_csv_create(AgvCommProtocolIface* out,
                             const AgvHostCsvProtoCfg* cfg);
int Protocol_blvr_create(AgvCommProtocolIface* out, const AgvBlvrProtoCfg* cfg);

#endif  // AGV_CORE__COMMUNICATION_IFACE_H_