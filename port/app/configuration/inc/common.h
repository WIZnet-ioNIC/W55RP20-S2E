#ifndef _COMMON_H
#define _COMMON_H

#include <stdint.h>

//////////////////////////////////
// Product Version              //
//////////////////////////////////
/* Application Firmware Version */
#define MAJOR_VER               1
#define MINOR_VER               2
#define MAINTENANCE_VER         3

#define DEV_CONFIG_VER          104

//#define STR_VERSION_STATUS      "Develop" // or "Stable"
#define STR_VERSION_STATUS      "Stable"

//////////////////////////////////
// W5500 HW Socket Definition  //
//////////////////////////////////
// 0 = PPPoE, DATA0~3 = 1~4, config UDP/TCP = 5/6, DHCP/DNS/FW update = 7.
// MACRAW, which the PPPoE negotiation runs over, exists only on socket 0, so socket 0
// is left to it rather than time-shared with a data channel that would then drop every
// time the line reconnects. Everything else moved up one.
//
// Original: DATA0~3 = 0~3, config UDP/TCP = 4/5, DHCP/DNS = 6, socket 7 free.
//
// HTTP macros retained only so httpHandler.c compiles; the task is not created.
#define SOCK_MAX_USED           8

#define SOCK_PPPOE              0
#define SOCK_DATA0              1
#define SOCK_DATA1              2
#define SOCK_DATA2              3
#define SOCK_DATA3              4
#define SOCK_CONFIG_UDP         5
#define SOCK_CONFIG_TCP         6
#define SOCK_DHCP               7
#define SOCK_DNS                7
#define SOCK_FWUPDATE           7
#define SOCK_NETBIOS            7
#define SOCK_NTP                7

#define MAX_HTTPSOCK	3
#define SOCK_HTTPSERVER_1       SOCK_DHCP
#define SOCK_HTTPSERVER_2       SOCK_DHCP
#define SOCK_HTTPSERVER_3       SOCK_DHCP

#define SEG_DATA0_SOCK          SOCK_DATA0
#define SEG_DATA1_SOCK          SOCK_DATA1
#define SEG_DATA2_SOCK          SOCK_DATA2
#define SEG_DATA3_SOCK          SOCK_DATA3
#define SEGCP_UDP_SOCK          SOCK_CONFIG_UDP
#define SEGCP_TCP_SOCK          SOCK_CONFIG_TCP

//////////////////////////////////
// Ethernet                     //
//////////////////////////////////
/* Buffer size */
#define DATA_BUF_SIZE           2048
#define CONFIG_BUF_SIZE         2048 //512
#define MQTT_BUF_SIZE           2048

#define ROOTCA_BUF_SIZE         2048
#define CLICA_BUF_SIZE          2048
#define PKEY_BUF_SIZE           2048

//////////////////////////////////
// Available board list         //
//////////////////////////////////
//#define WIZ2000_MB              1
#define S2E_SSL                   1
#define WIZ5XXSR-RP               2
#define UNKNOWN_DEVICE          0xff

//////////////////////////////////
//        Clock Setting         //
//////////////////////////////////

#define PLL_SYS_KHZ             (200000UL)

//////////////////////////////////
// Defines                      //
//////////////////////////////////
// Defines for S2E Status
typedef enum {ST_BOOT, ST_OPEN, ST_CONNECT, ST_UPGRADE, ST_ATMODE, ST_UDP} teDEVSTATUS;  // for Device status

#define DEVICE_APPBOOT_MODE     ST_BOOT
#define DEVICE_APP_MODE         ST_OPEN

// Gateway / command mode
#define DEVICE_AT_MODE          0
#define DEVICE_GW_MODE          1

// Network operation mode
#define TCP_CLIENT_MODE         0
#define TCP_SERVER_MODE         1
#define TCP_MIXED_MODE          2
#define UDP_MODE                3
#define SSL_TCP_CLIENT_MODE  4
#define MQTT_CLIENT_MODE  5
#define MQTTS_CLIENT_MODE  6

#define MQTT_TIMEOUT_MS                 400     // unit: ms

//#define MODBUS_TCP_CLIENT_MODE  4   // TCP client (Master)
//#define MODBUS_TCP_SERVER_MODE  5   // TCP server (Slave)

// How the device gets its IP. The numbering is the WIZ145SR W_MD command's.
#define IP_MODE_STATIC          0
#define IP_MODE_DHCP            1
#define IP_MODE_PPPOE           2

#define MIXED_SERVER            0
#define MIXED_CLIENT            1

// On/Off Status
typedef enum {
    OFF = 0,
    ON  = 1
} OnOff_State_Type;

// True/False Status
typedef enum {
    FALSE = 0,
    TRUE  = 1
} TrueFalse_State_Type;

typedef enum {
    DISABLE = 0,
    ENABLE = !DISABLE
} FunctionalState;

#define OP_COMMAND              0
#define OP_DATA                 1

#define RET_OK                  0
#define RET_NOK                 -1
#define RET_TIMEOUT             -2

#define STR_UART                "UART"
#define STR_ENABLED             "Enabled"
#define STR_DISABLED            "Disabled"
#define STR_BAR                 "=================================================="
#define STR_MODBUS_RTU          "ModbusRTU"
#define STR_MODBUS_ASCII        "ModbusASCII"
#define STR_MODBUS_TCP          "ModbusTCP"

#endif //_COMMON_H


