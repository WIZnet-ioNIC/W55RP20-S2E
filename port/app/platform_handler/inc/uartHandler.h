#ifndef UARTHANDLER_H_
#define UARTHANDLER_H_

#include <stdint.h>
#include "port_common.h"
#include "common.h"

//#define _UART_DEBUG_

#ifndef DATA_BUF_SIZE
    #define DATA_BUF_SIZE 2048
#endif

// XON/XOFF: Transmitter On / Off, Software flow control
#define UART_XON                        0x11 // 17
#define UART_XOFF                       0x13 // 19
#define UART_ON_THRESHOLD               (uint16_t)(SEG_DATA_BUF_SIZE / 10)
#define UART_OFF_THRESHOLD              (uint16_t)(SEG_DATA_BUF_SIZE - UART_ON_THRESHOLD)

// UART interface selector, RS-232/TTL or RS-422/485
#define UART_IF_RS232_TTL               0
#define UART_IF_RS422                   1
#define UART_IF_RS485                   2
#define UART_IF_RS485_REVERSE           3
#define SPI_IF_SLAVE                    4

#define UART_IF_STR_RS232_TTL          "TTL/RS-232"
//#define UART_IF_STR_RS232               "RS-232"
#define UART_IF_STR_RS422               "RS-422"
#define UART_IF_STR_RS485               "RS-485"
#define SPI_IF_STR_SLAVE                "SPI_SLAVE"

// If the define '__USE_UART_IF_SELECTOR__' disabled, default UART interface is selected to be 'UART_IF_DEFAULT'
//#define UART_IF_DEFAULT                 UART_IF_RS485
#define UART_IF_DEFAULT               UART_IF_RS232_TTL

// UART RTS/CTS pins
// RTS: output, CTS: input
#define UART_RTS_HIGH                   1
#define UART_RTS_LOW                    0

#define UART_CTS_HIGH                   1
#define UART_CTS_LOW                    0

#define UART_ID uart1

//#define DEBUG_UART_DEFAULT_BAUDRATE     (115200)
#define DEBUG_UART_DEFAULT_BAUDRATE     (921600)
//#define DEBUG_UART_DEFAULT_BAUDRATE     (3000000)

enum baud {
    baud_300 = 0,
    baud_600 = 1,
    baud_1200 = 2,
    baud_1800 = 3,
    baud_2400 = 4,
    baud_4800 = 5,
    baud_9600 = 6,
    baud_14400 = 7,
    baud_19200 = 8,
    baud_28800 = 9,
    baud_38400 = 10,
    baud_57600 = 11,
    baud_115200 = 12,
    baud_230400 = 13,
    baud_460800 = 14,
    baud_921600 = 15
};

enum word_len {
    word_len7 = 0,
    word_len8 = 1,
    word_len9 = 2
};

enum stop_bit {
    stop_bit1 = 0,
    stop_bit2 = 1
};

enum parity {
    parity_none = 0,
    parity_odd = 1,
    parity_even = 2
};

enum flow_ctrl {
    flow_none = 0,
    flow_xon_xoff = 1,
    flow_rts_cts = 2,
    flow_rtsonly = 3,  // RTS_ONLY
    flow_reverserts = 4 // Reverse RTS
};

enum protocol {
    protocol_none = 0,
    modbus_rtu = 1,
    modbus_ascii = 2
};

extern uint32_t baud_table[];
extern uint8_t word_len_table[];
extern uint8_t stop_bit_table[];
extern uint8_t * parity_table[];
extern uint8_t * flow_ctrl_table[];
extern uint8_t * uart_if_table[];

void on_uart_rx(void);
void DEBUG_UART_Configuration(void);
void DATA0_UART_Configuration(void);
void DATA0_UART_Deinit(void);
void DATA0_UART_Interrupt_Enable(void);
void DATA1_UART_Configuration(void);

// XON/XOFF Software flow control: Check the Buffer usage and Send the start/stop commands
void check_uart_flow_control(uint8_t flow_ctrl);

// Hardware flow control by GPIOs (RTS/CTS)
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
    uint8_t get_uart_cts_pin(void);
    void set_uart_rts_pin_high(void);
    void set_uart_rts_pin_low(void);
#endif

int32_t platform_uart_putc(uint16_t ch);                    // User Buffer -> UART
uint8_t get_uart_rs485_sel(void);
void uart_rs485_rs422_init(void);
void uart_rs485_disable(void);
void uart_rs485_enable(void);

#endif /* UARTHANDLER_H_ */
