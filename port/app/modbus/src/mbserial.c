#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "port_common.h"
#include "WIZnet_board.h"

#include "bufferHandler.h"
#include "uartHandler.h"
#include "common.h"
#include "mbserial.h"
#include "mbrtu.h"
#include "mbascii.h"
#include "seg.h"

BUFFER_DECLARATION(data0_serial_rx);

/*****************************************************************************
    Private functions
 ****************************************************************************/

/**
    @brief	UART interrupt handler using ring buffers
    @return	Nothing
*/

int UART_read(void *data, int bytes, int channel) {
    return data_buffer_gets(data, bytes, SEG_DATA0_CH);
}

uint32_t UART_write(void *data, int bytes, int channel) {
    return platform_uart_puts(data, bytes, SEG_DATA0_CH);
}
