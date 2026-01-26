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

BUFFER_DECLARATION(data0_serial_rx);

/*****************************************************************************
    Private functions
 ****************************************************************************/

/**
    @brief	UART interrupt handler using ring buffers
    @return	Nothing
*/

int UART_read(void *data, int bytes) {
<<<<<<< HEAD
    uint32_t i;
    uint8_t *data_ptr = data;
    if (IS_BUFFER_EMPTY(data0_rx)) {
        return RET_NOK;
    }

    for (i = 0; i < bytes; i++) {
        data_ptr[i] = (uint8_t)BUFFER_OUT(data0_rx);
    }
    BUFFER_OUT_MOVE(data0_rx, i);
    return i;
=======
    return data_buffer_gets(data, bytes);
>>>>>>> SPI
}

uint32_t UART_write(void *data, int bytes) {
    return platform_uart_puts(data, bytes);
}
