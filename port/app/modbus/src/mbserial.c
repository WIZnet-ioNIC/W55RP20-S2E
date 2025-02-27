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
 * Private functions
 ****************************************************************************/

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */

int UART_read(void *data, int bytes)
{
  return data_buffer_gets(data, bytes);  
}

uint32_t UART_write(void *data, int bytes)
{
  uint32_t i;
  uint8_t *data_ptr = data;

  for(i=0; i<bytes; i++)
    uart_putc(UART_ID, data_ptr[i]);
  return bytes;
}
