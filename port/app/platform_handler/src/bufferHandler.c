#include <string.h>
#include "common.h"
#include "ConfigData.h"
#include "bufferHandler.h"
#include "uartHandler.h"
#include "spiHandler.h"
#include "gpioHandler.h"
#include "seg.h"
#include "port_common.h"
#include "WIZnet_board.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private functions prototypes ----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// UART Ring buffer declaration
BUFFER_DEFINITION(data0_buffer_rx, SEG_DATA_BUF_SIZE);

void data_buffer_flush(void)
{
    BUFFER_CLEAR(data0_buffer_rx);
}

void put_byte_to_data_buffer(uint8_t ch)
{
    BUFFER_IN(data0_buffer_rx) = ch;
    BUFFER_IN_MOVE(data0_buffer_rx, 1);
}

uint16_t get_data_buffer_usedsize(void)
{
    return BUFFER_USED_SIZE(data0_buffer_rx);
}

uint16_t get_data_buffer_freesize(void)
{
    return BUFFER_FREE_SIZE(data0_buffer_rx);
}

uint8_t *get_data_buffer_ptr(void)
{
    return BUFFER_PTR(data0_buffer_rx);
}

int8_t is_data_buffer_empty(void)
{
    return IS_BUFFER_EMPTY(data0_buffer_rx);
}

int8_t is_data_buffer_full(void)
{
    return IS_BUFFER_FULL(data0_buffer_rx);
}

int32_t data_buffer_getc(void)
{
    int32_t ch;

    while(IS_BUFFER_EMPTY(data0_buffer_rx));
    ch = (int32_t)BUFFER_OUT(data0_buffer_rx);
    BUFFER_OUT_MOVE(data0_buffer_rx, 1);

    return ch;
}

int32_t data_buffer_getc_nonblk(void)
{
    int32_t ch;

    if(IS_BUFFER_EMPTY(data0_buffer_rx)) return RET_NOK;
    ch = (int32_t)BUFFER_OUT(data0_buffer_rx);
    BUFFER_OUT_MOVE(data0_buffer_rx, 1);

    return ch;
}

int32_t data_buffer_gets(uint8_t* buf, uint16_t bytes)
{
    uint16_t lentot = 0, len1st = 0;

    lentot = bytes = MIN(BUFFER_USED_SIZE(data0_buffer_rx), bytes);
    if(IS_BUFFER_OUT_SEPARATED(data0_buffer_rx) && (len1st = BUFFER_OUT_1ST_SIZE(data0_buffer_rx)) < bytes) {
        memcpy(buf, &BUFFER_OUT(data0_buffer_rx), len1st);
        BUFFER_OUT_MOVE(data0_buffer_rx, len1st);
        bytes -= len1st;
    }
    memcpy(buf+len1st, &BUFFER_OUT(data0_buffer_rx), bytes);
    BUFFER_OUT_MOVE(data0_buffer_rx, bytes);

    return lentot;
}

