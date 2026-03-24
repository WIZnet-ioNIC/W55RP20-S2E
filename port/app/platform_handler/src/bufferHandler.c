#include <string.h>
#include "common.h"
#include "ConfigData.h"
#include "bufferHandler.h"
#include "uartHandler.h"
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
BUFFER_DEFINITION(data1_buffer_rx, SEG_DATA_BUF_SIZE);

void data_buffer_flush(int channel) {
    if (channel == SEG_DATA0_CH) {
        BUFFER_CLEAR(data0_buffer_rx);
    } else {
        BUFFER_CLEAR(data1_buffer_rx);
    }
}

void put_byte_to_data_buffer(uint8_t ch, int channel) {
    if (channel == SEG_DATA0_CH) {
        BUFFER_IN(data0_buffer_rx) = ch;
        BUFFER_IN_MOVE(data0_buffer_rx, 1);
    } else {
        BUFFER_IN(data1_buffer_rx) = ch;
        BUFFER_IN_MOVE(data1_buffer_rx, 1);
    }
}

uint16_t get_data_buffer_usedsize(int channel) {
    if (channel == SEG_DATA0_CH) {
        return BUFFER_USED_SIZE(data0_buffer_rx);
    } else {
        return BUFFER_USED_SIZE(data1_buffer_rx);
    }
}

uint16_t get_data_buffer_freesize(int channel) {
    if (channel == SEG_DATA0_CH) {
        return BUFFER_FREE_SIZE(data0_buffer_rx);
    } else {
        return BUFFER_FREE_SIZE(data1_buffer_rx);
    }
}

uint8_t *get_data_buffer_ptr(int channel) {
    if (channel == SEG_DATA0_CH) {
        return BUFFER_PTR(data0_buffer_rx);
    } else {
        return BUFFER_PTR(data1_buffer_rx);
    }
}

int8_t is_data_buffer_empty(int channel) {
    if (channel == SEG_DATA0_CH) {
        return IS_BUFFER_EMPTY(data0_buffer_rx);
    } else {
        return IS_BUFFER_EMPTY(data1_buffer_rx);
    }
}

int8_t is_data_buffer_full(int channel) {
    if (channel == SEG_DATA0_CH) {
        return IS_BUFFER_FULL(data0_buffer_rx);
    } else {
        return IS_BUFFER_FULL(data1_buffer_rx);
    }
}

int32_t data_buffer_getc(int channel) {
    int32_t ch;

    if (channel == SEG_DATA0_CH) {
        while (IS_BUFFER_EMPTY(data0_buffer_rx));
        ch = (int32_t)BUFFER_OUT(data0_buffer_rx);
        BUFFER_OUT_MOVE(data0_buffer_rx, 1);
    } else {
        while (IS_BUFFER_EMPTY(data1_buffer_rx));
        ch = (int32_t)BUFFER_OUT(data1_buffer_rx);
        BUFFER_OUT_MOVE(data1_buffer_rx, 1);
    }
    return ch;
}

int32_t data_buffer_getc_nonblk(int channel) {
    int32_t ch;

    if (channel == SEG_DATA0_CH) {
        if (IS_BUFFER_EMPTY(data0_buffer_rx)) {
            return RET_NOK;
        }
        ch = (int32_t)BUFFER_OUT(data0_buffer_rx);
        BUFFER_OUT_MOVE(data0_buffer_rx, 1);
    } else {
        if (IS_BUFFER_EMPTY(data1_buffer_rx)) {
            return RET_NOK;
        }
        ch = (int32_t)BUFFER_OUT(data1_buffer_rx);
        BUFFER_OUT_MOVE(data1_buffer_rx, 1);
    }
    return ch;
}

int32_t data_buffer_gets(uint8_t* buf, uint16_t bytes, int channel) {
    uint16_t lentot = 0, len1st = 0;
    taskENTER_CRITICAL();
    if (channel == SEG_DATA0_CH) {
        lentot = bytes = MIN(BUFFER_USED_SIZE(data0_buffer_rx), bytes);
        if (IS_BUFFER_OUT_SEPARATED(data0_buffer_rx) && (len1st = BUFFER_OUT_1ST_SIZE(data0_buffer_rx)) < bytes) {
            memcpy(buf, &BUFFER_OUT(data0_buffer_rx), len1st);
            BUFFER_OUT_MOVE(data0_buffer_rx, len1st);
            bytes -= len1st;
        }
        memcpy(buf + len1st, &BUFFER_OUT(data0_buffer_rx), bytes);
        BUFFER_OUT_MOVE(data0_buffer_rx, bytes);
    } else {
        lentot = bytes = MIN(BUFFER_USED_SIZE(data1_buffer_rx), bytes);
        if (IS_BUFFER_OUT_SEPARATED(data1_buffer_rx) && (len1st = BUFFER_OUT_1ST_SIZE(data1_buffer_rx)) < bytes) {
            memcpy(buf, &BUFFER_OUT(data1_buffer_rx), len1st);
            BUFFER_OUT_MOVE(data1_buffer_rx, len1st);
            bytes -= len1st;
        }
        memcpy(buf + len1st, &BUFFER_OUT(data1_buffer_rx), bytes);
        BUFFER_OUT_MOVE(data1_buffer_rx, bytes);
    }

    taskEXIT_CRITICAL();
    return lentot;
}

