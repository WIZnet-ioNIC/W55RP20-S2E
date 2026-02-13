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

static spin_lock_t *buf_lock = NULL;

void data_buffer_init_lock(void) {
    buf_lock = spin_lock_init(spin_lock_claim_unused(true));
}

void data_buffer_flush(int channel) {
    if (channel == SEG_DATA0_CH) {
        BUFFER_CLEAR(data0_buffer_rx);
    } else {
        BUFFER_CLEAR(data1_buffer_rx);
    }
}

void put_byte_to_data_buffer(uint8_t ch, int channel) {
    uint32_t saved = spin_lock_blocking(buf_lock);

    if (channel == SEG_DATA0_CH) {
        BUFFER_IN(data0_buffer_rx) = ch;
        BUFFER_IN_MOVE(data0_buffer_rx, 1);
    } else {
        BUFFER_IN(data1_buffer_rx) = ch;
        BUFFER_IN_MOVE(data1_buffer_rx, 1);
    }
    spin_unlock(buf_lock, saved);
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
#if 0
/*
    Original implementation: buggy ring buffer read
    incorrect offset when buffer is wrap-around (rd > wr) and len1st >= bytes
*/
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
#endif

int32_t data_buffer_gets(uint8_t* buf, uint16_t bytes, int channel) {
    uint16_t lentot = 0, len1st = 0;
    taskENTER_CRITICAL();
    uint32_t saved = spin_lock_blocking(buf_lock);  // Acquire spin lock to prevent simultaneous access from both cores (RP2040 dual-core)

    if (channel == SEG_DATA0_CH) {
        lentot = bytes = MIN(BUFFER_USED_SIZE(data0_buffer_rx), bytes);  // Clamp read size to available data
        if (IS_BUFFER_OUT_SEPARATED(data0_buffer_rx)) {  // Check if buffer is wrap-around state (rd > wr)
            len1st = BUFFER_OUT_1ST_SIZE(data0_buffer_rx);  // Size from rd to end of buffer
            if (len1st < bytes) {
                // Data spans across buffer boundary: copy in two parts
                memcpy(buf, &BUFFER_OUT(data0_buffer_rx), len1st);          // 1st part: rd to end of buffer
                BUFFER_OUT_MOVE(data0_buffer_rx, len1st);                   // Advance rd to buffer start (wrap)
                bytes -= len1st;
                memcpy(buf + len1st, &BUFFER_OUT(data0_buffer_rx), bytes);  // 2nd part: buffer start to remaining bytes
                BUFFER_OUT_MOVE(data0_buffer_rx, bytes);
            } else {
                // 1st part is enough: copy without crossing boundary
                memcpy(buf, &BUFFER_OUT(data0_buffer_rx), bytes);
                BUFFER_OUT_MOVE(data0_buffer_rx, bytes);
            }
        } else {
            // No wrap-around: data is contiguous, copy directly
            memcpy(buf, &BUFFER_OUT(data0_buffer_rx), bytes);
            BUFFER_OUT_MOVE(data0_buffer_rx, bytes);
        }
    } else {
        lentot = bytes = MIN(BUFFER_USED_SIZE(data1_buffer_rx), bytes);  // Clamp read size to available data
        if (IS_BUFFER_OUT_SEPARATED(data1_buffer_rx)) {  // Check if buffer is wrap-around state (rd > wr)
            len1st = BUFFER_OUT_1ST_SIZE(data1_buffer_rx);  // Size from rd to end of buffer
            if (len1st < bytes) {
                // Data spans across buffer boundary: copy in two parts
                memcpy(buf, &BUFFER_OUT(data1_buffer_rx), len1st);          // 1st part: rd to end of buffer
                BUFFER_OUT_MOVE(data1_buffer_rx, len1st);                   // Advance rd to buffer start (wrap)
                bytes -= len1st;
                memcpy(buf + len1st, &BUFFER_OUT(data1_buffer_rx), bytes);  // 2nd part: buffer start to remaining bytes
                BUFFER_OUT_MOVE(data1_buffer_rx, bytes);
            } else {
                // 1st part is enough: copy without crossing boundary
                memcpy(buf, &BUFFER_OUT(data1_buffer_rx), bytes);
                BUFFER_OUT_MOVE(data1_buffer_rx, bytes);
            }
        } else {
            // No wrap-around: data is contiguous, copy directly
            memcpy(buf, &BUFFER_OUT(data1_buffer_rx), bytes);
            BUFFER_OUT_MOVE(data1_buffer_rx, bytes);
        }
    }
    spin_unlock(buf_lock, saved);  // Release spin lock
    taskEXIT_CRITICAL();
    return lentot;
}
