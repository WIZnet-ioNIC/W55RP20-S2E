#include <string.h>
#include "common.h"
#include "ConfigData.h"
#include "bufferHandler.h"
#include "uartHandler.h"
#include "gpioHandler.h"
#include "seg.h"
#include "port_common.h"
#include "WIZnet_board.h"

typedef struct {
    uint8_t data[SEG_DATA_BUF_SIZE];
    volatile uint16_t write_index;
    volatile uint16_t read_index;
} uart_ring_buffer_t;

static uart_ring_buffer_t uart_rx_buffer[DEVICE_UART_CNT];
static volatile uint32_t uart_rx_overflow[DEVICE_UART_CNT];

static uart_ring_buffer_t *get_uart_rx_buffer(int channel) {
    if ((channel < 0) || (channel >= DEVICE_UART_CNT)) {
        return NULL;
    }
    return &uart_rx_buffer[channel];
}

static uint16_t ring_buffer_used_size(const uart_ring_buffer_t *buffer) {
    return (SEG_DATA_BUF_SIZE + buffer->write_index - buffer->read_index) % SEG_DATA_BUF_SIZE;
}

static uint16_t ring_buffer_free_size(const uart_ring_buffer_t *buffer) {
    return (SEG_DATA_BUF_SIZE + buffer->read_index - buffer->write_index - 1) % SEG_DATA_BUF_SIZE;
}

void data_buffer_flush(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    if (buffer == NULL) {
        return;
    }

    buffer->write_index = 0;
    buffer->read_index = 0;
}

// Drop the incoming byte when the buffer is full instead of advancing the write
// index past unread data. The original write always stored the byte, so a full
// buffer silently overwrote data the seg task had not read yet - a loss that
// left no trace in any counter. The overflow counter now makes it visible.
//
// Original body (after the NULL check):
//     buffer->data[buffer->write_index] = ch;
//     buffer->write_index = (buffer->write_index + 1) % SEG_DATA_BUF_SIZE;
void put_byte_to_data_buffer(uint8_t ch, int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    if (buffer == NULL) {
        return;
    }

    if (ring_buffer_free_size(buffer) == 0) {
        uart_rx_overflow[channel]++;
        return;
    }

    buffer->data[buffer->write_index] = ch;
    buffer->write_index = (buffer->write_index + 1) % SEG_DATA_BUF_SIZE;
}

uint32_t get_data_buffer_overflow_count(int channel) {
    if ((channel < 0) || (channel >= DEVICE_UART_CNT)) {
        return 0;
    }
    return uart_rx_overflow[channel];
}

uint16_t get_data_buffer_usedsize(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    return (buffer == NULL) ? 0 : ring_buffer_used_size(buffer);
}

uint16_t get_data_buffer_freesize(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    return (buffer == NULL) ? 0 : ring_buffer_free_size(buffer);
}

uint8_t *get_data_buffer_ptr(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    return (buffer == NULL) ? NULL : buffer->data;
}

int8_t is_data_buffer_empty(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    return (buffer == NULL) ? TRUE : (buffer->read_index == buffer->write_index);
}

int8_t is_data_buffer_full(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    return (buffer == NULL) ? TRUE : (ring_buffer_free_size(buffer) == 0);
}

int32_t data_buffer_getc(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    if (buffer == NULL) {
        return RET_NOK;
    }

    while (buffer->read_index == buffer->write_index) {
        tight_loop_contents();
    }

    int32_t ch = buffer->data[buffer->read_index];
    buffer->read_index = (buffer->read_index + 1) % SEG_DATA_BUF_SIZE;
    return ch;
}

int32_t data_buffer_getc_nonblk(int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    if ((buffer == NULL) || (buffer->read_index == buffer->write_index)) {
        return RET_NOK;
    }

    int32_t ch = buffer->data[buffer->read_index];
    buffer->read_index = (buffer->read_index + 1) % SEG_DATA_BUF_SIZE;
    return ch;
}

int32_t data_buffer_gets(uint8_t* buf, uint16_t bytes, int channel) {
    uart_ring_buffer_t *buffer = get_uart_rx_buffer(channel);
    if ((buffer == NULL) || (buf == NULL)) {
        return 0;
    }

    uint16_t total = MIN(ring_buffer_used_size(buffer), bytes);
    uint16_t first = MIN(total, SEG_DATA_BUF_SIZE - buffer->read_index);
    uint16_t second = total - first;

    memcpy(buf, &buffer->data[buffer->read_index], first);
    if (second > 0) {
        memcpy(buf + first, buffer->data, second);
    }

    buffer->read_index = (buffer->read_index + total) % SEG_DATA_BUF_SIZE;
    return total;
}
