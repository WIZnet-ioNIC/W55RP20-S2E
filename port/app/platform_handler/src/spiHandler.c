#include <string.h>
#include "common.h"
#include "ConfigData.h"
#include "segcp.h"
#include "bufferHandler.h"
#include "uartHandler.h"
#include "spiHandler.h"
#include "gpioHandler.h"
#include "deviceHandler.h"
#include "seg.h"
#include "port_common.h"
#include "WIZnet_board.h"

#define SPI_RESET_TIMEOUT_MS 10

extern xSemaphoreHandle seg_u2e_sem;
extern xSemaphoreHandle seg_spi_pending_sem;
extern xSemaphoreHandle segcp_uart_sem;

extern TimerHandle_t spi_reset_timer;

volatile spi_slave_state_t current_state = STATE_COMMAND;
volatile uint8_t command = 0;
volatile uint16_t data_length = 0;
volatile uint8_t dummy_count = 0;
volatile bool data_ready_to_send = false;
volatile uint8_t atcmd_bytes[4];  // Temporary buffer for atcmd bytes

static uint dma_tx;
static uint dma_rx;
static dma_channel_config dma_channel_config_tx;
static dma_channel_config dma_channel_config_rx;
uint16_t atcmd_size = 0;
uint32_t spi_clock = 0; // SPI clock frequency in Hz

//static uint8_t spi_buffer[2048];
extern uint8_t g_send_buf[];
extern uint8_t g_recv_buf[];
extern uint8_t gSEGCPREQ[];
extern uint8_t gSEGCPREP[];
extern uint16_t e2u_size;
extern uint16_t u2e_size;

void on_spi_rx(void) {
    //uartRxByte: // 1-byte character variable for UART Interrupt request handler
    uint8_t received_byte = 0;
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t length_bytes[2]; // Temporary buffer for length bytes
    static uint8_t length_index = 0;
    static uint8_t atcmd_index = 0;

    while (spi_is_readable(SPI_ID)) {
        received_byte = spi_get_hw(SPI_ID)->dr;
        switch (current_state) {
        case STATE_COMMAND:
            command = received_byte;
            //printf("command = 0x%X\r\n", command);
            if (command == SPI_MASTER_WRITE_CMD) {
                if (check_serial_store_permitted(received_byte)) { // ret: [0] not permitted / [1] permitted
                    current_state = STATE_LENGTH;
                    length_index = 0;
                } else {
                    current_state = STATE_NACK;
                    xSemaphoreGiveFromISR(seg_spi_pending_sem, &xHigherPriorityTaskWoken);
                    return;
                }
            } else if (command == SPI_MASTER_READ_LEN_CMD) {
                current_state = STATE_DUMMY;
            } else if ((command > 0x40) && (command < 0x5b)) { //'A' ~ 'Z'
                current_state = STATE_ATCMD;
                atcmd_bytes[atcmd_index++] = received_byte;
            }
            //printf("0 current_state = %d\r\n", current_state);
            break;

        case STATE_LENGTH:
            length_bytes[length_index++] = received_byte;
            if (length_index == 2) {
                //data_length = (length_bytes[0] << 8) | length_bytes[1];
                data_length = length_bytes[0] | (length_bytes[1] << 8);
                if (data_length > DATA_BUF_SIZE) {
                    //printf("Error: Data length too large\n");
                    current_state = STATE_NACK;
                    irq_set_enabled(SPI0_IRQ, false);
                    xSemaphoreGiveFromISR(seg_spi_pending_sem, &xHigherPriorityTaskWoken);
                    return;
                } else {
                    //printf("data_length = %d\r\n", data_length);
                    current_state = STATE_DUMMY;
                }
                length_index = 0;
            }
            //printf("1 current_state = %d\r\n", current_state);
            break;

        case STATE_DUMMY:
            if (received_byte == SPI_DUMMY) {
                dummy_count++;
                if ((command == SPI_MASTER_WRITE_CMD && dummy_count == 1) ||
                        (command == SPI_MASTER_READ_LEN_CMD && dummy_count == 3)) {
                    if (command == SPI_MASTER_WRITE_CMD) {
                        current_state = STATE_SLAVE_DATA_READ;
                    } else if (command == SPI_MASTER_READ_LEN_CMD) {
                        current_state = STATE_SLAVE_DATA_WRITE;
                    }
                    irq_set_enabled(SPI0_IRQ, false);
                    dummy_count = 0;
                    xSemaphoreGiveFromISR(seg_spi_pending_sem, &xHigherPriorityTaskWoken);
                    return;
                }
            } else {
                //printf("Invalid Dummy Byte: 0x%02X\n", received_byte);
                current_state = STATE_NACK;
                irq_set_enabled(SPI0_IRQ, false);
                xSemaphoreGiveFromISR(seg_spi_pending_sem, &xHigherPriorityTaskWoken);
                return;
            }
            //printf("2 current_state = %d\r\n", current_state);
            break;

        case STATE_ATCMD:
            atcmd_bytes[atcmd_index++] = received_byte;
            if (atcmd_index == 4) {
                irq_set_enabled(SPI0_IRQ, false);
                atcmd_index = 0;
                xSemaphoreGiveFromISR(seg_spi_pending_sem, &xHigherPriorityTaskWoken);
                return;
            }
            break;

        default:
            //printf("Invalid State current_state = %d recv = 0x%02X\n", current_state, received_byte);
            current_state = STATE_COMMAND;
            break;
        }
    }
}

void platform_spi_write(uint8_t *data, uint16_t data_len) {
    spi_write_blocking(SPI_ID, data, data_len);
}

void platform_spi_read(uint8_t *data, uint16_t data_len) {
    spi_read_blocking(SPI_ID, 0x00, data, data_len);
}

void platform_spi_write_dma(uint8_t *data, uint16_t data_len) {
    spi_slave_write_dma(data, data_len);
}

void platform_spi_read_dma(uint8_t *data, uint16_t data_len) {
    spi_slave_read_dma(data, data_len);
}

void platform_spi_transfer(uint8_t *write_data, uint8_t *read_data, uint16_t data_len) {
    spi_write_read_blocking(SPI_ID, write_data, read_data, data_len);
}

void platform_spi_reset(void) {
    current_state = STATE_COMMAND;
    e2u_size = 0;
    u2e_size = 0;
    GPIO_Output_Set(DATA0_SPI_INT_PIN);

    dma_channel_cleanup(dma_tx);
    dma_channel_cleanup(dma_rx);
    dma_channel_unclaim(dma_tx);
    dma_channel_unclaim(dma_rx);
    spi_deinit(SPI_ID);
    DATA0_SPI_Configuration(spi_clock * 12);
}

void DATA0_SPI_Configuration(uint32_t main_clock) {
    spi_clock = spi_init(SPI_ID, main_clock / 12);
    spi_set_slave(SPI_ID, true);

    gpio_init(DATA0_SPI_SCK_PIN);
    gpio_init(DATA0_SPI_RX_PIN);
    gpio_init(DATA0_SPI_TX_PIN);
    gpio_init(DATA0_SPI_CSn_PIN);
    //gpio_init(DATA0_SPI_INT_PIN);

    gpio_set_function(DATA0_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DATA0_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DATA0_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DATA0_SPI_CSn_PIN, GPIO_FUNC_SPI);
    //GPIO_Configuration(DATA0_SPI_INT_PIN, IO_OUTPUT, IO_PULLUP);
    //GPIO_Output_Set(DATA0_SPI_INT_PIN);

    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(DATA0_SPI_SCK_PIN, DATA0_SPI_RX_PIN, DATA0_SPI_TX_PIN, DATA0_SPI_CSn_PIN, GPIO_FUNC_SPI));

    irq_set_exclusive_handler(SPI0_IRQ, on_spi_rx);
    irq_set_enabled(SPI0_IRQ, true);
    spi_get_hw(SPI_ID)->imsc = SPI_SSPIMSC_RXIM_BITS; // RX interrupt

    dma_tx = dma_claim_unused_channel(true);
    dma_rx = dma_claim_unused_channel(true);

    dma_channel_config_tx = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&dma_channel_config_tx, DMA_SIZE_8);
    if (SPI_ID == spi0) {
        channel_config_set_dreq(&dma_channel_config_tx, DREQ_SPI0_TX);
    } else if (SPI_ID == spi1) {
        channel_config_set_dreq(&dma_channel_config_tx, DREQ_SPI1_TX);
    }

    channel_config_set_read_increment(&dma_channel_config_tx, true);
    channel_config_set_write_increment(&dma_channel_config_tx, false);

    // We set the inbound DMA to transfer from the SPI receive FIFO to a memory buffer paced by the SPI RX FIFO DREQ
    // We coinfigure the read address to remain unchanged for each element, but the write
    // address to increment (so data is written throughout the buffer)
    dma_channel_config_rx = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&dma_channel_config_rx, DMA_SIZE_8);
    if (SPI_ID == spi0) {
        channel_config_set_dreq(&dma_channel_config_rx, DREQ_SPI0_RX);
    } else if (SPI_ID == spi1) {
        channel_config_set_dreq(&dma_channel_config_rx, DREQ_SPI1_RX);
    }
    //channel_config_set_dreq(&dma_channel_config_tx, spi_get_dreq(SPI_ID, false));
    channel_config_set_read_increment(&dma_channel_config_rx, false);
    channel_config_set_write_increment(&dma_channel_config_rx, true);

    if (spi_reset_timer == NULL) {
        // Create a timer to reset the SPI state after a timeout
        spi_reset_timer = xTimerCreate("spi_reset_timer", SPI_RESET_TIMEOUT_MS / portTICK_PERIOD_MS, pdFALSE, NULL, spi_reset_timer_callback);
    }
}

static void spi_slave_read_dma(uint8_t *pBuf, uint16_t len) {
    //dma_channel_wait_for_finish_blocking(dma_rx);

    dma_channel_configure(dma_rx, &dma_channel_config_rx,
                          pBuf,                      // write address
                          &spi_get_hw(SPI_ID)->dr, // read address
                          len,                       // element count (each element is of size transfer_data_size)
                          true);                    // don't start yet

    //dma_start_channel_mask(1u << dma_rx);
    dma_channel_wait_for_finish_blocking(dma_rx);
    //while(spi_is_busy(SPI_ID));
}

static void spi_slave_write_dma(uint8_t *pBuf, uint16_t len) {
    dma_channel_configure(dma_tx, &dma_channel_config_tx,
                          &spi_get_hw(SPI_ID)->dr, // write address
                          pBuf,                      // read address
                          len,                       // element count (each element is of size transfer_data_size)
                          true);                    // don't start yet

    //dma_start_channel_mask(1u << dma_tx);
    dma_channel_wait_for_finish_blocking(dma_tx);
    while (spi_is_busy(SPI_ID));

}

void spi_data_transfer_task(void *argument) {
    uint8_t header[4];

    while (1) {
        //printf("spi_data_transfer_task current_state = %d\r\n", current_state);
        xSemaphoreTake(seg_spi_pending_sem, portMAX_DELAY);
        switch (current_state) {
        case STATE_SLAVE_DATA_WRITE:
            xTimerStop(spi_reset_timer, 0);
            header[0] = SPI_SLAVE_WRITE_LEN_CMD;
            memcpy(&header[1], &e2u_size, 2);
            header[3] = SPI_DUMMY;

            if (e2u_size) {
                memcpy(get_data_buffer_ptr(), header, 4);
                memcpy(get_data_buffer_ptr() + 4, g_recv_buf, e2u_size);
                vTaskEnterCritical();
                platform_spi_write(get_data_buffer_ptr(), e2u_size + 4);
                vTaskExitCritical();
                //platform_spi_write_dma(get_data_buffer_ptr(), e2u_size + 4);
                e2u_size = 0;
                current_state = STATE_COMMAND;
                irq_set_enabled(SPI0_IRQ, true);
                GPIO_Output_Set(DATA0_SPI_INT_PIN);
            } else {
                vTaskEnterCritical();
                platform_spi_write(header, 4);
                vTaskExitCritical();
                //platform_spi_write_dma(header, 4);
                current_state = STATE_COMMAND;
                irq_set_enabled(SPI0_IRQ, true);
            }
            break;

        case STATE_SLAVE_DATA_READ:
#if 1
            vTaskEnterCritical();
            spi_send_ack();
            platform_spi_read(g_send_buf, data_length);
            //platform_spi_read_dma(g_send_buf, data_length);
            vTaskExitCritical();
#endif
            u2e_size = data_length;
            xSemaphoreGive(seg_u2e_sem);
            current_state = STATE_COMMAND;
            break;

        case STATE_ATCMD:
            memset(gSEGCPREQ, 0x00, CONFIG_BUF_SIZE);
            if (memcmp(atcmd_bytes + 2, SEGCP_DELIMETER, 2)) {
                atcmd_size = atcmd_bytes[2] | (atcmd_bytes[3] << 8);
                memcpy(gSEGCPREQ, atcmd_bytes, 2);
                spi_send_ack();
                //platform_spi_read_dma(gSEGCPREQ + 2, atcmd_size);
                platform_spi_read(gSEGCPREQ + 2, atcmd_size);
                atcmd_size += 2;
            } else {
                memcpy(gSEGCPREQ, atcmd_bytes, 4);
            }
            current_state = STATE_COMMAND;
            xSemaphoreGive(segcp_uart_sem);
            break;

        case STATE_NACK:
            spi_send_nack();
            current_state = STATE_COMMAND;
            irq_set_enabled(SPI0_IRQ, true);
            break;

        default:
            break;
        }
    }
}

void spi_send_ack(void) {
    uint8_t header[4];
    header[0] = SPI_ACK;
    header[1] = SPI_DUMMY;
    header[2] = SPI_DUMMY;
    header[3] = SPI_DUMMY;

    //PRT_INFO("Send ACK\r\n");
    platform_spi_write(header, 4);
    //platform_spi_write_dma(header, 4);
}

void spi_send_nack(void) {
    uint8_t header[4];
    header[0] = SPI_NACK;
    header[1] = SPI_DUMMY;
    header[2] = SPI_DUMMY;
    header[3] = SPI_DUMMY;

    //PRT_INFO("Send NACK\r\n");
    platform_spi_write(header, 4);
    //platform_spi_write_dma(header, 4);
}

void spi_reset_timer_callback(TimerHandle_t xTimer) {
    // Reset the SPI interface
    //PRT_INFO("SPI reset due to timeout\r\n");
    platform_spi_reset();
}