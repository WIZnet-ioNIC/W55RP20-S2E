#include <string.h>
#include "common.h"
#include "ConfigData.h"
#include "deviceHandler.h"
#include "uartHandler.h"
#include "gpioHandler.h"
#include "bufferHandler.h"
#include "seg.h"
#include "port_common.h"
#include "WIZnet_board.h"
#include "uart_tx.pio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private functions prototypes ----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

#if (DEVICE_BOARD_NAME == W232N)
uint32_t baud_table[] = {300, 600, 1200, 1800, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 230400};
#else
uint32_t baud_table[] = {300, 600, 1200, 1800, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 230400, 460800, 921600};
#endif
uint8_t word_len_table[] = {7, 8, 9};
uint8_t * parity_table[] = {(uint8_t *)"N", (uint8_t *)"ODD", (uint8_t *)"EVEN"};
uint8_t stop_bit_table[] = {1, 2};
uint8_t * flow_ctrl_table[] = {(uint8_t *)"NONE", (uint8_t *)"XON/XOFF", (uint8_t *)"RTS/CTS", (uint8_t *)"RTS Only", (uint8_t *)"RTS Only Reverse"};
uint8_t * uart_if_table[] = {(uint8_t *)UART_IF_STR_RS232_TTL, (uint8_t *)UART_IF_STR_RS422, (uint8_t *)UART_IF_STR_RS485, (uint8_t *)UART_IF_STR_RS485};

// XON/XOFF Status;
static uint8_t xonoff_status = UART_XON;

// RTS Status; __USE_GPIO_HARDWARE_FLOWCONTROL__ defined
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
static uint8_t rts_status = UART_RTS_LOW;
#endif

// UART Interface selector; RS-422 or RS-485 use only
static uint8_t uart_if_mode[DEVICE_UART_CNT] = {UART_IF_RS422, UART_IF_RS422};

extern xSemaphoreHandle seg_u2e_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle segcp_uart_sem;

uint dma_uart_tx[DEVICE_UART_CNT];
dma_channel_config dma_uart_c[DEVICE_UART_CNT];

/* Public functions ----------------------------------------------------------*/

////////////////////////////////////////////////////////////////////////////////
// Data UART Configuration
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Data UART Configuration & IRQ handler
////////////////////////////////////////////////////////////////////////////////

// RX interrupt handler
void data0_uart_rx(void) {
    //uartRxByte: // 1-byte character variable for UART Interrupt request handler
    uint8_t ch = 0, input_flag = 0;
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(DATA0_UART_ID)) {
        ch = uart_getc(DATA0_UART_ID);

        if (!(check_modeswitch_trigger(ch))) { // ret: [0] data / [!0] trigger code
            if (is_data_buffer_full(SEG_DATA0_CH) == TRUE) {
                data_buffer_flush(SEG_DATA0_CH);
            }

            if (check_serial_store_permitted(ch, SEG_DATA0_CH)) { // ret: [0] not permitted / [1] permitted
                put_byte_to_data_buffer(ch, SEG_DATA0_CH);
                input_flag = 1;
            }
        }
    }

    if (input_flag) {
        init_time_delimiter_timer(SEG_DATA0_CH);
#if 0
        if (opmode == DEVICE_GW_MODE) {
            xSemaphoreGiveFromISR(seg_u2e_sem[SEG_DATA0_CH], &xHigherPriorityTaskWoken);
        } else if (opmode == DEVICE_AT_MODE) {
            xSemaphoreGiveFromISR(segcp_uart_sem, &xHigherPriorityTaskWoken);
        }
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#else
        if (opmode == DEVICE_AT_MODE) {
            xSemaphoreGiveFromISR(segcp_uart_sem, &xHigherPriorityTaskWoken);
        }
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif
    }
}

void data1_uart_rx(void) {
    //uartRxByte: // 1-byte character variable for UART Interrupt request handler
    uint8_t ch = 0, input_flag = 0;
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(DATA1_UART_ID)) {
        ch = uart_getc(DATA1_UART_ID);

        if (is_data_buffer_full(SEG_DATA1_CH) == TRUE) {
            data_buffer_flush(SEG_DATA1_CH);
        }

        if (check_serial_store_permitted(ch, SEG_DATA1_CH)) { // ret: [0] not permitted / [1] permitted
            put_byte_to_data_buffer(ch, SEG_DATA1_CH);
            input_flag = 1;
        }
    }

    if (input_flag) {
        init_time_delimiter_timer(SEG_DATA1_CH);
        if (opmode == DEVICE_GW_MODE) {
            xSemaphoreGiveFromISR(seg_u2e_sem[SEG_DATA1_CH], &xHigherPriorityTaskWoken);
        }
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}


void DATA_UART_Configuration(void) {
    struct __serial_option *serial_option;
    uint8_t valid_arg = 0;
    uint8_t temp_data_bits, temp_stop_bits, temp_parity;
    uart_inst_t *uart_id[DEVICE_UART_CNT] = {DATA0_UART_ID, DATA1_UART_ID};

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select

    gpio_init(DATA0_UART_TX_PIN);
    gpio_init(DATA0_UART_RX_PIN);
    gpio_init(DATA0_UART_CTS_PIN);
    gpio_init(DATA0_UART_RTS_PIN);

    gpio_set_function(DATA0_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_CTS_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_RTS_PIN, GPIO_FUNC_UART);
    gpio_pull_up(DATA0_UART_RX_PIN);

    serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[SEG_DATA0_CH]);
    // Deinitialize UART
    uart_deinit(uart_id[SEG_DATA0_CH]);

    // Set up our UART with a basic baud rate.
    uart_init(uart_id[SEG_DATA0_CH], 2400);

    /* Set Baud Rate */
    if (serial_option->baud_rate < (sizeof(baud_table) / sizeof(baud_table[0]))) {
        uart_set_baudrate(uart_id[SEG_DATA0_CH], baud_table[serial_option->baud_rate]);
        valid_arg = 1;
    }

    if (!valid_arg) {
        uart_set_baudrate(uart_id[SEG_DATA0_CH], baud_table[baud_115200]);
    }

    /* Set Data Bits */
    switch (serial_option->data_bits) {
    case word_len7:
        temp_data_bits = 7;
        break;
    case word_len8:
        temp_data_bits = 8;
        break;
    case word_len9:
        temp_data_bits = 9;
        break;
    default:
        temp_data_bits = 8;
        serial_option->data_bits = word_len8;
        break;
    }

    /* Set Stop Bits */
    switch (serial_option->stop_bits) {
    case stop_bit1:
        temp_stop_bits = 1;
        break;
    case stop_bit2:
        temp_stop_bits = 2;
        break;
    default:
        temp_stop_bits = 1;
        serial_option->stop_bits = stop_bit1;
        break;
    }

    /* Set Parity Bits */
    switch (serial_option->parity) {
    case parity_none:
        temp_parity = UART_PARITY_NONE;
        break;
    case parity_odd:
        temp_parity = UART_PARITY_ODD;
        break;
    case parity_even:
        temp_parity = UART_PARITY_EVEN;
        break;
    default:
        temp_parity = UART_PARITY_NONE;
        serial_option->parity = parity_none;
        break;
    }

    /* Flow Control */
    if (serial_option->uart_interface == UART_IF_RS232_TTL) {
        // RS232 Hardware Flow Control
        //7     RTS     Request To Send     Output
        //8     CTS     Clear To Send       Input
        switch (serial_option->flow_control) {
        case flow_none:
            uart_set_hw_flow(uart_id[SEG_DATA0_CH], false, false);
            break;
        case flow_rts_cts:
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
            uart_set_hw_flow(uart_id[SEG_DATA0_CH], false, false);
            set_uart_rts_pin_low(uartNum);
#else
            uart_set_hw_flow(uart_id[SEG_DATA0_CH], true, true);
#endif
            break;
        case flow_xon_xoff:
            uart_set_hw_flow(uart_id[SEG_DATA0_CH], false, false);
            break;
        default:
            uart_set_hw_flow(uart_id[SEG_DATA0_CH], false, false);
            serial_option->flow_control = flow_none;
            break;
        }
    }

#ifdef __USE_UART_485_422__
    else { // UART_IF_RS422 || UART_IF_RS485
        uart_set_hw_flow(uart_id[SEG_DATA0_CH], false, false);

        // GPIO configuration (RTS pin -> GPIO: 485SEL)
        if ((serial_option->flow_control != flow_rtsonly) && (serial_option->flow_control != flow_reverserts)) {
            uart_if_mode[SEG_DATA0_CH] = get_uart_rs485_sel(SEG_DATA0_CH);
        } else {
            if (serial_option->flow_control == flow_rtsonly) {
                uart_if_mode[SEG_DATA0_CH] = UART_IF_RS485;
            } else {
                uart_if_mode[SEG_DATA0_CH] = UART_IF_RS485_REVERSE;
            }
        }
        uart_rs485_rs422_init(SEG_DATA0_CH);
        serial_option->uart_interface = uart_if_mode[SEG_DATA0_CH];
    }
    // Set our data format
    uart_set_format(uart_id[SEG_DATA0_CH], temp_data_bits, temp_stop_bits, temp_parity);
    uart_set_fifo_enabled(uart_id[SEG_DATA0_CH], true);
    dma_uart_tx[SEG_DATA0_CH] = dma_claim_unused_channel(true);
    dma_uart_c[SEG_DATA0_CH] = dma_channel_get_default_config(dma_uart_tx[SEG_DATA0_CH]);
    channel_config_set_transfer_data_size(&dma_uart_c[SEG_DATA0_CH], DMA_SIZE_8);
    channel_config_set_dreq(&dma_uart_c[SEG_DATA0_CH], uart_get_dreq(uart_id[SEG_DATA0_CH], true));
    PRT_INFO("serial_option->flow_control = %d\r\n", serial_option->flow_control);
    PRT_INFO("data_bits = %d, stop_bits = %d, parity = %d\r\n", temp_data_bits, temp_stop_bits, temp_parity);
    PRT_INFO("baud = %d\r\n", baud_table[serial_option->baud_rate]);

#endif

}

void DATA_UART_Deinit(void) {
    uart_deinit(DATA0_UART_ID);
    uart_deinit(DATA1_UART_ID);
}

void DATA_UART_Interrupt_Enable(void) {
    uint8_t uart_irq[DEVICE_UART_CNT] = {UART1_IRQ, UART0_IRQ};
    uart_inst_t *uart_id[DEVICE_UART_CNT] = {DATA0_UART_ID, DATA1_UART_ID};

    // Set up a RX interrupt
    irq_set_exclusive_handler(uart_irq[SEG_DATA0_CH], data0_uart_rx);
    irq_set_enabled(uart_irq[SEG_DATA0_CH], true);
    uart_set_irq_enables(uart_id[SEG_DATA0_CH], true, false);
}

void check_uart_flow_control(uint8_t flow_ctrl, int channel) {
    if (flow_ctrl == flow_xon_xoff) {
        if ((xonoff_status == UART_XON) && (get_data_buffer_usedsize(channel) > UART_OFF_THRESHOLD)) { // Send the transmit stop command to peer - go XOFF
            platform_uart_putc(UART_XOFF, channel);
            xonoff_status = UART_XOFF;
#ifdef _UART_DEBUG_
            printf(" >> SEND XOFF [%d / %d]\r\n", get_data_buffer_usedsize(), SEG_DATA_BUF_SIZE);
#endif
        } else if ((xonoff_status == UART_XOFF) && (get_data_buffer_usedsize(channel) < UART_ON_THRESHOLD)) { // Send the transmit start command to peer. -go XON
            platform_uart_putc(UART_XON, channel);
            xonoff_status = UART_XON;
#ifdef _UART_DEBUG_
            printf(" >> SEND XON [%d / %d]\r\n", get_data_buffer_usedsize(), SEG_DATA_BUF_SIZE);
#endif
        }
    }
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
    else if (flow_ctrl == flow_rts_cts) { // RTS pin control
        // Buffer full occurred
        if ((rts_status == UART_RTS_LOW) && (get_data_buffer_usedsize() > UART_OFF_THRESHOLD)) {
            set_uart_rts_pin_high(uartNum);
            rts_status = UART_RTS_HIGH;
#ifdef _UART_DEBUG_
            printf(" >> UART_RTS_HIGH [%d / %d]\r\n", get_data_buffer_usedsize(), SEG_DATA_BUF_SIZE);
#endif
        }

        // Clear the buffer full event
        if ((rts_status == UART_RTS_HIGH) && (get_data_buffer_usedsize() <= UART_OFF_THRESHOLD)) {
            set_uart_rts_pin_low(uartNum);
            rts_status = UART_RTS_LOW;
#ifdef _UART_DEBUG_
            printf(" >> UART_RTS_LOW [%d / %d]\r\n", get_data_buffer_usedsize(), SEG_DATA_BUF_SIZE);
#endif
        }
    }
#endif
}


int32_t platform_uart_putc(uint16_t ch, int channel) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);
    uint8_t c[1];

    if (serial_option->data_bits == word_len8) {
        c[0] = ch & 0x00FF;
    } else if (serial_option->data_bits == word_len7) {
        c[0] = ch & 0x007F; // word_len7
    }
    device_wdt_reset();
    uart_putc(channel ? DATA1_UART_ID : DATA0_UART_ID, c[0]);

    return RET_OK;
}

int32_t platform_uart_puts_dma(uint8_t* buf, uint16_t bytes, int channel) {
    uart_inst_t *uart_id[DEVICE_UART_CNT] = {DATA0_UART_ID, DATA1_UART_ID};

    while (dma_channel_is_busy(dma_uart_tx[channel])) {
        // Wait for the DMA channel to be free
    }
    dma_channel_configure(dma_uart_tx[channel], &dma_uart_c[channel],
                          &uart_get_hw(uart_id[channel])->dr, // write address
                          buf, // read address
                          bytes, // element count (each element is of size transfer_data_size)
                          true); // don't start yet

    //uart_putc(channel ? DATA1_UART_ID : DATA0_UART_ID, c[0]);
    return RET_OK;
}

int32_t platform_uart_puts(uint8_t* buf, uint16_t bytes, int channel) {
    uint32_t i;

    uart_rs485_enable(channel);
    for (i = 0; i < bytes; i++) {
        platform_uart_putc(buf[i], channel);
        device_wdt_reset();
    }
    uart_rs485_disable(channel);

    return bytes;
}

#ifdef __USE_UART_485_422__
uint8_t get_uart_rs485_sel(int channel) {
    GPIO_Configuration(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN, GPIO_IN, IO_PULLUP);// UART0 RTS pin: GPIO / Input
    if (GPIO_Input_Read(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN) == IO_LOW) {
        uart_if_mode[channel] = UART_IF_RS422;
    } else {
        uart_if_mode[channel] = UART_IF_RS485;
    }

    return uart_if_mode[channel];
}

void uart_rs485_rs422_init(int channel) {
    GPIO_Configuration(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN, GPIO_OUT, IO_NOPULL); // UART0 RTS pin: GPIO / Output
    if (uart_if_mode[channel] == UART_IF_RS485) {
        GPIO_Output_Reset(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN);    // UART0 RTS pin init, Set the signal low
    } else {
        GPIO_Output_Set(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN);    // UART0 RTS pin init, Set the signal low
    }
}

void uart_rs485_enable(int channel) {
    if (uart_if_mode[channel] == UART_IF_RS485) {
        GPIO_Output_Set(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN);
    } else if (uart_if_mode[channel] == UART_IF_RS485_REVERSE) {
        GPIO_Output_Reset(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN);
    }
}

void uart_rs485_disable(int channel) {
    if (uart_if_mode[channel] == UART_IF_RS485) {
        uart_tx_wait_blocking(channel ? DATA1_UART_ID : DATA0_UART_ID);
        // RTS pin -> Low;
        GPIO_Output_Reset(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN);

    } else if (uart_if_mode[channel] == UART_IF_RS485_REVERSE) {
        uart_tx_wait_blocking(channel ? DATA1_UART_ID : DATA0_UART_ID);
        // RTS pin -> High
        GPIO_Output_Set(channel ? DATA1_UART_RTS_PIN : DATA0_UART_RTS_PIN);
    }
    //UART_IF_RS422: None
}
#endif

#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__

uint8_t get_uart_cts_pin(void) {
    uint8_t cts_pin = UART_CTS_HIGH;

#ifdef _UART_DEBUG_
    static uint8_t prev_cts_pin;
#endif
    cts_pin = GPIO_Input_Read(DATA0_UART_CTS_PIN);


#ifdef _UART_DEBUG_
    if (cts_pin != prev_cts_pin) {
        printf(" >> UART_CTS_%s\r\n", cts_pin ? "HIGH" : "LOW");
        prev_cts_pin = cts_pin;
    }
#endif

    return cts_pin;
}

void set_uart_rts_pin_high(void) {
    GPIO_Output_Set(DATA0_UART_RTS_PIN);
}

void set_uart_rts_pin_low(void) {
    GPIO_Output_Reset(DATA0_UART_RTS_PIN);
}

#endif

#ifdef UART_PIO_DEBUG
static void debug_uart_init(void) {
    gpio_init(DEBUG_UART_TX_PIN);
    gpio_set_dir(DEBUG_UART_TX_PIN, GPIO_OUT);

    uint offset = pio_add_program(pio0, &uart_tx_program);
    uart_tx_program_init(pio0, 0, offset, DEBUG_UART_TX_PIN, PICO_DEFAULT_UART_BAUD_RATE);
}

static void debug_uart_puts(const char *buf, int len) {
    for (int i = 0; i < len; i++) {
        uart_tx_program_putc(pio0, 0, buf[i]);
    }
}

static struct stdio_driver debug_driver = {
    .out_chars = debug_uart_puts,
    .in_chars = NULL,
};

void debug_uart_enable(void) {
    debug_uart_init();
    stdio_set_driver_enabled(&debug_driver, true);
}
#endif