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
#if (DEVICE_UART_CNT > 2)
#include "uart_rx.pio.h"    // [4Port #12] DATA2/DATA3 PIO UART RX
#include "hardware/pio.h"
#include "hardware/irq.h"
#endif

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private functions prototypes ----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

#if (DEVICE_BOARD_NAME == W232N)
uint32_t baud_table[] = {300, 600, 1200, 1800, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 230400};
#else
uint32_t baud_table[] = {300, 600, 1200, 1800, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 2000000, 4000000, 8000000};
#endif
uint8_t word_len_table[] = {7, 8, 9};
uint8_t * parity_table[] = {(uint8_t *)"N", (uint8_t *)"ODD", (uint8_t *)"EVEN"};
uint8_t stop_bit_table[] = {1, 2};
uint8_t * flow_ctrl_table[] = {(uint8_t *)"NONE", (uint8_t *)"XON/XOFF", (uint8_t *)"RTS/CTS", (uint8_t *)"RTS Only", (uint8_t *)"RTS Only Reverse", (uint8_t *)"DTR/DSR"};
uint8_t * uart_if_table[] = {(uint8_t *)UART_IF_STR_RS232_TTL, (uint8_t *)UART_IF_STR_RS422, (uint8_t *)UART_IF_STR_RS485, (uint8_t *)UART_IF_STR_RS485};

// XON/XOFF Status;
static uint8_t xonoff_status[DEVICE_UART_CNT];

// UART Interface selector; RS-422 or RS-485 use only
static uint8_t uart_if_mode[DEVICE_UART_CNT] = {UART_IF_RS422, UART_IF_RS422};

extern xSemaphoreHandle seg_u2e_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle segcp_uart_sem;

uint dma_uart_tx[DEVICE_UART_CNT];
dma_channel_config dma_uart_c[DEVICE_UART_CNT];

static const uint data_uart_rts_pin[DEVICE_UART_CNT] = {
    DATA0_UART_RTS_PIN,
    DATA1_UART_RTS_PIN,
#if (DEVICE_UART_CNT > 2)
    DATA2_UART_RTS_PIN,
#endif
#if (DEVICE_UART_CNT > 3)
    DATA3_UART_RTS_PIN,
#endif
};
static uint8_t data_uart_rts_status[DEVICE_UART_CNT];
static uint8_t data_uart_dtr_status[DEVICE_UART_CNT];

#if (DEVICE_UART_CNT > 2)
/* DATA2/DATA3 PIO UART on pio0 (pio1 is the W5500 SPI).                       */
/* Fixed 8N1 (data/stop/parity config ignored); TTL only (no RS-485 yet).      */
#define PIO_DATA_UART   pio0

// TX/RX GPIOs per PIO channel (indices >= UART_HW_CH_CNT)
static const uint pio_uart_tx_pin[DEVICE_UART_CNT] = {
    0, 0,
    DATA2_UART_TX_PIN,
#if (DEVICE_UART_CNT > 3)
    DATA3_UART_TX_PIN,
#endif
};
static const uint pio_uart_rx_pin[DEVICE_UART_CNT] = {
    0, 0,
    DATA2_UART_RX_PIN,
#if (DEVICE_UART_CNT > 3)
    DATA3_UART_RX_PIN,
#endif
};
static const uint pio_uart_cts_pin[DEVICE_UART_CNT] = {
    0, 0,
    DATA2_UART_CTS_PIN,
#if (DEVICE_UART_CNT > 3)
    DATA3_UART_CTS_PIN,
#endif
};
static uint pio_tx_sm[DEVICE_UART_CNT];
static uint pio_rx_sm[DEVICE_UART_CNT];

static uint32_t pio_uart_baud(int channel) {
    struct __serial_option *so = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);
    if (so->baud_rate < (sizeof(baud_table) / sizeof(baud_table[0]))) {
        return baud_table[so->baud_rate];
    }
    return baud_table[baud_115200];
}

// Load the TX/RX PIO programs and initialise a TX+RX state machine per PIO channel.
static void pio_data_uart_init_sm(void) {
    uint tx_offset = pio_add_program(PIO_DATA_UART, &uart_tx_program);
    uint tx_cts_offset = pio_add_program(PIO_DATA_UART, &uart_tx_cts_program);
    uint rx_offset = pio_add_program(PIO_DATA_UART, &uart_rx_program);

    for (int i = UART_HW_CH_CNT; i < DEVICE_UART_CNT; i++) {
        struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[i]);
        uint8_t use_cts = (serial_option->flow_control == flow_rts_cts);
        uint8_t use_dtr_dsr = (serial_option->flow_control == flow_dtr_dsr);
        uint32_t baud = pio_uart_baud(i);

        // The CTS/RTS pins double as DSR/DTR; both are plain GPIO here, so only
        // the pull and the direction of the input side differ.
        gpio_init(pio_uart_cts_pin[i]);
        gpio_set_dir(pio_uart_cts_pin[i], GPIO_IN);
        gpio_pull_up(pio_uart_cts_pin[i]);

        gpio_init(data_uart_rts_pin[i]);
        gpio_set_dir(data_uart_rts_pin[i], GPIO_OUT);
        gpio_put(data_uart_rts_pin[i], UART_RTS_LOW);
        data_uart_rts_status[i] = UART_RTS_LOW;
        data_uart_dtr_status[i] = UART_RTS_LOW;

        pio_tx_sm[i] = pio_claim_unused_sm(PIO_DATA_UART, true);
        if (use_cts) {
            // The SM samples CTS before each byte, so queued bytes stop at the
            // next byte boundary just as the HW UART's CTS gating does.
            uart_tx_cts_program_init(PIO_DATA_UART, pio_tx_sm[i], tx_cts_offset,
                                     pio_uart_tx_pin[i], pio_uart_cts_pin[i], baud);
        } else {
            uart_tx_program_init(PIO_DATA_UART, pio_tx_sm[i], tx_offset, pio_uart_tx_pin[i], baud);
        }
        pio_rx_sm[i] = pio_claim_unused_sm(PIO_DATA_UART, true);
        uart_rx_program_init(PIO_DATA_UART, pio_rx_sm[i], rx_offset, pio_uart_rx_pin[i], baud);
        PRT_INFO("PIO UART ch%d: TX GP%d(sm%d) RX GP%d(sm%d) baud %d flow %s\r\n",
                 i, pio_uart_tx_pin[i], pio_tx_sm[i], pio_uart_rx_pin[i], pio_rx_sm[i], (int)baud,
                 use_cts ? "rts/cts" : (use_dtr_dsr ? "dtr/dsr" : "none"));
    }
}

// PIO RX FIFO-not-empty ISR. Drains each PIO channel into its ring buffer.
// Mirrors data1_uart_rx: no mode-switch trigger (AT mode is DATA0 only).
static void pio_data_uart_rx_isr(void) {
    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    for (int i = UART_HW_CH_CNT; i < DEVICE_UART_CNT; i++) {
        uint8_t input_flag = 0;
        while (!pio_sm_is_rx_fifo_empty(PIO_DATA_UART, pio_rx_sm[i])) {
            uint8_t ch = (uint8_t)(*((io_rw_8 *)&PIO_DATA_UART->rxf[pio_rx_sm[i]] + 3));
            if (is_data_buffer_full(i) == TRUE) {
                data_buffer_flush(i);
            }
            if (check_serial_store_permitted(ch, i)) {
                put_byte_to_data_buffer(ch, i);
                input_flag = 1;
            }
        }
        if (input_flag) {
            init_time_delimiter_timer(i);
            if (opmode == DEVICE_GW_MODE) {
                xSemaphoreGiveFromISR(seg_u2e_sem[i], &xHigherPriorityTaskWoken);
            }
        }
    }
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static void pio_data_uart_irq_enable(void) {
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_data_uart_rx_isr);
    for (int i = UART_HW_CH_CNT; i < DEVICE_UART_CNT; i++) {
        pio_set_irqn_source_enabled(PIO_DATA_UART, 0,
                                    (enum pio_interrupt_source)(pis_sm0_rx_fifo_not_empty + pio_rx_sm[i]), true);
    }
    irq_set_enabled(PIO0_IRQ_0, true);
}

// TX one byte on a PIO channel (blocks only if the 8-deep TX FIFO is full).
// With CTS handshaking the SM stalls until the peer is ready, so the FIFO can
// stay full indefinitely; feed the watchdog while waiting for room.
static void pio_data_uart_putc(int channel, uint8_t c) {
    while (pio_sm_is_tx_fifo_full(PIO_DATA_UART, pio_tx_sm[channel])) {
        device_wdt_reset();
    }
    uart_tx_program_putc(PIO_DATA_UART, pio_tx_sm[channel], (char)c);
}
#endif  // DEVICE_UART_CNT > 2

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
        if (opmode == DEVICE_GW_MODE) {
            xSemaphoreGiveFromISR(seg_u2e_sem[SEG_DATA0_CH], &xHigherPriorityTaskWoken);
        } else if (opmode == DEVICE_AT_MODE) {
            xSemaphoreGiveFromISR(segcp_uart_sem, &xHigherPriorityTaskWoken);
        }
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
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
    // Only HW UART instances are valid; PIO channels are zero and skipped below.
    uart_inst_t *uart_id[DEVICE_UART_CNT] = {DATA0_UART_ID, DATA1_UART_ID};

    for (int i = 0; i < DEVICE_UART_CNT; i++) {
        xonoff_status[i] = UART_XON;
    }

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select

    gpio_init(DATA0_UART_TX_PIN);
    gpio_init(DATA0_UART_RX_PIN);
    gpio_init(DATA0_UART_CTS_PIN);
    gpio_init(DATA0_UART_RTS_PIN);

    gpio_init(DATA1_UART_TX_PIN);
    gpio_init(DATA1_UART_RX_PIN);
    gpio_init(DATA1_UART_CTS_PIN);
    gpio_init(DATA1_UART_RTS_PIN);

    gpio_set_function(DATA0_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_CTS_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA0_UART_RTS_PIN, GPIO_FUNC_UART);
    gpio_pull_up(DATA0_UART_RX_PIN);

    gpio_set_function(DATA1_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA1_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA1_UART_CTS_PIN, GPIO_FUNC_UART);
    gpio_set_function(DATA1_UART_RTS_PIN, GPIO_FUNC_UART);
    gpio_pull_up(DATA1_UART_RX_PIN);

    for (int i = 0; i < DEVICE_UART_CNT; i++) {
        if (i >= UART_HW_CH_CNT) {
            continue;   // DATA2/DATA3 are PIO UARTs, configured separately
        }
        serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[i]);
        // Deinitialize UART
        uart_deinit(uart_id[i]);

        // Set up our UART with a basic baud rate.
        uart_init(uart_id[i], 2400);

        /* Set Baud Rate */
        if (serial_option->baud_rate < (sizeof(baud_table) / sizeof(baud_table[0]))) {
            PRT_INFO("Real baudrate = %d\r\n", uart_set_baudrate(uart_id[i], baud_table[serial_option->baud_rate]));
            valid_arg = 1;
        }

        if (!valid_arg) {
            PRT_INFO("Real baudrate = %d\r\n", uart_set_baudrate(uart_id[i], baud_table[baud_115200]));
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
                uart_set_hw_flow(uart_id[i], false, false);
                break;
            case flow_rts_cts:
                // CTS is handled by the UART peripheral. RTS reflects the
                // gateway receive-buffer state, so keep it under GPIO control.
                uart_set_hw_flow(uart_id[i], true, false);
                GPIO_Configuration(data_uart_rts_pin[i], IO_OUTPUT, IO_NOPULL);
                GPIO_Output_Reset(data_uart_rts_pin[i]);
                data_uart_rts_status[i] = UART_RTS_LOW;
                break;
            case flow_xon_xoff:
                uart_set_hw_flow(uart_id[i], false, false);
                break;
            case flow_dtr_dsr:
                // DTR/DSR reuse the RTS/CTS pins as plain GPIO. RP2040 has no
                // hardware DTR/DSR, so both directions are driven in software.
                uart_set_hw_flow(uart_id[i], false, false);
                init_flowcontrol_dtr_pin(i);
                init_flowcontrol_dsr_pin(i);
                data_uart_dtr_status[i] = UART_RTS_LOW;
                break;
            default:
                uart_set_hw_flow(uart_id[i], false, false);
                serial_option->flow_control = flow_none;
                break;
            }
        }

#ifdef __USE_UART_485_422__
        else { // UART_IF_RS422 || UART_IF_RS485
            uart_set_hw_flow(uart_id[i], false, false);

            // GPIO configuration (RTS pin -> GPIO: 485SEL)
            if ((serial_option->flow_control != flow_rtsonly) && (serial_option->flow_control != flow_reverserts)) {
                // [Disabled 2026-07] HW RTS select-pin read — 422/485 now chosen via AT command (uart_interface).
                //uart_if_mode[i] = get_uart_rs485_sel(i);
                uart_if_mode[i] = serial_option->uart_interface; // follow AT-command selected interface
            } else {
                if (serial_option->flow_control == flow_rtsonly) {
                    uart_if_mode[i] = UART_IF_RS485;
                } else {
                    uart_if_mode[i] = UART_IF_RS485_REVERSE;
                }
            }
            uart_rs485_rs422_init(i);
            // [Disabled 2026-07] write-back of pin-derived mode — keep AT-command value authoritative.
            //serial_option->uart_interface = uart_if_mode[i];
        }
        // Set our data format
        uart_set_format(uart_id[i], temp_data_bits, temp_stop_bits, temp_parity);
        uart_set_fifo_enabled(uart_id[i], true);

        dma_uart_tx[i] = dma_claim_unused_channel(true);
        dma_uart_c[i] = dma_channel_get_default_config(dma_uart_tx[i]);
        channel_config_set_transfer_data_size(&dma_uart_c[i], DMA_SIZE_8);
        channel_config_set_dreq(&dma_uart_c[i], uart_get_dreq(uart_id[i], true));

        PRT_INFO("serial_option->flow_control = %d\r\n", serial_option->flow_control);
        PRT_INFO("data_bits = %d, stop_bits = %d, parity = %d\r\n", temp_data_bits, temp_stop_bits, temp_parity);
        PRT_INFO("baud = %d\r\n", baud_table[serial_option->baud_rate]);

#endif
    }

#if (DEVICE_UART_CNT > 2)
    pio_data_uart_init_sm();   // DATA2/DATA3 PIO UART (RX IRQ enabled in DATA_UART_Interrupt_Enable)
#endif
}

void DATA_UART_Deinit(void) {
    uart_deinit(DATA0_UART_ID);
    uart_deinit(DATA1_UART_ID);
}

void DATA_UART_Interrupt_Enable(void) {
    uint8_t uart_irq[DEVICE_UART_CNT] = {UART1_IRQ, UART0_IRQ};
    uart_inst_t *uart_id[DEVICE_UART_CNT] = {DATA0_UART_ID, DATA1_UART_ID};

    for (int i = 0; i < UART_HW_CH_CNT; i++) {
        // Set up a RX interrupt
        irq_set_exclusive_handler(uart_irq[i], i ? data1_uart_rx : data0_uart_rx);
        irq_set_enabled(uart_irq[i], true);
        uart_set_irq_enables(uart_id[i], true, false);
    }
#if (DEVICE_UART_CNT > 2)
    pio_data_uart_irq_enable();
#endif
}

void check_uart_flow_control(uint8_t flow_ctrl, int channel) {
    if (flow_ctrl == flow_xon_xoff) {
        if ((xonoff_status[channel] == UART_XON) && (get_data_buffer_usedsize(channel) > UART_OFF_THRESHOLD)) { // Send the transmit stop command to peer - go XOFF
            platform_uart_putc(UART_XOFF, channel);
            xonoff_status[channel] = UART_XOFF;
#ifdef _UART_DEBUG_
            printf(" >> SEND XOFF [%d / %d]\r\n", get_data_buffer_usedsize(channel), SEG_DATA_BUF_SIZE);
#endif
        } else if ((xonoff_status[channel] == UART_XOFF) && (get_data_buffer_usedsize(channel) < UART_ON_THRESHOLD)) { // Send the transmit start command to peer. -go XON
            platform_uart_putc(UART_XON, channel);
            xonoff_status[channel] = UART_XON;
#ifdef _UART_DEBUG_
            printf(" >> SEND XON [%d / %d]\r\n", get_data_buffer_usedsize(channel), SEG_DATA_BUF_SIZE);
#endif
        }
    } else if (flow_ctrl == flow_rts_cts) {
        uint16_t used_size = get_data_buffer_usedsize(channel);

        if ((data_uart_rts_status[channel] == UART_RTS_LOW) && (used_size > UART_OFF_THRESHOLD)) {
            gpio_put(data_uart_rts_pin[channel], UART_RTS_HIGH);
            data_uart_rts_status[channel] = UART_RTS_HIGH;
        } else if ((data_uart_rts_status[channel] == UART_RTS_HIGH) && (used_size <= UART_ON_THRESHOLD)) {
            gpio_put(data_uart_rts_pin[channel], UART_RTS_LOW);
            data_uart_rts_status[channel] = UART_RTS_LOW;
        }
    } else if (flow_ctrl == flow_dtr_dsr) {
        // DTR mirrors RTS: deassert while our receive buffer is filling up.
        uint16_t used_size = get_data_buffer_usedsize(channel);

        if ((data_uart_dtr_status[channel] == UART_RTS_LOW) && (used_size > UART_OFF_THRESHOLD)) {
            set_flowcontrol_dtr_pin(ON, channel);
            data_uart_dtr_status[channel] = UART_RTS_HIGH;
        } else if ((data_uart_dtr_status[channel] == UART_RTS_HIGH) && (used_size <= UART_ON_THRESHOLD)) {
            set_flowcontrol_dtr_pin(OFF, channel);
            data_uart_dtr_status[channel] = UART_RTS_LOW;
        }
    }
}

uint8_t platform_uart_cts_ready(int channel) {
#if (DEVICE_UART_CNT > 2)
    if (channel >= UART_HW_CH_CNT) {
        return gpio_get(pio_uart_cts_pin[channel]) == UART_CTS_LOW;
    }
#endif
    // DATA0/DATA1 CTS gating is handled by the RP2040 UART peripheral.
    return 1;
}


int32_t platform_uart_putc(uint16_t ch, int channel) {
    if (channel >= UART_HW_CH_CNT) {
#if (DEVICE_UART_CNT > 2)
        device_wdt_reset();
        pio_data_uart_putc(channel, (uint8_t)(ch & 0xFF));
#endif
        return RET_OK;
    }
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

// Block until a channel's TX DMA has finished reading its source buffer.
// Callers must do this before overwriting the buffer they last handed to
// platform_uart_puts_dma(), which starts the transfer and returns immediately.
void platform_uart_tx_wait(int channel) {
    if (channel >= UART_HW_CH_CNT) {
        return;     // PIO transmit is synchronous
    }
    while (dma_channel_is_busy(dma_uart_tx[channel])) {
        device_wdt_reset();
    }
}

int32_t platform_uart_puts_dma(uint8_t* buf, uint16_t bytes, int channel) {
    uart_inst_t *uart_id[DEVICE_UART_CNT] = {DATA0_UART_ID, DATA1_UART_ID};

    if (channel >= UART_HW_CH_CNT) {
        // PIO UART has no DMA TX path; use the blocking PIO transmit path.
        return platform_uart_puts(buf, bytes, channel);
    }
    platform_uart_tx_wait(channel);
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

    if (channel >= UART_HW_CH_CNT) {
#if (DEVICE_UART_CNT > 2)
        for (i = 0; i < bytes; i++) {   // PIO UART TX (TTL 8N1; RS-485 direction TODO)
            pio_data_uart_putc(channel, buf[i]);
            device_wdt_reset();
        }
#endif
        return bytes;
    }
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
    GPIO_Configuration(data_uart_rts_pin[channel], GPIO_IN, IO_PULLUP);// UART0 RTS pin: GPIO / Input
    if (GPIO_Input_Read(data_uart_rts_pin[channel]) == IO_LOW) {
        uart_if_mode[channel] = UART_IF_RS422;
    } else {
        uart_if_mode[channel] = UART_IF_RS485;
    }

    return uart_if_mode[channel];
}

void uart_rs485_rs422_init(int channel) {
    GPIO_Configuration(data_uart_rts_pin[channel], GPIO_OUT, IO_NOPULL); // UART0 RTS pin: GPIO / Output
    if (uart_if_mode[channel] == UART_IF_RS485) {
        GPIO_Output_Reset(data_uart_rts_pin[channel]);    // UART0 RTS pin init, Set the signal low
    } else {
        GPIO_Output_Set(data_uart_rts_pin[channel]);    // UART0 RTS pin init, Set the signal low
    }
}

void uart_rs485_enable(int channel) {
    // PIO channels are TTL only; they have no direction control to drive and
    // must not touch another channel's RTS pin.
    if (channel >= UART_HW_CH_CNT) {
        return;
    }
    if (uart_if_mode[channel] == UART_IF_RS485) {
        GPIO_Output_Set(data_uart_rts_pin[channel]);
    } else if (uart_if_mode[channel] == UART_IF_RS485_REVERSE) {
        GPIO_Output_Reset(data_uart_rts_pin[channel]);
    }
}

void uart_rs485_disable(int channel) {
    if (channel >= UART_HW_CH_CNT) {
        return;
    }
    if (uart_if_mode[channel] == UART_IF_RS485) {
        uart_tx_wait_blocking(channel ? DATA1_UART_ID : DATA0_UART_ID);
        // RTS pin -> Low;
        GPIO_Output_Reset(data_uart_rts_pin[channel]);

    } else if (uart_if_mode[channel] == UART_IF_RS485_REVERSE) {
        uart_tx_wait_blocking(channel ? DATA1_UART_ID : DATA0_UART_ID);
        // RTS pin -> High
        GPIO_Output_Set(data_uart_rts_pin[channel]);
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
