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
// Had only two initialisers while the array was DEVICE_UART_CNT long, so DATA2 and
// DATA3 silently defaulted to 0 (TTL) instead of the intended RS-422.
//
// Original:
//     static uint8_t uart_if_mode[DEVICE_UART_CNT] = {UART_IF_RS422, UART_IF_RS422};
static uint8_t uart_if_mode[DEVICE_UART_CNT] = {
    UART_IF_RS422, UART_IF_RS422,
#if (DEVICE_UART_CNT > 2)
    UART_IF_RS422,
#endif
#if (DEVICE_UART_CNT > 3)
    UART_IF_RS422,
#endif
};

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

// Yield while spinning on a transmit path so the other channels' tasks can run.
// Without any yield a channel waiting on flow control burns its whole time slice,
// and under four-channel load that starved the S2E direction of the other channels
// (one channel took 43 kB/s while the rest got under 4 kB/s).
//
// A plain taskYIELD() is not enough on its own, though: it only ever hands the CPU
// to tasks of equal or higher priority. Every
// data-path task ends up at priority 31 (configMAX_PRIORITIES is 32, so xTaskCreate
// clamps the 40..52 values down), while seg_ch_task sits alone at 18 - so a channel
// spinning here can keep eight peers fed and never let 18 run again. seg_ch_task
// holds seg_critical_sem across do_seg() and is what reasserts RTS at the end of it,
// so the channel wedges: ring buffer full, RTS deasserted for good, S2E dead.
//
// Yield cheaply while a transfer is merely in progress, and once the wait has
// clearly stopped being transient, sleep a tick so lower priorities get in.
//
// The bound is elapsed time, not a spin count: a full 2 kB chunk at 460,800 bps
// takes about 44 ms, and a spin count large enough to cover that is impossible to
// pick (each taskYIELD() costs microseconds, so a healthy transfer easily runs to
// thousands of iterations). 200 ms clears any legitimate chunk at this baud while
// still being reached almost immediately by a transfer the peer has frozen with CTS.
//
// This is preferred over raising seg_ch_task's priority - that removed the stall but
// made it contend for seg_critical_sem with the drain task, causing ring overflow and
// lost frames - and over returning early from ether_to_uart(), which emptied the
// transmit pipe and cost about 12 % of E2S throughput.
#define UART_TX_SPIN_YIELD_MS 200U

// Waiting out a transmit DMA is different from waiting for FIFO space. The DMA is
// pure hardware time - up to 44 ms for a 2 kB chunk at 460,800 bps - and the task has
// nothing to do until it lands, so yielding through all of it keeps a priority 31 task
// runnable for the whole transfer. Four channels doing that fill both cores, and
// seg_ch_task at 18 stops being scheduled; because it holds seg_critical_sem across
// do_seg(), the drain task blocks behind it and the channel stops outright. Sleeping
// costs at most one tick per chunk against 44 ms of wire time, and the wire is the
// limit here, not the task. The short window keeps small transfers, which finish well
// inside a tick, from paying that tick.
#define UART_TX_DMA_YIELD_MS 2U

static inline void uart_tx_spin_wait_bounded(uint32_t *started_ms, uint32_t yield_ms) {
    device_wdt_reset();

    // These paths also run before the scheduler starts.
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return;
    }

    if (*started_ms == 0) {
        *started_ms = (uint32_t)millis();
        // millis() can legitimately be 0 at boot; 1 is close enough and keeps the
        // "not started yet" sentinel usable.
        if (*started_ms == 0) {
            *started_ms = 1;
        }
    }

    if (((uint32_t)millis() - *started_ms) < yield_ms) {
        taskYIELD();
    } else {
        vTaskDelay(1);
    }
}

static inline void uart_tx_spin_wait_until(uint32_t *started_ms) {
    uart_tx_spin_wait_bounded(started_ms, UART_TX_SPIN_YIELD_MS);
}

// Stop the peer from the RX ISR as soon as the ring buffer crosses its
// high-water mark. check_uart_flow_control() runs only from the seg task, which
// leaves just SEG_DATA_BUF_SIZE - UART_OFF_THRESHOLD bytes of slack (409 B, about
// 8.9 ms at 460800 bps). A task busy in the E2S path overruns that easily, and
// the overflow used to be discarded silently. Reassertion stays in
// check_uart_flow_control(): only the task drains the buffer, so only the task
// needs to decide when it is safe to resume.
// XON/XOFF is not handled here because it has to transmit a byte.
static void uart_rx_flow_gate(int channel) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);

    if (get_data_buffer_usedsize(channel) <= UART_OFF_THRESHOLD) {
        return;
    }

    if (serial_option->flow_control == flow_rts_cts) {
        if (data_uart_rts_status[channel] == UART_RTS_LOW) {
            gpio_put(data_uart_rts_pin[channel], UART_RTS_HIGH);
            data_uart_rts_status[channel] = UART_RTS_HIGH;
        }
    } else if (serial_option->flow_control == flow_dtr_dsr) {
        if (data_uart_dtr_status[channel] == UART_RTS_LOW) {
            set_flowcontrol_dtr_pin(ON, channel);
            data_uart_dtr_status[channel] = UART_RTS_HIGH;
        }
    }
}

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

// Serial parameters, resolved once at init and cached for the TX/RX paths.
// PIO has no parity hardware, so parity is split across CPU and state machine:
// the CPU appends the parity bit as the top payload bit on TX, and strips and
// verifies it on RX. The state machine only ever shifts payload_bits.
static uint8_t pio_uart_data_bits[DEVICE_UART_CNT];
static uint8_t pio_uart_parity[DEVICE_UART_CNT];
static uint8_t pio_uart_payload_bits[DEVICE_UART_CNT];
static uint8_t pio_uart_stop_bits[DEVICE_UART_CNT];
static uint8_t pio_uart_de_mode[DEVICE_UART_CNT];   // RS-422/485 owns the RTS pin
static volatile uint32_t pio_uart_framing_err[DEVICE_UART_CNT];
static volatile uint32_t pio_uart_parity_err[DEVICE_UART_CNT];

// RX-not-empty generated almost one CPU interrupt per PIO byte: DATA2/DATA3 at
// 460800 baud therefore consumed roughly 90,000 IRQs/s.  Since PIO0_IRQ_0 has a
// lower IRQ number than UART0/1, it starved the two PL011 FIFOs. Raising the HW
// IRQ priority merely moved the loss to DATA3.  Drain PIO RX with DMA instead;
// a small task publishes accumulated words to the existing UART ring in batches.
#define PIO_UART_RX_DMA_WORDS 1024U
#define PIO_UART_RX_DMA_MASK  (PIO_UART_RX_DMA_WORDS - 1U)
#define PIO_UART_RX_BATCH_MAX 64U
#define PIO_UART_RX_DMA_RING_BITS 12U  // 1024 x uint32_t = 4096-byte address ring

// The address ring wraps the write pointer forever, but the transfer count does
// not: the channel halts once it reaches zero, which at 460800 baud arrives
// after roughly 27 hours of continuous traffic and silently kills DATA2/DATA3
// reception until the next reset.  Re-arm from the consumer task while the
// count is still comfortably above zero.  The channel is only stopped for the
// few register writes that restart it, which the 8-word PIO FIFO (174 us at
// 460800 baud) absorbs, so no byte is lost.
#define PIO_UART_RX_DMA_REARM_LEFT (1U << 20)

static uint pio_uart_rx_dma[DEVICE_UART_CNT];
static uint16_t pio_uart_rx_dma_read[DEVICE_UART_CNT];
static uint32_t pio_uart_rx_dma_ring[DEVICE_UART_CNT - UART_HW_CH_CNT][PIO_UART_RX_DMA_WORDS]
__attribute__((aligned(1U << PIO_UART_RX_DMA_RING_BITS)));
static TaskHandle_t pio_uart_rx_task_handle;
static volatile uint32_t pio_uart_rx_task_heartbeat;
static volatile uint32_t pio_uart_rx_consumed[DEVICE_UART_CNT];
static volatile uint32_t pio_uart_rx_stored[DEVICE_UART_CNT];
static volatile uint32_t pio_uart_rx_dropped[DEVICE_UART_CNT];
static volatile uint32_t pio_uart_rx_rearm[DEVICE_UART_CNT];

static uint8_t pio_uart_resolve_data_bits(uint8_t cfg) {
    switch (cfg) {
    case word_len7:
        return 7;
    case word_len9:
        return 9;
    case word_len8:
    default:
        return 8;
    }
}

// stop_bit1 / stop_bit2 are the only values the enum carries; 1.5 stop bits are
// named in the config comment but have no encoding, so anything else means 1.
static uint8_t pio_uart_resolve_stop_bits(uint8_t cfg) {
    return (cfg == stop_bit2) ? 2 : 1;
}

// Parity over the data bits only. Even parity makes the count of ones even; odd
// parity inverts that.
static uint32_t pio_uart_parity_bit(int channel, uint32_t data) {
    uint32_t v = data & ((1u << pio_uart_data_bits[channel]) - 1u);
    uint32_t p = 0;

    while (v) {
        p ^= (v & 1u);
        v >>= 1;
    }

    return (pio_uart_parity[channel] == parity_odd) ? (p ^ 1u) : p;
}

// Time for one complete frame to leave the wire, used before dropping the RS-485
// driver enable. Start bit + payload + stop bits, rounded up.
static uint32_t pio_uart_frame_us(int channel, uint32_t baud) {
    uint32_t bits = 1u + pio_uart_payload_bits[channel] + pio_uart_stop_bits[channel];

    if (baud == 0) {
        return 0;
    }

    return ((bits * 1000000u) + baud - 1u) / baud;
}

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
        uint32_t baud = pio_uart_baud(i);
        uint8_t data_bits = pio_uart_resolve_data_bits(serial_option->data_bits);
        uint8_t stop_bits = pio_uart_resolve_stop_bits(serial_option->stop_bits);
        uint8_t parity = serial_option->parity;
        // RS-422/485 drives the direction control on the RTS pin, so RTS/CTS flow
        // control cannot coexist with it - the same trade-off the HW channels make.
        uint8_t use_de = (serial_option->uart_interface != UART_IF_RS232_TTL);
        uint8_t use_cts = (serial_option->flow_control == flow_rts_cts) && !use_de;
        uint8_t use_dtr_dsr = (serial_option->flow_control == flow_dtr_dsr) && !use_de;

        pio_uart_data_bits[i] = data_bits;
        pio_uart_parity[i] = parity;
        pio_uart_payload_bits[i] = data_bits + ((parity != parity_none) ? 1 : 0);
        pio_uart_stop_bits[i] = stop_bits;
        pio_uart_de_mode[i] = use_de;

        // DATA_UART_Configuration() only resolves uart_if_mode for the HW channels,
        // so without this the PIO channels would keep the array's default and never
        // toggle direction however the interface is configured.
        uart_if_mode[i] = serial_option->uart_interface;

        // The CTS/RTS pins double as DSR/DTR; both are plain GPIO here, so only
        // the pull and the direction of the input side differ.
        gpio_init(pio_uart_cts_pin[i]);
        gpio_set_dir(pio_uart_cts_pin[i], GPIO_IN);
        gpio_pull_up(pio_uart_cts_pin[i]);

        if (use_de) {
            // Direction control instead of RTS. Idle level per interface, same as
            // the HW channels.
            uart_rs485_rs422_init(i);
            data_uart_rts_status[i] = UART_RTS_LOW;
            data_uart_dtr_status[i] = UART_RTS_LOW;
        } else {
            gpio_init(data_uart_rts_pin[i]);
            gpio_set_dir(data_uart_rts_pin[i], GPIO_OUT);
            gpio_put(data_uart_rts_pin[i], UART_RTS_LOW);
            data_uart_rts_status[i] = UART_RTS_LOW;
            data_uart_dtr_status[i] = UART_RTS_LOW;
        }

        pio_tx_sm[i] = pio_claim_unused_sm(PIO_DATA_UART, true);
        if (use_cts) {
            // The SM samples CTS before each byte, so queued bytes stop at the
            // next byte boundary just as the HW UART's CTS gating does.
            uart_tx_cts_program_init_fmt(PIO_DATA_UART, pio_tx_sm[i], tx_cts_offset,
                                         pio_uart_tx_pin[i], pio_uart_cts_pin[i], baud,
                                         data_bits, parity != parity_none, stop_bits);
        } else {
            uart_tx_program_init_fmt(PIO_DATA_UART, pio_tx_sm[i], tx_offset, pio_uart_tx_pin[i], baud,
                                     data_bits, parity != parity_none, stop_bits);
        }
        // TX DMA, paced by the SM's TX-FIFO DREQ. Byte-sized transfers land in
        // the low byte of TXF, which is what the program consumes (OUT shifts
        // right, so the low 8 bits of the 32-bit OSR are the ones sent).
        // With CTS handshaking a stalled SM stops draining its FIFO, the DREQ
        // stops asserting and the DMA halts on its own - back-pressure reaches
        // the DMA in hardware, without the CPU spinning for it.
        dma_uart_tx[i] = dma_claim_unused_channel(true);
        dma_uart_c[i] = dma_channel_get_default_config(dma_uart_tx[i]);
        channel_config_set_transfer_data_size(&dma_uart_c[i], DMA_SIZE_8);
        channel_config_set_dreq(&dma_uart_c[i], pio_get_dreq(PIO_DATA_UART, pio_tx_sm[i], true));

        pio_rx_sm[i] = pio_claim_unused_sm(PIO_DATA_UART, true);
        uart_rx_program_init_fmt(PIO_DATA_UART, pio_rx_sm[i], rx_offset, pio_uart_rx_pin[i], baud,
                                 data_bits, parity != parity_none);
        PRT_INFO("PIO UART ch%d: TX GP%d(sm%d) RX GP%d(sm%d) baud %d %d%c%d flow %s%s\r\n",
                 i, pio_uart_tx_pin[i], pio_tx_sm[i], pio_uart_rx_pin[i], pio_rx_sm[i], (int)baud,
                 data_bits,
                 (parity == parity_odd) ? 'O' : ((parity == parity_even) ? 'E' : 'N'),
                 stop_bits,
                 use_cts ? "rts/cts" : (use_dtr_dsr ? "dtr/dsr" : "none"),
                 use_de ? " (RS-422/485 DE on RTS pin)" : "");
    }
}

static uint16_t pio_uart_store_batch(const uint8_t *data, uint16_t size, int channel) {
    struct __serial_option *serial_option =
        (struct __serial_option *)&get_DevConfig_pointer()->serial_option[channel];

    if (serial_option->flow_control != flow_xon_xoff) {
        if (!check_serial_store_permitted(0, channel)) {
            return 0;
        }
        return put_bytes_to_data_buffer(data, size, channel);
    }

    // XON/XOFF control bytes must still be interpreted individually.
    uint16_t stored = 0;
    for (uint16_t n = 0; n < size; n++) {
        if (check_serial_store_permitted(data[n], channel)) {
            put_byte_to_data_buffer(data[n], channel);
            stored++;
        }
    }
    return stored;
}

static uint8_t pio_uart_decode_rx_word(int channel, uint32_t fifo_word) {
    if (pio_uart_parity[channel] == parity_none) {
        return (uint8_t)(fifo_word >> 24);
    }

    uint8_t data_bits = pio_uart_data_bits[channel];
    uint32_t payload = uart_rx_program_extract(fifo_word, pio_uart_payload_bits[channel]);
    uint8_t ch = (uint8_t)(payload & ((1u << data_bits) - 1u));
    uint32_t received_parity = (payload >> data_bits) & 1u;

    if (received_parity != pio_uart_parity_bit(channel, ch)) {
        pio_uart_parity_err[channel]++;
    }
    return ch;
}

static void pio_data_uart_rx_dma_rearm(int channel);

static void pio_data_uart_rx_task(void *argument) {
    (void)argument;

    for (;;) {
        // One beat represents a complete attempt to service both DATA2 and DATA3.
        // If this stops while the rest of the product keeps running, the shared
        // consumer task itself (or the core executing it) has stopped.
        pio_uart_rx_task_heartbeat++;
        seg_postmortem_mark(SEG_PM_TASK_PIO_RX, 0);

        for (int channel = UART_HW_CH_CNT; channel < DEVICE_UART_CNT; channel++) {
            uint ring_row = (uint)(channel - UART_HW_CH_CNT);
            uintptr_t ring_base = (uintptr_t)&pio_uart_rx_dma_ring[ring_row][0];
            uintptr_t write_addr;
            uint16_t write_index;
            uint8_t input_flag = 0;

            if (dma_channel_hw_addr(pio_uart_rx_dma[channel])->transfer_count <
                    PIO_UART_RX_DMA_REARM_LEFT) {
                pio_data_uart_rx_dma_rearm(channel);
            }

            write_addr =
                (uintptr_t)dma_channel_hw_addr(pio_uart_rx_dma[channel])->write_addr;
            write_index =
                (uint16_t)(((write_addr - ring_base) / sizeof(uint32_t)) & PIO_UART_RX_DMA_MASK);

            // The RX program's optional framing flag is independent of the DMA
            // data path. It is normally compiled out, but retain the counter.
            if (pio_interrupt_get(PIO_DATA_UART, pio_rx_sm[channel])) {
                pio_interrupt_clear(PIO_DATA_UART, pio_rx_sm[channel]);
                pio_uart_framing_err[channel]++;
            }

            while (pio_uart_rx_dma_read[channel] != write_index) {
                uint8_t batch[PIO_UART_RX_BATCH_MAX];
                uint16_t count = 0;

                while ((pio_uart_rx_dma_read[channel] != write_index) &&
                        (count < PIO_UART_RX_BATCH_MAX)) {
                    uint16_t read_index = pio_uart_rx_dma_read[channel];
                    batch[count++] = pio_uart_decode_rx_word(
                                         channel, pio_uart_rx_dma_ring[ring_row][read_index]);
                    pio_uart_rx_dma_read[channel] =
                        (uint16_t)((read_index + 1U) & PIO_UART_RX_DMA_MASK);
                }

                uint16_t stored = pio_uart_store_batch(batch, count, channel);

                pio_uart_rx_consumed[channel] += count;
                pio_uart_rx_stored[channel] += stored;
                pio_uart_rx_dropped[channel] += (uint32_t)(count - stored);
                if (stored > 0) {
                    input_flag = 1;
                }
            }

            if (input_flag) {
                uart_rx_flow_gate(channel);
                init_time_delimiter_timer(channel);
                if ((opmode == DEVICE_GW_MODE) && (seg_u2e_sem[channel] != NULL)) {
                    xSemaphoreGive(seg_u2e_sem[channel]);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void pio_data_uart_rx_dma_start(int channel, volatile void *write_addr) {
    dma_channel_config config = dma_channel_get_default_config(pio_uart_rx_dma[channel]);

    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config,
                            pio_get_dreq(PIO_DATA_UART, pio_rx_sm[channel], false));
    channel_config_set_ring(&config, true, PIO_UART_RX_DMA_RING_BITS);

    dma_channel_configure(pio_uart_rx_dma[channel], &config,
                          write_addr,
                          &PIO_DATA_UART->rxf[pio_rx_sm[channel]],
                          UINT32_MAX, true);
}

// Restart the channel where it stopped so the consumer's read index stays
// valid; the ring keeps the write pointer inside the same buffer either way.
static void pio_data_uart_rx_dma_rearm(int channel) {
    volatile void *write_addr;

    dma_channel_abort(pio_uart_rx_dma[channel]);
    write_addr = (volatile void *)(uintptr_t)
                 dma_channel_hw_addr(pio_uart_rx_dma[channel])->write_addr;
    pio_data_uart_rx_dma_start(channel, write_addr);
    pio_uart_rx_rearm[channel]++;
}

static void pio_data_uart_dma_enable(void) {
    for (int channel = UART_HW_CH_CNT; channel < DEVICE_UART_CNT; channel++) {
        uint ring_row = (uint)(channel - UART_HW_CH_CNT);

        pio_uart_rx_dma[channel] = dma_claim_unused_channel(true);
        pio_uart_rx_dma_read[channel] = 0;
        pio_data_uart_rx_dma_start(channel, pio_uart_rx_dma_ring[ring_row]);
    }

    if (xTaskCreate(pio_data_uart_rx_task, "PIO_UART_RX", 768, NULL,
                    configMAX_PRIORITIES - 1,
                    &pio_uart_rx_task_handle) != pdPASS) {
        pio_uart_rx_task_handle = NULL;
    }
}

typedef struct {
    uint32_t transfer_count;
    uint32_t consumed;
    uint32_t stored;
    uint32_t dropped;
    uint32_t overflow;
    uint16_t write_index;
    uint16_t read_index;
    uint16_t ring_used;
    uint8_t dma_channel;
    uint8_t dma_busy;
    uint8_t fifo_level;
    uint8_t sm_enabled;
    uint8_t sm_pc;
    uint8_t rx_pin;
    uint8_t rts_shadow;
    uint8_t rts_pin;
} pio_uart_rx_diag_snapshot_t;

static void pio_uart_rx_diag_snapshot(int channel,
                                      pio_uart_rx_diag_snapshot_t *snapshot) {
    uint ring_row = (uint)(channel - UART_HW_CH_CNT);
    uintptr_t ring_base = (uintptr_t)&pio_uart_rx_dma_ring[ring_row][0];
    dma_channel_hw_t *channel_hw = dma_channel_hw_addr(pio_uart_rx_dma[channel]);
    uintptr_t write_addr = (uintptr_t)channel_hw->write_addr;

    snapshot->transfer_count = channel_hw->transfer_count;
    snapshot->consumed = pio_uart_rx_consumed[channel];
    snapshot->stored = pio_uart_rx_stored[channel];
    snapshot->dropped = pio_uart_rx_dropped[channel];
    snapshot->overflow = get_data_buffer_overflow_count(channel);
    snapshot->write_index =
        (uint16_t)(((write_addr - ring_base) / sizeof(uint32_t)) & PIO_UART_RX_DMA_MASK);
    snapshot->read_index = pio_uart_rx_dma_read[channel];
    snapshot->ring_used = get_data_buffer_usedsize(channel);
    snapshot->dma_channel = (uint8_t)pio_uart_rx_dma[channel];
    snapshot->dma_busy = dma_channel_is_busy(pio_uart_rx_dma[channel]) ? 1U : 0U;
    snapshot->fifo_level = (uint8_t)pio_sm_get_rx_fifo_level(PIO_DATA_UART,
                           pio_rx_sm[channel]);
    snapshot->sm_enabled =
        (PIO_DATA_UART->ctrl & (1U << pio_rx_sm[channel])) ? 1U : 0U;
    snapshot->sm_pc = pio_sm_get_pc(PIO_DATA_UART, pio_rx_sm[channel]);
    snapshot->rx_pin = (uint8_t)gpio_get(pio_uart_rx_pin[channel]);
    snapshot->rts_shadow = uart_rts_is_blocked(channel);
    snapshot->rts_pin = uart_rts_pin_is_blocked(channel);
}

static void pio_uart_rx_diag_emit(const char *tag, const char *reason,
                                  int fault_channel, uint32_t stalled_ms) {
    pio_uart_rx_diag_snapshot_t data2;
    pio_uart_rx_diag_snapshot_t data3;
    uint32_t task_state = 0xFFU;
    uint32_t task_hwm = 0U;

    pio_uart_rx_diag_snapshot(SEG_DATA2_CH, &data2);
    pio_uart_rx_diag_snapshot(SEG_DATA3_CH, &data3);
    if (pio_uart_rx_task_handle != NULL) {
        task_state = (uint32_t)eTaskGetState(pio_uart_rx_task_handle);
        task_hwm = (uint32_t)uxTaskGetStackHighWaterMark(pio_uart_rx_task_handle);
    }

    // Keep this to one stdio call. pico-sdk serializes stdio with a spin lock,
    // and frequent multi-call diagnostics previously caused watchdog resets.
    printf("[%s] reason=%s fault_ch=%d stalled_ms=%lu "
           "task_hb=%lu task_state=%lu task_hwm=%lu "
           "ch2{dma=%u busy=%u tc=%lu wr=%u rd=%u fifo=%u sm=%u pc=%u pin=%u "
           "rts=%u/%u consumed=%lu stored=%lu dropped=%lu ring=%u overflow=%lu} "
           "ch3{dma=%u busy=%u tc=%lu wr=%u rd=%u fifo=%u sm=%u pc=%u pin=%u "
           "rts=%u/%u consumed=%lu stored=%lu dropped=%lu ring=%u overflow=%lu}\r\n",
           tag, reason, fault_channel, (unsigned long)stalled_ms,
           (unsigned long)pio_uart_rx_task_heartbeat,
           (unsigned long)task_state, (unsigned long)task_hwm,
           (unsigned int)data2.dma_channel, (unsigned int)data2.dma_busy,
           (unsigned long)data2.transfer_count,
           (unsigned int)data2.write_index, (unsigned int)data2.read_index,
           (unsigned int)data2.fifo_level, (unsigned int)data2.sm_enabled,
           (unsigned int)data2.sm_pc, (unsigned int)data2.rx_pin,
           (unsigned int)data2.rts_shadow, (unsigned int)data2.rts_pin,
           (unsigned long)data2.consumed, (unsigned long)data2.stored,
           (unsigned long)data2.dropped, (unsigned int)data2.ring_used,
           (unsigned long)data2.overflow,
           (unsigned int)data3.dma_channel, (unsigned int)data3.dma_busy,
           (unsigned long)data3.transfer_count,
           (unsigned int)data3.write_index, (unsigned int)data3.read_index,
           (unsigned int)data3.fifo_level, (unsigned int)data3.sm_enabled,
           (unsigned int)data3.sm_pc, (unsigned int)data3.rx_pin,
           (unsigned int)data3.rts_shadow, (unsigned int)data3.rts_pin,
           (unsigned long)data3.consumed, (unsigned long)data3.stored,
           (unsigned long)data3.dropped, (unsigned int)data3.ring_used,
           (unsigned long)data3.overflow);
}

void pio_uart_rx_diag_poll(void) {
    enum {
        PIO_RX_DIAG_STALL_MS = 3000U,
        PIO_RX_DIAG_REPEAT_MS = 60000U,
    };
    static uint32_t last_heartbeat;
    static uint32_t heartbeat_progress_at;
    static uint32_t last_transfer_count[DEVICE_UART_CNT];
    static uint32_t last_consumed[DEVICE_UART_CNT];
    static uint32_t last_stored[DEVICE_UART_CNT];
    static uint32_t dma_not_consumed_since[DEVICE_UART_CNT];
    static uint32_t store_stalled_since[DEVICE_UART_CNT];
    static uint32_t dma_stopped_since[DEVICE_UART_CNT];
    static uint32_t last_report_at;
    static uint8_t sample_valid;
    static uint8_t fault_reported;
    const char *reason = NULL;
    int fault_channel = -1;
    uint32_t stalled_ms = 0U;
    uint32_t now = (uint32_t)millis();
    uint32_t heartbeat = pio_uart_rx_task_heartbeat;

    if (pio_uart_rx_task_handle == NULL) {
        reason = "task_create_failed";
    } else if (!sample_valid || (heartbeat != last_heartbeat)) {
        last_heartbeat = heartbeat;
        heartbeat_progress_at = now;
    } else {
        stalled_ms = now - heartbeat_progress_at;
        if (stalled_ms >= PIO_RX_DIAG_STALL_MS) {
            reason = "shared_task_stalled";
        }
    }

    for (int channel = UART_HW_CH_CNT;
            (reason == NULL) && (channel < DEVICE_UART_CNT); channel++) {
        uint32_t transfer_count =
            dma_channel_hw_addr(pio_uart_rx_dma[channel])->transfer_count;
        uint32_t consumed = pio_uart_rx_consumed[channel];
        uint32_t stored = pio_uart_rx_stored[channel];
        uint8_t sm_enabled =
            (PIO_DATA_UART->ctrl & (1U << pio_rx_sm[channel])) ? 1U : 0U;

        if (dma_channel_is_busy(pio_uart_rx_dma[channel])) {
            dma_stopped_since[channel] = 0U;
        }

        if (!sm_enabled) {
            reason = "rx_sm_disabled";
            fault_channel = channel;
        } else if (!dma_channel_is_busy(pio_uart_rx_dma[channel])) {
            // The consumer task briefly stops the channel to re-arm its transfer
            // count, so only a stop that outlives that window is a real fault.
            if (dma_stopped_since[channel] == 0U) {
                dma_stopped_since[channel] = now;
            }
            stalled_ms = now - dma_stopped_since[channel];
            if (stalled_ms >= PIO_RX_DIAG_STALL_MS) {
                reason = "rx_dma_stopped";
                fault_channel = channel;
            }
        } else if (sample_valid && (transfer_count != last_transfer_count[channel]) &&
                   (consumed == last_consumed[channel])) {
            if (dma_not_consumed_since[channel] == 0U) {
                dma_not_consumed_since[channel] = now;
            }
            stalled_ms = now - dma_not_consumed_since[channel];
            if (stalled_ms >= PIO_RX_DIAG_STALL_MS) {
                reason = "dma_not_consumed";
                fault_channel = channel;
            }
        } else {
            dma_not_consumed_since[channel] = 0U;
        }

        if ((reason == NULL) && sample_valid &&
                (consumed != last_consumed[channel]) &&
                (stored == last_stored[channel])) {
            if (store_stalled_since[channel] == 0U) {
                store_stalled_since[channel] = now;
            }
            stalled_ms = now - store_stalled_since[channel];
            if (stalled_ms >= PIO_RX_DIAG_STALL_MS) {
                reason = "store_stalled";
                fault_channel = channel;
            }
        } else if ((consumed == last_consumed[channel]) ||
                   (stored != last_stored[channel])) {
            store_stalled_since[channel] = 0U;
        }

        last_transfer_count[channel] = transfer_count;
        last_consumed[channel] = consumed;
        last_stored[channel] = stored;
    }
    sample_valid = 1U;

    if (reason == NULL) {
        fault_reported = 0U;
        return;
    }
    if (!fault_reported || ((uint32_t)(now - last_report_at) >= PIO_RX_DIAG_REPEAT_MS)) {
        pio_uart_rx_diag_emit("PIO_RX_DIAG", reason, fault_channel, stalled_ms);
        last_report_at = now;
        fault_reported = 1U;
    }
}

// TX one byte on a PIO channel (blocks only if the 8-deep TX FIFO is full).
// With CTS handshaking the SM stalls until the peer is ready, so the FIFO can
// stay full indefinitely; feed the watchdog while waiting for room.
static void pio_data_uart_putc(int channel, uint8_t c) {
    // A DMA transfer may still be feeding this SM's FIFO; writing directly into
    // it now would interleave this byte into the middle of that stream.
    platform_uart_tx_wait(channel);

    // Original:
    //     while (pio_sm_is_tx_fifo_full(PIO_DATA_UART, pio_tx_sm[channel])) {
    //         device_wdt_reset();
    //     }
    {
        uint32_t waited_since = 0;

        while (pio_sm_is_tx_fifo_full(PIO_DATA_UART, pio_tx_sm[channel])) {
            uart_tx_spin_wait_until(&waited_since);
        }
    }

    if (pio_uart_parity[channel] == parity_none) {
        uart_tx_program_putc(PIO_DATA_UART, pio_tx_sm[channel], (char)c);
        return;
    }

    // The state machine shifts payload_bits and knows nothing about parity, so the
    // parity bit rides above the data bits and a full word has to be written -
    // a byte write could not carry it.
    {
        uint8_t data_bits = pio_uart_data_bits[channel];
        uint32_t payload = (uint32_t)c & ((1u << data_bits) - 1u);

        payload |= pio_uart_parity_bit(channel, payload) << data_bits;
        pio_sm_put(PIO_DATA_UART, pio_tx_sm[channel], payload);
    }
}

// Wait for a frame that is still in the TX FIFO or the shift register to finish
// leaving the wire. FIFO-empty alone is not enough: the state machine is still
// clocking out the byte it already pulled.
static void pio_uart_tx_drain(int channel) {
    uint32_t baud = pio_uart_baud(channel);

    {
        uint32_t waited_since = 0;

        while (!pio_sm_is_tx_fifo_empty(PIO_DATA_UART, pio_tx_sm[channel])) {
            uart_tx_spin_wait_until(&waited_since);
        }
    }
    busy_wait_us_32(pio_uart_frame_us(channel, baud));
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
            // A full buffer used to be flushed here, discarding all
            // SEG_DATA_BUF_SIZE bytes to make room for one. See the note in
            // put_byte_to_data_buffer().
            //
            // Original:
            //     if (is_data_buffer_full(SEG_DATA0_CH) == TRUE) {
            //         data_buffer_flush(SEG_DATA0_CH);
            //     }

            if (check_serial_store_permitted(ch, SEG_DATA0_CH)) { // ret: [0] not permitted / [1] permitted
                put_byte_to_data_buffer(ch, SEG_DATA0_CH);
                input_flag = 1;
            }
        }
    }

    if (input_flag) {
        uart_rx_flow_gate(SEG_DATA0_CH);
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

        // A full buffer used to be flushed here, discarding all
        // SEG_DATA_BUF_SIZE bytes to make room for one. See the note in
        // put_byte_to_data_buffer().
        //
        // Original:
        //     if (is_data_buffer_full(SEG_DATA1_CH) == TRUE) {
        //         data_buffer_flush(SEG_DATA1_CH);
        //     }

        if (check_serial_store_permitted(ch, SEG_DATA1_CH)) { // ret: [0] not permitted / [1] permitted
            put_byte_to_data_buffer(ch, SEG_DATA1_CH);
            input_flag = 1;
        }
    }

    if (input_flag) {
        uart_rx_flow_gate(SEG_DATA1_CH);
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
        default:
            // The PL011 only implements 5 to 8 data bits, so word_len9 is not
            // reachable on the HW channels; store the value actually applied so
            // a later read reports the real line format. The PIO channels keep
            // their own 9-bit support.
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
    pio_data_uart_dma_enable();
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

#if (DEVICE_UART_CNT > 2)
// PIO receive errors, counted from the RX ISR. HW channels return 0: the PL011
// latches its own framing/parity flags but nothing reads them yet, so a count here
// would be misleading.
uint32_t get_uart_framing_error_count(int channel) {
    if ((channel < UART_HW_CH_CNT) || (channel >= DEVICE_UART_CNT)) {
        return 0;
    }
    return pio_uart_framing_err[channel];
}

uint32_t get_uart_parity_error_count(int channel) {
    if ((channel < UART_HW_CH_CNT) || (channel >= DEVICE_UART_CNT)) {
        return 0;
    }
    return pio_uart_parity_err[channel];
}
#endif

// TEMPORARY - E2S stall investigation. The transmit side stalls with the host's
// frames accepted over TCP but nothing reaching the wire, and RTS is not involved,
// so [FLOW_DIAG]'s trigger cannot see it. These two report the state the transmit
// path actually waits on.
uint8_t uart_cts_level(int channel) {
    uint pin;

    switch (channel) {
    case SEG_DATA0_CH:
        pin = DATA0_UART_CTS_PIN;
        break;
    case SEG_DATA1_CH:
        pin = DATA1_UART_CTS_PIN;
        break;
#if (DEVICE_UART_CNT > 2)
    case SEG_DATA2_CH:
        pin = DATA2_UART_CTS_PIN;
        break;
#endif
#if (DEVICE_UART_CNT > 3)
    case SEG_DATA3_CH:
        pin = DATA3_UART_CTS_PIN;
        break;
#endif
    default:
        return 0;
    }

    // CTS is active low: 0 means the peer is ready to receive.
    return (uint8_t)gpio_get(pin);
}

uint8_t uart_tx_dma_busy(int channel) {
    if ((channel < 0) || (channel >= DEVICE_UART_CNT)) {
        return 0;
    }
    return dma_channel_is_busy(dma_uart_tx[channel]) ? 1 : 0;
}

uint8_t uart_rts_is_blocked(int channel) {
    if ((channel < 0) || (channel >= DEVICE_UART_CNT)) {
        return 0;
    }

    return (data_uart_rts_status[channel] == UART_RTS_HIGH);
}

// The pin is reported alongside the shadow because check_uart_flow_control() gates
// the reassert on the shadow alone: a pin left deasserted while the shadow reads
// asserted would hold the peer off with nothing left to clear it, and a trigger
// that only watched the shadow would never see it.
uint8_t uart_rts_pin_is_blocked(int channel) {
    if ((channel < 0) || (channel >= DEVICE_UART_CNT)) {
        return 0;
    }

    return (gpio_get(data_uart_rts_pin[channel]) == UART_RTS_HIGH);
}

uint8_t platform_uart_cts_ready(int channel) {
    /*
        Check the physical CTS input for every channel, including the PL011
        DATA0/DATA1 UARTs.  Hardware flow control stops a PL011 transfer when
        CTS is high, but a DMA channel feeding that UART remains busy.  Starting
        or waiting on that DMA then wedges ether_to_uart() indefinitely.

        Returning not-ready before the socket buffer is consumed leaves the TCP
        data in place and lets the channel resume without loss when CTS goes low.
        PIO and hardware UART channels now follow the same rule.
    */
    return uart_cts_level(channel) == UART_CTS_LOW;
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

    if (serial_option->data_bits == word_len7) {
        c[0] = ch & 0x007F;
    } else {
        c[0] = ch & 0x00FF;
    }
    device_wdt_reset();
    uart_putc(channel ? DATA1_UART_ID : DATA0_UART_ID, c[0]);

    return RET_OK;
}

// Block until a channel's TX DMA has finished reading its source buffer.
// Callers must do this before overwriting the buffer they last handed to
// platform_uart_puts_dma(), which starts the transfer and returns immediately.
void platform_uart_tx_wait(int channel) {
    // The PIO channels used to transmit synchronously, so there was nothing to
    // wait for. They now have a DMA path of their own and need the same wait.
    //
    // Original:
    //     if (channel >= UART_HW_CH_CNT) {
    //         return;     // PIO transmit is synchronous
    //     }
    //
    // Original:
    //     while (dma_channel_is_busy(dma_uart_tx[channel])) {
    //         device_wdt_reset();
    //     }
    {
        uint32_t waited_since = 0;

        while (dma_channel_is_busy(dma_uart_tx[channel])) {
            uart_tx_spin_wait_bounded(&waited_since, UART_TX_DMA_YIELD_MS);
        }
    }
}

int32_t platform_uart_puts_dma(uint8_t* buf, uint16_t bytes, int channel) {
    uart_inst_t *uart_id[DEVICE_UART_CNT] = {DATA0_UART_ID, DATA1_UART_ID};

    platform_uart_tx_wait(channel);

    // The PIO channels used to fall back to the byte-at-a-time blocking path,
    // which held the CPU for the whole transfer and capped them at roughly half
    // the HW channels' throughput. They now use DMA like the HW channels.
    //
    // Original:
    //     if (channel >= UART_HW_CH_CNT) {
    //         // PIO UART has no DMA TX path; use the blocking PIO transmit path.
    //         return platform_uart_puts(buf, bytes, channel);
    //     }
    //     platform_uart_tx_wait(channel);
    if (channel >= UART_HW_CH_CNT) {
#if (DEVICE_UART_CNT > 2)
        // The DMA writes bytes into the low byte of TXF, so it can only carry an
        // 8-bit-or-narrower payload and cannot toggle a direction pin around the
        // transfer. Parity needs a 9th bit and RS-422/485 needs the driver released
        // after the last frame, so both fall back to the byte-at-a-time path.
        if ((pio_uart_parity[channel] != parity_none) || pio_uart_de_mode[channel]) {
            return platform_uart_puts(buf, bytes, channel);
        }
        dma_channel_configure(dma_uart_tx[channel], &dma_uart_c[channel],
                              (io_rw_8 *)&PIO_DATA_UART->txf[pio_tx_sm[channel]], // write address
                              buf, // read address
                              bytes, // element count (each element is of size transfer_data_size)
                              true); // start now
#endif
        return RET_OK;
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

    if (channel >= UART_HW_CH_CNT) {
#if (DEVICE_UART_CNT > 2)
        // Original: the loop ran without direction control ("RS-485 direction TODO").
        uart_rs485_enable(channel);
        for (i = 0; i < bytes; i++) {
            pio_data_uart_putc(channel, buf[i]);
            device_wdt_reset();
        }
        // Unlike the HW path this waits for the last frame to clear the wire before
        // releasing the driver, so the closing bits are not cut off.
        if (pio_uart_de_mode[channel]) {
            pio_uart_tx_drain(channel);
        }
        uart_rs485_disable(channel);
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
    // PIO channels used to be TTL only and had no direction control to drive.
    // They now use their own RTS pin as the driver enable, which is why RTS/CTS
    // flow control is refused in RS-422/485 mode (see pio_data_uart_init_sm).
    //
    // Original:
    //     if (channel >= UART_HW_CH_CNT) {
    //         return;
    //     }
#if (DEVICE_UART_CNT > 2)
    if ((channel >= UART_HW_CH_CNT) && !pio_uart_de_mode[channel]) {
        return;
    }
#else
    if (channel >= UART_HW_CH_CNT) {
        return;
    }
#endif
    if (uart_if_mode[channel] == UART_IF_RS485) {
        GPIO_Output_Set(data_uart_rts_pin[channel]);
    } else if (uart_if_mode[channel] == UART_IF_RS485_REVERSE) {
        GPIO_Output_Reset(data_uart_rts_pin[channel]);
    }
}

void uart_rs485_disable(int channel) {
    uint8_t is_pio = 0;

    // Original:
    //     if (channel >= UART_HW_CH_CNT) {
    //         return;
    //     }
#if (DEVICE_UART_CNT > 2)
    if (channel >= UART_HW_CH_CNT) {
        if (!pio_uart_de_mode[channel]) {
            return;
        }
        is_pio = 1;
    }
#else
    if (channel >= UART_HW_CH_CNT) {
        return;
    }
#endif

    if ((uart_if_mode[channel] != UART_IF_RS485) &&
            (uart_if_mode[channel] != UART_IF_RS485_REVERSE)) {
        return;     // UART_IF_RS422: full duplex, the driver stays enabled
    }

    // The transmitter has to be idle before the driver is released, or the closing
    // bits never reach the wire. PIO channels are already drained by their caller
    // (pio_uart_tx_drain) because FIFO-empty alone does not mean the shift register
    // has finished.
    if (!is_pio) {
        uart_tx_wait_blocking(channel ? DATA1_UART_ID : DATA0_UART_ID);
    }

    if (uart_if_mode[channel] == UART_IF_RS485) {
        // RTS pin -> Low;
        GPIO_Output_Reset(data_uart_rts_pin[channel]);
    } else {
        // RTS pin -> High
        GPIO_Output_Set(data_uart_rts_pin[channel]);
    }
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
