#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "WIZnet_board.h"
#include "socket.h"

#include "mqtt_transport_interface.h"
#include "seg.h"
#include "deviceHandler.h"
#include "timerHandler.h"
#include "bufferHandler.h"
#include "uartHandler.h"
#include "gpioHandler.h"
#include "ConfigData.h"

//#include <semphr.h>

#include "port_common.h"

// TLS support
#ifdef __USE_S2E_OVER_TLS__
#include "SSLInterface.h"
wiz_tls_context s2e_tlsContext[DEVICE_UART_CNT];
#endif

#include "mqtt_transport_interface.h"

#include "netHandler.h"

//Modbus supprot
#include "mb.h"
#include "mbrtu.h"
#include "mbascii.h"
#include "mbserial.h"

/* Private variables ---------------------------------------------------------*/
uint8_t opmode = DEVICE_GW_MODE;
uint8_t sw_modeswitch_at_mode_on = SEG_DISABLE;

// static variables for function: check_modeswitch_trigger()
static uint8_t triggercode_idx;
static uint8_t ch_tmp[3];

// Gateway mode <-> command mode switch gap time
uint8_t enable_modeswitch_timer = SEG_DISABLE;
volatile uint16_t modeswitch_time = 0;
volatile uint16_t modeswitch_gap_time = DEFAULT_MODESWITCH_INTER_GAP;

static uint8_t mixed_state[DEVICE_UART_CNT] = {MIXED_SERVER, };
static uint16_t client_any_port = 0;

uint8_t enable_serial_input_timer[DEVICE_UART_CNT] = {SEG_DISABLE, };
volatile uint16_t serial_input_time[DEVICE_UART_CNT] = {0, };
uint8_t flag_serial_input_time_elapse[DEVICE_UART_CNT] = {SEG_DISABLE, }; // for Time delimiter

// flags
uint8_t flag_connect_pw_auth[DEVICE_UART_CNT] = {SEG_DISABLE, }; // TCP_SERVER_MODE only
uint8_t flag_auth_time[DEVICE_UART_CNT] = {SEG_DISABLE, }; // TCP_SERVER_MODE only
uint8_t flag_send_keepalive[DEVICE_UART_CNT] = {SEG_DISABLE, };
uint8_t flag_first_keepalive[DEVICE_UART_CNT] = {SEG_DISABLE, };
uint8_t flag_inactivity[DEVICE_UART_CNT] = {SEG_DISABLE, };
static uint8_t connect_pw_progress[DEVICE_UART_CNT] = {0, };

#define SEG_AUTH_FAILED   0U
#define SEG_AUTH_SUCCESS  1U
#define SEG_AUTH_PENDING  2U

/*
    A TCP server normally moves CLOSED -> INIT -> LISTEN and then completes a
    SYN_RECV transition in milliseconds.  The old state machine silently ignored
    every other W5500 state.  If a handshake or close transition got stranded,
    that channel therefore stopped accepting connections until the whole chip was
    reset.  Track those transient states per channel and recycle only the affected
    socket after a bounded interval.
*/
#define SEG_TCP_SERVER_STUCK_TIMEOUT_MS 5000U
static uint8_t seg_tcp_server_transient_active[DEVICE_UART_CNT] = {SEG_DISABLE, };
static uint8_t seg_tcp_server_transient_state[DEVICE_UART_CNT] = {SOCK_CLOSED, };
static uint32_t seg_tcp_server_transient_since[DEVICE_UART_CNT] = {0, };
static uint8_t seg_udp_transient_active[DEVICE_UART_CNT] = {SEG_DISABLE, };
static uint8_t seg_udp_transient_state[DEVICE_UART_CNT] = {SOCK_CLOSED, };
static uint32_t seg_udp_transient_since[DEVICE_UART_CNT] = {0, };

// User's buffer / size idx
extern uint8_t g_send_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];
extern uint8_t g_recv_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];
extern uint8_t g_recv_mqtt_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];

/*the flag of modbus*/
extern volatile uint8_t mb_state_rtu_finish[DEVICE_UART_CNT];
extern volatile uint8_t mb_state_ascii_finish[DEVICE_UART_CNT];

extern xSemaphoreHandle seg_u2e_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle seg_spi_pending_sem;
//extern xSemaphoreHandle conn_seg_sem;
extern xSemaphoreHandle net_seg_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle net_seg_u2e_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle seg_timer_sem;
extern xSemaphoreHandle segcp_uart_sem;
extern xSemaphoreHandle seg_critical_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle seg_socket_sem;
extern xSemaphoreHandle seg_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle seg_recv_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle wizchip_critical_sem;

void seg_wizchip_api_lock(void) {
    if (seg_socket_sem != NULL) {
        xSemaphoreTake(seg_socket_sem, portMAX_DELAY);
    }
}

void seg_wizchip_api_unlock(void) {
    if (seg_socket_sem != NULL) {
        xSemaphoreGive(seg_socket_sem);
    }
}

// The app-wide socket mutex is the one lock every task needs, and it had no
// owner tracking - only the SPI critical section did. send() holds this one
// across an unbounded `while (getSn_CR(sn))`, so a command the chip never
// retires parks the receive task, the recovery poll and the diagnostic reporter
// in the same instant. That is what a wedge looks like from the log: a last
// [FLOW_DIAG], then silence, then the watchdog.
static const char *volatile seg_sock_lock_owner;
static volatile uint32_t seg_sock_lock_since_us;

// Naming the holder was not enough: SEG_U2E_Task turned out to be the one that
// keeps the mutex, but the bounded command wait added for it never even runs,
// because the send returns SOCK_BUSY on free space long before reaching it. So
// also record which call inside the lock is the slow one.
// Split finer after step 2 came back with the SPI lock unheld: four short
// register reads cannot take five seconds between them, so which one it is
// matters now.
#define SEG_SOCK_STEP_OTHER      0U
#define SEG_SOCK_STEP_TX_FSR     1U   // caller-side free-size read
#define SEG_SOCK_STEP_SEND_SR    2U   // getSn_SR()
#define SEG_SOCK_STEP_SEND_IR    3U   // getSn_IR() / setSn_IR()
#define SEG_SOCK_STEP_SEND_FSR   4U   // getSn_TxMAX() and the free-size read
#define SEG_SOCK_STEP_SEND_DATA  5U   // wiz_send_data(), a 2 KB SPI burst
#define SEG_SOCK_STEP_SEND_CMD   6U   // the Sn_CR poll after Sn_CR_SEND
#define SEG_SOCK_STEP_RECV       7U
#define SEG_SOCK_STEP_RX_AVAIL   8U

static volatile uint8_t seg_sock_lock_step;

// The step number is not cleared on release, so it names the last place the
// lock reached rather than where it stopped - and a send that ends in SOCK_BUSY
// leaves it at 4 every normal pass. Count acquisitions too: if the count moves
// while the hold looks stuck, the task is spinning through the lock rather than
// standing still in it, and those need opposite fixes.
static volatile uint32_t seg_sock_lock_seq;

uint32_t seg_socket_lock_seq(void) {
    return seg_sock_lock_seq;
}

// ioLibrary's getSn_TX_FSR() reads the 16-bit free size twice and repeats until
// two consecutive reads agree, with nothing bounding the loop. That settles at
// once while the chip is idle, but on a channel whose transmit is creeping - a
// wedged socket still drains around 1.6 kB/s - the value moves between every
// pair of reads and the two never agree. The postmortem caught SEG_U2E_Task
// there holding the app-wide socket mutex, which stops the receive task from
// reaching the watchdog feed at the tail of its loop, and the device resets
// 8.388 s later having logged nothing.
//
// Read it the same way with a limit, and take the last sample when the reads
// will not settle. A free size one sample stale costs at most one send; not
// returning costs the device. The vendor's `if (val1 != 0)` guard is dropped
// with it - it skips the confirming read whenever the first one is zero, so
// zero was the single value that arrived unverified.
#define SEG_TX_FSR_MAX_ATTEMPTS 8U

static uint16_t seg_tx_fsr_bounded(uint8_t sock) {
    uint16_t val;
    uint16_t val1;
    uint32_t attempts = 0;

    do {
        val1 = WIZCHIP_READ(Sn_TX_FSR(sock));
        val1 = (uint16_t)((val1 << 8) +
                          WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sock), 1)));
        val = WIZCHIP_READ(Sn_TX_FSR(sock));
        val = (uint16_t)((val << 8) +
                         WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sock), 1)));
        attempts++;
    } while ((val != val1) && (attempts < SEG_TX_FSR_MAX_ATTEMPTS));

    return val;
}

// The watchdog is fed at the tail of the receive loop, so recv_beat at the head
// proves the loop was entered, not that it finished. Between the two sits
// ether_to_uart(), and the last run went fourteen seconds from its final log
// line to the reset. Record where in that stretch each channel is standing.
#define SEG_RECV_STEP_HEAD      1U   // loop entered, nothing called yet
#define SEG_RECV_STEP_CTS       2U   // waiting on the flow-control gate
#define SEG_RECV_STEP_RX_AVAIL  3U   // asking the chip how much is waiting
#define SEG_RECV_STEP_TX_WAIT   4U   // platform_uart_tx_wait()
#define SEG_RECV_STEP_SOCK_RECV 5U   // pulling the payload off the chip
#define SEG_RECV_STEP_UART_TX   6U   // handing the payload to the UART
#define SEG_RECV_STEP_TAIL      7U   // past the delay, about to feed

static volatile uint8_t seg_recv_step[DEVICE_UART_CNT];

// Four channels packed one byte each, so a single record says which of them
// stopped and where.
uint32_t seg_recv_steps_packed(void) {
    uint32_t packed = 0;
    int channel;

    for (channel = 0; channel < DEVICE_UART_CNT; channel++) {
        packed |= ((uint32_t)seg_recv_step[channel] << (channel * 8U));
    }

    return packed;
}

static void seg_socket_lock(void) {
    seg_wizchip_api_lock();
    seg_sock_lock_seq++;
    seg_sock_lock_step = SEG_SOCK_STEP_OTHER;
    seg_sock_lock_since_us = time_us_32();
    seg_sock_lock_owner = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
                          ? pcTaskGetName(NULL) : "init";
}

static void seg_socket_unlock(void) {
    seg_sock_lock_owner = NULL;
    seg_wizchip_api_unlock();
}

// Reports how long the mutex has been held, and by whom as four characters of
// the holder's name. Offset four is where the SEG_ tasks stop agreeing: U2E_,
// Recv, Task. Reads nothing but two volatiles, so it is safe from the idle hook
// while every other task is blocked.
uint32_t seg_socket_lock_held_ms(uint32_t *owner_tag) {
    // Sample the sequence around the read and discard the result if the lock
    // changed hands meanwhile. Without that, a task cycling through here
    // thousands of times a second can update the timestamp between the clock
    // read and the timestamp read, making the difference negative - which wraps
    // to an enormous unsigned value and reports a hold that never happened.
    // That is exactly what the reports of a five-second hold turned out to be,
    // and the same mistake the watchdog gap tracker already had to correct.
    uint32_t seq = seg_sock_lock_seq;
    uint32_t since = seg_sock_lock_since_us;
    const char *owner = seg_sock_lock_owner;
    uint32_t now = time_us_32();

    if ((owner == NULL) || (seg_sock_lock_seq != seq)) {
        return 0;
    }

    if (owner_tag != NULL) {
        uint32_t tag = 0;
        uint32_t len = (uint32_t)strlen(owner);

        for (uint32_t i = 0; i < 3U; i++) {
            uint32_t idx = i + 4U;
            uint8_t c = (idx < len) ? (uint8_t)owner[idx] : (uint8_t)' ';

            tag |= ((uint32_t)c << (i * 8U));
        }
        *owner_tag = tag | ((uint32_t)seg_sock_lock_step << 24U);
    }

    return (now - since) / 1000U;
}

// ioLibrary's sendto() remains in its SENDOK/TIMEOUT polling loop even when the
// socket was opened with SF_IO_NONBLOCK.  Holding seg_socket_sem across that
// loop lets one unreachable UDP peer pause every data/configuration socket for
// the complete W5500 retry window.  Keep a tiny per-socket completion state and
// return as soon as the W5500 has accepted Sn_CR_SEND instead.
static uint8_t seg_udp_send_pending[_WIZCHIP_SOCK_NUM_] = {0, };

static void seg_udp_send_reset_locked(uint8_t sock) {
    if (sock < _WIZCHIP_SOCK_NUM_) {
        seg_udp_send_pending[sock] = SEG_DISABLE;
    }
}

void seg_wizchip_udp_send_reset(uint8_t sock) {
    if (sock >= _WIZCHIP_SOCK_NUM_) {
        return;
    }

    seg_socket_lock();
    seg_udp_send_reset_locked(sock);
    if ((getSn_MR(sock) & 0x0f) == Sn_MR_UDP) {
        setSn_IR(sock, Sn_IR_SENDOK | Sn_IR_TIMEOUT);
    }
    seg_socket_unlock();
}

int32_t seg_wizchip_udp_send_nonblocking(uint8_t sock, uint8_t *buf,
        uint16_t len, uint8_t *addr, uint16_t port) {
    int32_t result = SOCK_BUSY;
    uint8_t interrupt;
    uint16_t tx_max;

    if (sock >= _WIZCHIP_SOCK_NUM_) {
        return SOCKERR_SOCKNUM;
    }
    if ((buf == NULL) || (addr == NULL) || (len == 0)) {
        return SOCKERR_DATALEN;
    }
    if ((addr[0] | addr[1] | addr[2] | addr[3]) == 0) {
        return SOCKERR_IPINVALID;
    }
    if (port == 0) {
        return SOCKERR_PORTZERO;
    }

    seg_socket_lock();

    if (((getSn_MR(sock) & 0x0f) != Sn_MR_UDP) ||
            (getSn_SR(sock) != SOCK_UDP)) {
        result = SOCKERR_SOCKSTATUS;
        goto out;
    }

    // A command still present in Sn_CR has not yet been accepted by the chip.
    // Do not spin here: another pass will reap it after the register clears.
    if (getSn_CR(sock) != 0) {
        goto out;
    }

    interrupt = getSn_IR(sock);
    if (seg_udp_send_pending[sock] == SEG_ENABLE) {
        if (interrupt & Sn_IR_TIMEOUT) {
            setSn_IR(sock, Sn_IR_TIMEOUT);
            seg_udp_send_pending[sock] = SEG_DISABLE;
            result = SOCKERR_TIMEOUT;
            goto out;
        }
        if (!(interrupt & Sn_IR_SENDOK)) {
            goto out;
        }
        setSn_IR(sock, Sn_IR_SENDOK);
        seg_udp_send_pending[sock] = SEG_DISABLE;
    } else {
        // Clear stale completion bits left by a previous socket generation.
        if (interrupt & (Sn_IR_SENDOK | Sn_IR_TIMEOUT)) {
            setSn_IR(sock, interrupt & (Sn_IR_SENDOK | Sn_IR_TIMEOUT));
        }
    }

    tx_max = getSn_TxMAX(sock);
    if (len > tx_max) {
        len = tx_max;
    }
    if (seg_tx_fsr_bounded(sock) < len) {
        goto out;
    }

    setSn_DIPR(sock, addr);
    setSn_DPORT(sock, port);
    wiz_send_data(sock, buf, len);
    setSn_CR(sock, Sn_CR_SEND);
    seg_udp_send_pending[sock] = SEG_ENABLE;
    result = (int32_t)len;

out:
    seg_socket_unlock();
    return result;
}

extern TimerHandle_t seg_inactivity_timer[DEVICE_UART_CNT];
extern TimerHandle_t seg_keepalive_timer[DEVICE_UART_CNT];
extern TimerHandle_t seg_auth_timer[DEVICE_UART_CNT];
extern TimerHandle_t spi_reset_timer;

uint16_t u2e_size[DEVICE_UART_CNT] = {0, };
uint16_t e2u_size[DEVICE_UART_CNT] = {0, };

// Raw-TCP send progress is tracked independently from verbose diagnostics.  A
// successful return is the only safe proof that this connection is draining;
// task heartbeats can keep advancing while send() returns SOCK_BUSY forever.
static volatile uint16_t seg_s2e_last_send_len[DEVICE_UART_CNT];
static volatile int16_t seg_s2e_last_send_rc[DEVICE_UART_CNT];
static volatile uint32_t seg_s2e_tx_progress[DEVICE_UART_CNT];

// send() returns SOCK_BUSY from two places that mean opposite things: one where
// a SEND is still outstanding on the chip, and one where the transmit buffer is
// simply too full for this request.  Which of the two produced a stall decides
// whether the chip is holding an unacknowledged segment or the driver missed a
// SEND_OK, and the flag that separates them - sock_is_sending - is a static
// inside socket.c.  Mirror its transitions here instead; done under the same
// lock as the send, the copy is exact.
static volatile uint8_t seg_s2e_send_pending[DEVICE_UART_CNT];

// Manual TCP keepalive is only useful while a connection is idle.  Issuing
// SEND_KEEP merely outside the socket API mutex is not sufficient: send()
// returns after starting Sn_CR_SEND, so that command may still be pending when
// the timer task enters setsockopt(SO_KEEPALIVESEND).  Track application traffic
// and never issue SEND_KEEP over an active or backpressured data stream.
static volatile uint32_t seg_tcp_last_activity_ms[DEVICE_UART_CNT];

static void seg_tcp_mark_activity(int channel) {
    if ((channel >= 0) && (channel < DEVICE_UART_CNT)) {
        seg_tcp_last_activity_ms[channel] = (uint32_t)millis();
    }
}

// Stamped by seg_ch_task once per loop and read by seg_ch_u2e_task, which runs at a
// higher priority and otherwise has no way to tell that it is starving the task it
// shares seg_critical_sem with. Not diagnostics - seg_ch_u2e_task acts on it.
static volatile uint32_t seg_task_heartbeat_ms[DEVICE_UART_CNT];

// Every capture so far showed wizchip_critical_sem held at the sampling instant, so
// record who took it and when. Done by re-registering ioLibrary's critical-section
// callbacks instead of editing the driver port: wizchip_cris_initialize() runs once
// at startup (App.c) and nothing re-registers after this, so the swap sticks.
static const char *volatile seg_wiz_lock_owner;
static volatile uint32_t seg_wiz_lock_since;
static volatile uint8_t seg_wiz_lock_waiters;

static void seg_wiz_cris_enter(void) {
    // A task queued on this semaphore has not set itself as owner yet, so an
    // unheld lock and a contended one look identical from outside. They are
    // opposite findings, so say which it is.
    seg_wiz_lock_waiters++;
    xSemaphoreTake(wizchip_critical_sem, portMAX_DELAY);
    seg_wiz_lock_waiters--;
    // Hardware timer, not millis(): the 1 ms callback behind millis() is starved
    // in the very fault this measures, so it would read zero exactly when the
    // number matters.
    seg_wiz_lock_since = time_us_32();
    // Also reached before the scheduler starts, where there is no task to name.
    seg_wiz_lock_owner = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
                         ? pcTaskGetName(NULL) : "init";
}

static void seg_wiz_cris_exit(void) {
    seg_wiz_lock_owner = NULL;
    xSemaphoreGive(wizchip_critical_sem);
}

static uint32_t seg_wiz_lock_held_ms(void) {
    // Timestamp before clock, for the same reason as the socket mutex: the
    // other order turns a lock taken between the two reads into a negative
    // difference, and an unsigned one at that.
    uint32_t since = seg_wiz_lock_since;
    uint32_t now;

    if (seg_wiz_lock_owner == NULL) {
        return 0;
    }
    now = time_us_32();

    return ((int32_t)(now - since) > 0) ? ((now - since) / 1000U) : 0U;
}

// The locks are two deep. Bounding the free-size read pushed the failure from
// cycle 4-18 out to 63, but the postmortem still names the same task at the same
// step - and 32 SPI accesses cannot take five seconds, so that read was never
// what held it. What remains inside that step is this lock: every WIZCHIP_READ
// takes the SPI critical section, so a task holding the app-wide socket mutex
// can stand here waiting for whoever owns the inner one. FLOW_DIAG has been
// printing wiz_owner=SEG_Task all along, but it needs the same lock to print, so
// it falls silent exactly when the value is worth having. Read it from the idle
// hook instead, which keeps running when every task is blocked.
//
// One character separates the holders: offset four is where the task names stop
// agreeing (SEG_Task, SEG_U2E_Task, SEG_Recv_Task, SEGCP_udp_Task).
uint32_t seg_wiz_lock_snapshot(uint8_t *owner_char, uint8_t *waiters) {
    const char *owner = seg_wiz_lock_owner;

    if (waiters != NULL) {
        *waiters = seg_wiz_lock_waiters;
    }

    if (owner == NULL) {
        if (owner_char != NULL) {
            *owner_char = 0;
        }
        return 0;
    }

    if (owner_char != NULL) {
        *owner_char = (strlen(owner) > 4U) ? (uint8_t)owner[4] : (uint8_t)'?';
    }

    return seg_wiz_lock_held_ms();
}


#if SEG_S2E_STALL_RECOVERY_ENABLE
// A full-duplex 44 kB/s soak intentionally creates short periods of legitimate
// RTS backpressure, some lasting more than ten seconds.  Recovery therefore
// requires thirty seconds with no successful raw-TCP send as well as the exact
// W5500 state captured in the permanent failure.  Recycling a socket loses that
// connection, so false positives are worse than a delayed recovery.
#define SEG_S2E_STALL_RECOVERY_MS 30000U

static uint32_t seg_s2e_recovery_progress_mark[DEVICE_UART_CNT];
static uint32_t seg_s2e_recovery_stall_since[DEVICE_UART_CNT];

static uint8_t seg_s2e_recovery_mode_supported(uint8_t mode) {
    return ((mode == TCP_CLIENT_MODE) ||
            (mode == TCP_SERVER_MODE) ||
            (mode == TCP_MIXED_MODE)) ? TRUE : FALSE;
}

static void seg_s2e_recovery_reset(int channel, uint32_t progress) {
    seg_s2e_recovery_progress_mark[channel] = progress;
    seg_s2e_recovery_stall_since[channel] = 0;
}

static void seg_s2e_recovery_poll(int channel) {
    struct __network_connection *network_connection =
        (struct __network_connection *)&get_DevConfig_pointer()->network_connection[channel];
    struct __serial_option *serial_option =
        (struct __serial_option *)&get_DevConfig_pointer()->serial_option[channel];
    uint32_t now = (uint32_t)millis();
    uint32_t progress = seg_s2e_tx_progress[channel];
    uint16_t ring_used = get_data_buffer_usedsize(channel);
    uint8_t sock = seg_data_sock[channel];

    if (progress != seg_s2e_recovery_progress_mark[channel]) {
        seg_s2e_recovery_reset(channel, progress);
        return;
    }

    if ((opmode != DEVICE_GW_MODE) ||
            (get_serial_communation_protocol(channel) != SEG_SERIAL_PROTOCOL_NONE) ||
            !seg_s2e_recovery_mode_supported(network_connection->working_mode) ||
            (network_connection->working_state != ST_CONNECT) ||
            (serial_option->flow_control != flow_rts_cts) ||
            !uart_rts_is_blocked(channel) ||
            (ring_used == 0) ||
            (u2e_size[channel] == 0) ||
            (seg_s2e_last_send_len[channel] == 0) ||
            (seg_s2e_last_send_rc[channel] != SOCK_BUSY)) {
        seg_s2e_recovery_reset(channel, progress);
        return;
    }

    seg_socket_lock();
    uint16_t tx_fsr_first = seg_tx_fsr_bounded(sock);
    uint16_t tx_fsr_second = seg_tx_fsr_bounded(sock);
    uint8_t sn_ir = getSn_IR(sock);
    uint8_t sn_sr = getSn_SR(sock);
    seg_socket_unlock();

    if ((sn_sr != SOCK_ESTABLISHED) ||
            (tx_fsr_first != 0) || (tx_fsr_second != 0) ||
            (sn_ir & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON))) {
        seg_s2e_recovery_reset(channel, progress);
        return;
    }

    if (seg_s2e_recovery_stall_since[channel] == 0) {
        seg_s2e_recovery_stall_since[channel] = now;
        return;
    }

    uint32_t stalled_ms = (uint32_t)(now - seg_s2e_recovery_stall_since[channel]);
    if (stalled_ms < SEG_S2E_STALL_RECOVERY_MS) {
        return;
    }

    // Close only the affected channel. Take seg_critical_sem ourselves so the
    // final validation and close are one atomic operation with respect to both
    // channel data tasks. A send may have completed while this monitor waited for
    // the lock; in that case the connection is healthy and must be left alone.
    xSemaphoreTake(seg_critical_sem[channel], portMAX_DELAY);
    progress = seg_s2e_tx_progress[channel];
    if ((progress != seg_s2e_recovery_progress_mark[channel]) ||
            (network_connection->working_state != ST_CONNECT) ||
            !uart_rts_is_blocked(channel) ||
            (u2e_size[channel] == 0) ||
            (seg_s2e_last_send_rc[channel] != SOCK_BUSY)) {
        xSemaphoreGive(seg_critical_sem[channel]);
        seg_s2e_recovery_reset(channel, progress);
        return;
    }

    ring_used = get_data_buffer_usedsize(channel);
    seg_socket_lock();
    tx_fsr_first = seg_tx_fsr_bounded(sock);
    tx_fsr_second = seg_tx_fsr_bounded(sock);
    sn_ir = getSn_IR(sock);
    sn_sr = getSn_SR(sock);
    seg_socket_unlock();

    if ((ring_used == 0) || (sn_sr != SOCK_ESTABLISHED) ||
            (tx_fsr_first != 0) || (tx_fsr_second != 0) ||
            (sn_ir & (Sn_IR_SENDOK | Sn_IR_TIMEOUT | Sn_IR_DISCON))) {
        xSemaphoreGive(seg_critical_sem[channel]);
        seg_s2e_recovery_reset(channel, progress);
        return;
    }

    uint16_t staged = u2e_size[channel];
    uint16_t send_len = seg_s2e_last_send_len[channel];
    int16_t send_rc = seg_s2e_last_send_rc[channel];
    // Sampled before the recycle: closing the socket clears the mirror.
    uint8_t sending = seg_s2e_send_pending[channel];

    process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
    set_device_status(ST_OPEN, channel);
    xSemaphoreGive(seg_critical_sem[channel]);
    seg_s2e_recovery_reset(channel, seg_s2e_tx_progress[channel]);

    // Print after releasing the channel lock. USB stdio can block, and recovery
    // must not turn a socket fault into a channel-task lockout.
    printf("[S2E_RECOVERY] ch=%d sock=%u stalled_ms=%lu ring=%u u2e=%u "
           "tx_fsr=%u/%u sn_ir=0x%02x sr=0x%02x send_len=%u send_rc=%d sending=%u "
           "action=socket_recycle\r\n",
           channel, (unsigned int)sock, (unsigned long)stalled_ms,
           (unsigned int)ring_used, (unsigned int)staged,
           (unsigned int)tx_fsr_first, (unsigned int)tx_fsr_second,
           (unsigned int)sn_ir, (unsigned int)sn_sr,
           (unsigned int)send_len, (int)send_rc, (unsigned int)sending);
}
#endif

// channel -> W5500 data socket map (also used by netHandler.c)
const uint8_t seg_data_sock[DEVICE_UART_CNT] = {
    SEG_DATA0_SOCK, SEG_DATA1_SOCK,
#if (DEVICE_UART_CNT > 2)
    SEG_DATA2_SOCK,
#endif
#if (DEVICE_UART_CNT > 3)
    SEG_DATA3_SOCK,
#endif
};

// channel -> TCP-connection status LED pin
static const uint16_t seg_status_pin[DEVICE_UART_CNT] = {
    DATA0_STATUS_TCPCONNECT_PIN, DATA1_STATUS_TCPCONNECT_PIN,
#if (DEVICE_UART_CNT > 2)
    DATA2_STATUS_TCPCONNECT_PIN,
#endif
#if (DEVICE_UART_CNT > 3)
    DATA3_STATUS_TCPCONNECT_PIN,
#endif
};

// UDP 1:N peer information is owned by the channel that received it.  The old
// single global peer let the last packet on DATA3 redirect DATA0's serial data.
static uint8_t udp_peer_ip[DEVICE_UART_CNT][4] = {{0, }};
static uint16_t udp_peer_port[DEVICE_UART_CNT] = {0, };
static uint8_t udp_peer_valid[DEVICE_UART_CNT] = {SEG_DISABLE, };

// XON/XOFF (Software flow control) flag, Serial data can be transmitted to peer when XON enabled.
uint8_t isXON[DEVICE_UART_CNT] = {SEG_ENABLE, SEG_ENABLE,
#if (DEVICE_UART_CNT > 2)
                                  SEG_ENABLE,
#endif
#if (DEVICE_UART_CNT > 3)
                                  SEG_ENABLE,
#endif
                                 };

char * str_working[] = {"TCP_CLIENT_MODE", "TCP_SERVER_MODE", "TCP_MIXED_MODE", "UDP_MODE", "SSL_TCP_CLIENT_MODE", "MQTT_CLIENT_MODE", "MQTTS_CLIENT_MODE"};

//Network mqtt_n;
//MQTTClient mqtt_c = DefaultClient;
//MQTTPacket_connectData mqtt_data = MQTTPacket_connectData_initializer;
NetworkContext_t g_network_context[DEVICE_UART_CNT];
TransportInterface_t g_transport_interface[DEVICE_UART_CNT];
mqtt_config_t g_mqtt_config[DEVICE_UART_CNT];

/* Private functions prototypes ----------------------------------------------*/
void proc_SEG_tcp_client(uint8_t sock, int channel);
void proc_SEG_tcp_server(uint8_t sock, int channel);
void proc_SEG_tcp_mixed(uint8_t sock, int channel);
void proc_SEG_udp(uint8_t sock, int channel);
void proc_SEG_mqtt_client(uint8_t sock, int channel);
void proc_SEG_mqtts_client(uint8_t sock, int channel);

#ifdef __USE_S2E_OVER_TLS__
void proc_SEG_tcp_client_over_tls(uint8_t sock, int channel);
#endif

void uart_to_ether(uint8_t sock, int channel);
void ether_to_uart(uint8_t sock, int channel);
uint16_t get_serial_data(int channel);
void restore_serial_data(uint8_t idx);

/*
    ioLibrary's CRIS callback protects a single SPI register transaction, but a
    non-blocking socket operation spans several transactions. socket.c also
    stores all sockets' state in shared bitmaps (sock_is_sending/sock_io_mode),
    whose read-modify-write updates are not atomic on the dual-core build.
    Serialize complete operations across every SEG data socket. This is kept
    separate from seg_critical_sem and is never held while waiting for UART.
*/
// ioLibrary's send() ends with an unbounded `while (getSn_CR(sn))`, and callers
// hold the app-wide socket mutex across the whole call. A SEND the chip never
// retires therefore stops the receive task, the recovery poll and the reporter
// behind that lock, and the watchdog resets the device with nothing logged -
// confirmed by a postmortem naming SEG_U2E_Task as the holder, reproducible in
// under a minute by cycling the peer's receive window.
//
// The driver is a submodule tracking the vendor's repository, so the wait is
// bounded here instead. This performs the same sequence send() does for a TCP
// socket; the differences are the limit on the command wait and keeping the
// outstanding-send flag in this file rather than socket.c's static bitmap.
// Only the raw SEG data path uses this, and nothing else calls send() on those
// sockets while it does, so the two states never disagree.
//
// Retiring a command takes the chip microseconds. This allows several thousand
// times that, and only runs out when the chip has stopped answering - at which
// point releasing the mutex matters more than this one transfer. The data is
// already staged in the transmit buffer and the stall recovery recycles the
// socket if nothing moves again.
#define SEG_SEND_CMD_POLL_LIMIT 20000UL

static int16_t seg_tcp_send_locked(uint8_t sock, uint8_t *buf, uint16_t len,
                                   int channel) {
    uint32_t polls = 0;
    uint16_t freesize;
    uint8_t sr;

    if (len == 0) {
        return SOCKERR_DATALEN;
    }

    seg_sock_lock_step = SEG_SOCK_STEP_SEND_SR;
    sr = getSn_SR(sock);
    if ((sr != SOCK_ESTABLISHED) && (sr != SOCK_CLOSE_WAIT)) {
        return SOCKERR_SOCKSTATUS;
    }

    if (seg_s2e_send_pending[channel]) {
        uint8_t sn_ir;

        seg_sock_lock_step = SEG_SOCK_STEP_SEND_IR;
        sn_ir = getSn_IR(sock);

        if (sn_ir & Sn_IR_SENDOK) {
            setSn_IR(sock, Sn_IR_SENDOK);
            seg_s2e_send_pending[channel] = 0;
        } else if (sn_ir & Sn_IR_TIMEOUT) {
            // Left open deliberately. send() closes here, but close() spins on
            // the same command register this whole function exists to escape.
            seg_s2e_send_pending[channel] = 0;
            return SOCKERR_TIMEOUT;
        } else {
            return SOCK_BUSY;
        }
    }

    seg_sock_lock_step = SEG_SOCK_STEP_SEND_FSR;
    freesize = getSn_TxMAX(sock);
    if (len > freesize) {
        len = freesize;
    }
    if (len > seg_tx_fsr_bounded(sock)) {
        return SOCK_BUSY;
    }

    seg_sock_lock_step = SEG_SOCK_STEP_SEND_DATA;
    wiz_send_data(sock, buf, len);
    setSn_CR(sock, Sn_CR_SEND);

    seg_sock_lock_step = SEG_SOCK_STEP_SEND_CMD;
    while (getSn_CR(sock)) {
        if (++polls >= SEG_SEND_CMD_POLL_LIMIT) {
            seg_s2e_send_pending[channel] = 1;
            return SOCK_BUSY;
        }
    }

    seg_s2e_send_pending[channel] = 1;

    return (int16_t)len;
}

static void seg_send_pending_clear_sock(uint8_t sock) {
    int channel;

    for (channel = 0; channel < DEVICE_UART_CNT; channel++) {
        if (seg_data_sock[channel] == sock) {
            seg_s2e_send_pending[channel] = 0;
        }
    }
}

static int8_t seg_socket_open(uint8_t sock, uint8_t protocol,
                              uint16_t port, uint8_t flag) {
    int8_t result;

    seg_socket_lock();
    seg_udp_send_reset_locked(sock);
    result = socket(sock, protocol, port, flag);
    seg_send_pending_clear_sock(sock);
    seg_socket_unlock();

    return result;
}

static int8_t seg_socket_connect(uint8_t sock, uint8_t *addr, uint16_t port) {
    int8_t result;

    seg_socket_lock();
    result = connect(sock, addr, port);
    seg_socket_unlock();

    return result;
}

static int8_t seg_socket_listen(uint8_t sock) {
    int8_t result;

    seg_socket_lock();
    result = listen(sock);
    seg_socket_unlock();

    return result;
}

static void seg_tcp_server_transient_reset(int channel) {
    seg_tcp_server_transient_active[channel] = SEG_DISABLE;
    seg_tcp_server_transient_state[channel] = SOCK_CLOSED;
    seg_tcp_server_transient_since[channel] = 0;
}

static uint8_t seg_tcp_server_transient_timed_out(int channel, uint8_t state) {
    uint32_t now = (uint32_t)millis();

    if ((seg_tcp_server_transient_active[channel] == SEG_DISABLE) ||
            (seg_tcp_server_transient_state[channel] != state)) {
        seg_tcp_server_transient_active[channel] = SEG_ENABLE;
        seg_tcp_server_transient_state[channel] = state;
        seg_tcp_server_transient_since[channel] = now;
        return FALSE;
    }

    return ((uint32_t)(now - seg_tcp_server_transient_since[channel]) >=
            SEG_TCP_SERVER_STUCK_TIMEOUT_MS) ? TRUE : FALSE;
}

static void seg_udp_transient_reset(int channel) {
    seg_udp_transient_active[channel] = SEG_DISABLE;
    seg_udp_transient_state[channel] = SOCK_CLOSED;
    seg_udp_transient_since[channel] = 0;
}

static uint8_t seg_udp_transient_timed_out(int channel, uint8_t state) {
    uint32_t now = (uint32_t)millis();

    if ((seg_udp_transient_active[channel] == SEG_DISABLE) ||
            (seg_udp_transient_state[channel] != state)) {
        seg_udp_transient_active[channel] = SEG_ENABLE;
        seg_udp_transient_state[channel] = state;
        seg_udp_transient_since[channel] = now;
        return FALSE;
    }

    return ((uint32_t)(now - seg_udp_transient_since[channel]) >=
            SEG_TCP_SERVER_STUCK_TIMEOUT_MS) ? TRUE : FALSE;
}

static void seg_udp_peer_reset(int channel) {
    memset(udp_peer_ip[channel], 0, sizeof(udp_peer_ip[channel]));
    udp_peer_port[channel] = 0;
    udp_peer_valid[channel] = SEG_DISABLE;
}

static int8_t seg_socket_disconnect(uint8_t sock) {
    int8_t result;

    seg_socket_lock();
    result = disconnect(sock);
    seg_send_pending_clear_sock(sock);
    seg_socket_unlock();

    return result;
}

static int8_t seg_socket_close(uint8_t sock) {
    int8_t result;

    seg_socket_lock();
    result = close(sock);
    seg_udp_send_reset_locked(sock);
    seg_send_pending_clear_sock(sock);
    seg_socket_unlock();

    return result;
}

static int16_t seg_socket_send(uint8_t sock, uint8_t *buf, uint16_t len, int channel) {
    int16_t sent;

    seg_socket_lock();
    sent = seg_tcp_send_locked(sock, buf, len, channel);
    if (sent > 0) {
        seg_tcp_mark_activity(channel);
    }
    seg_socket_unlock();

    return sent;
}

static int16_t seg_socket_send_available(uint8_t sock, uint8_t *buf,
        uint16_t *len, int channel) {
    int16_t sent;
    uint16_t freesize;

    seg_socket_lock();
    seg_sock_lock_step = SEG_SOCK_STEP_TX_FSR;
    freesize = seg_tx_fsr_bounded(sock);
    if ((freesize > 0) && (*len > freesize)) {
        *len = freesize;
    }
    sent = seg_tcp_send_locked(sock, buf, *len, channel);
    if (sent > 0) {
        seg_tcp_mark_activity(channel);
    }
    seg_socket_unlock();

    seg_s2e_last_send_len[channel] = *len;
    seg_s2e_last_send_rc[channel] = sent;
    if (sent > 0) {
        seg_s2e_tx_progress[channel] += (uint32_t)sent;
    }

    return sent;
}

static uint16_t seg_socket_rx_available(uint8_t sock, int channel) {
    uint16_t available;

    (void)channel;
    seg_socket_lock();
    seg_sock_lock_step = SEG_SOCK_STEP_RX_AVAIL;
    available = getSn_RX_RSR(sock);
    seg_socket_unlock();

    return available;
}

static int16_t seg_socket_recv(uint8_t sock, uint8_t *buf, uint16_t len,
                               int channel) {
    int16_t received;
    uint16_t reg_val = SIK_RECEIVED & 0x00FF;

    seg_socket_lock();
    seg_sock_lock_step = SEG_SOCK_STEP_RECV;
    received = (int16_t)recv(sock, buf, len);
    ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);
    if (received > 0) {
        seg_tcp_mark_activity(channel);
    }
    seg_socket_unlock();

    return received;
}

static int16_t seg_socket_recvfrom(uint8_t sock, uint8_t *buf, uint16_t len,
                                   uint8_t *addr, uint16_t *port, int channel) {
    int16_t received;
    uint16_t reg_val = SIK_RECEIVED & 0x00FF;

    seg_socket_lock();
    received = (int16_t)recvfrom(sock, buf, len, addr, port);
    ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);
    if (received > 0) {
        seg_tcp_mark_activity(channel);
    }
    seg_socket_unlock();

    return received;
}

static int16_t seg_socket_sendto(uint8_t sock, uint8_t *buf, uint16_t len,
                                 uint8_t *addr, uint16_t port, int channel) {
    int16_t sent = (int16_t)seg_wizchip_udp_send_nonblocking(sock, buf, len,
                   addr, port);
    if (sent > 0) {
        seg_tcp_mark_activity(channel);
    }

    return sent;
}

static int8_t seg_socket_getopt(uint8_t sock, sockopt_type option, void *value) {
    int8_t result;

    seg_socket_lock();
    result = getsockopt(sock, option, value);
    seg_socket_unlock();

    return result;
}

uint8_t check_connect_pw_auth(int channel, uint8_t *buf, uint16_t len,
                              uint16_t *consumed);
uint8_t check_tcp_connect_exception(int channel);
void reset_SEG_timeflags(uint8_t channel);

uint16_t get_tcp_any_port(void);

static uint32_t seg_keepalive_interval_ms(uint32_t value_ms) {
    return (value_ms < SEG_KEEPALIVE_MIN_INTERVAL_MS) ?
           SEG_KEEPALIVE_MIN_INTERVAL_MS : value_ms;
}

static TickType_t seg_keepalive_interval_ticks(uint32_t value_ms) {
    TickType_t ticks = pdMS_TO_TICKS(seg_keepalive_interval_ms(value_ms));

    return (ticks == 0) ? 1 : ticks;
}

static BaseType_t ensure_channel_timer(TimerHandle_t *timer,
                                       const char *name,
                                       TickType_t period,
                                       UBaseType_t auto_reload,
                                       int channel,
                                       TimerCallbackFunction_t callback) {
    if ((timer == NULL) || (channel < 0) || (channel >= DEVICE_UART_CNT)) {
        return pdFAIL;
    }

    if (*timer == NULL) {
        *timer = xTimerCreate(name,
                              period,
                              auto_reload,
                              (void *)(uintptr_t)channel,
                              callback);
    }

    return (*timer == NULL) ? pdFAIL : pdPASS;
}

// Drain what the peer sent before its FIN out to the UART, then let the caller
// disconnect. Bounded, because the loop it replaces was not:
//
//     while (getSn_RX_RSR(sock) || e2u_size[channel]) {
//         ether_to_uart(sock, channel);
//     }
//
// That runs inside do_seg(), which holds seg_critical_sem, and ether_to_uart()
// waits on the TX DMA with no timeout of its own. A peer that stops accepting
// serial data therefore parks do_seg() for good: seg_ch_u2e_task never gets the
// semaphore, the ring buffer is never drained, RTS is never reasserted - and
// check_uart_flow_control() sits at the end of do_seg(), so it is never reached
// either. Observed as a permanently stalled channel with a full ring buffer and
// hundreds of unconsumed seg_u2e_sem wake-ups.
//
// This is the connection teardown path, so bounding it cannot slow the steady
// state. Data still in flight when the budget runs out is dropped, which is what
// already happened to it when the channel stalled - except the channel now
// recovers.
#define SEG_CLOSE_WAIT_DRAIN_MS 500U

static void drain_socket_to_uart(uint8_t sock, int channel) {
    uint32_t started = millis();


    while (getSn_RX_RSR(sock) || e2u_size[channel]) {
        ether_to_uart(sock, channel);    // receive remaining packets

        if ((uint32_t)(millis() - started) >= SEG_CLOSE_WAIT_DRAIN_MS) {
            PRT_SEG(" > SEG:CLOSE_WAIT drain gave up ch%d rx=%d e2u=%d\r\n",
                    channel, (int)getSn_RX_RSR(sock), (int)e2u_size[channel]);
            break;
        }
    }

}

// The RX ISR is the only producer of seg_u2e_sem while packing_time is 0, which
// leaves the time delimiter timer disarmed, and flow control silences that ISR
// as soon as the ring buffer crosses UART_OFF_THRESHOLD. seg_ch_u2e_task takes
// the semaphore before it knows whether it can drain, so a pass that finds
// send() short of TX space, or the channel not yet ST_CONNECT, consumes a
// wake-up for good. Once the count reaches zero with the ring buffer above the
// threshold there is no producer left: RTS stays deasserted, no byte arrives, no
// wake-up is generated, and the channel never resumes.
//
// Hand the wake-up back from the seg task, which keeps running at its own
// priority every 10 ms and already holds seg_critical_sem here. Only when the
// count is zero, so a channel whose send() keeps failing does not accumulate a
// backlog of wake-ups to burn through once it recovers. Restricted to
// SEG_SERIAL_PROTOCOL_NONE because the Modbus branches of seg_ch_u2e_task act on
// every wake-up without checking for buffered data.
static void restore_u2e_wakeup(int channel) {
    if (get_serial_communation_protocol(channel) != SEG_SERIAL_PROTOCOL_NONE) {
        return;
    }

    if (uxSemaphoreGetCount(seg_u2e_sem[channel]) != 0) {
        return;
    }

    if (get_data_buffer_usedsize(channel) || u2e_size[channel]) {
        xSemaphoreGive(seg_u2e_sem[channel]);
    }
}

/* Public & Private functions ------------------------------------------------*/

void do_seg(uint8_t sock, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);


    if (opmode == DEVICE_GW_MODE) {
        switch (network_connection->working_mode) {
        case TCP_CLIENT_MODE:
            proc_SEG_tcp_client(sock, channel);
            break;

        case TCP_SERVER_MODE:
            proc_SEG_tcp_server(sock, channel);
            break;

        case TCP_MIXED_MODE:
            proc_SEG_tcp_mixed(sock, channel);
            break;

        case UDP_MODE:
            proc_SEG_udp(sock, channel);
            break;

#ifdef __USE_S2E_OVER_TLS__
        case SSL_TCP_CLIENT_MODE:
            proc_SEG_tcp_client_over_tls(sock, channel);
            break;
#endif

        case MQTT_CLIENT_MODE:
            proc_SEG_mqtt_client(sock, channel);
            break;

#ifdef __USE_S2E_OVER_TLS__
        case MQTTS_CLIENT_MODE:
            proc_SEG_mqtts_client(sock, channel);
            break;
#endif

        default:
            break;
        }

        // XON/XOFF Software flow control: Check the Buffer usage and Send the start/stop commands
        // [WIZnet Device] -> [Peer]
        if ((serial_option->flow_control == flow_xon_xoff) ||
                (serial_option->flow_control == flow_rts_cts) ||
                (serial_option->flow_control == flow_dtr_dsr)) {
            check_uart_flow_control(serial_option->flow_control, channel);
        }

        restore_u2e_wakeup(channel);
    }

}

void set_device_status(teDEVSTATUS status, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __device_option *device_option = (struct __device_option *) & (get_DevConfig_pointer()->device_option);
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    uint8_t prev_working_state = network_connection->working_state;

    switch (status) {
    case ST_BOOT:       // Boot Mode
        network_connection->working_state = ST_BOOT;
        break;

    case ST_OPEN:       // TCP connection state: disconnected (or UDP mode)
        network_connection->working_state = ST_OPEN;
        break;

    case ST_CONNECT:    // TCP connection state: connected
        network_connection->working_state = ST_CONNECT;
        break;

    case ST_UPGRADE:    // TCP connection state: disconnected
        network_connection->working_state = ST_UPGRADE;
        break;

    case ST_ATMODE:     // TCP connection state: disconnected
        network_connection->working_state = ST_ATMODE;
        break;

    case ST_UDP:        // UDP mode
        network_connection->working_state = ST_UDP;
    default:
        break;
    }

    if ((network_connection->working_state == ST_CONNECT) &&
            (prev_working_state != ST_CONNECT)) {
        seg_tcp_mark_activity(channel);
        if ((tcp_option->keepalive_en == ENABLE) &&
                (seg_keepalive_timer[channel] != NULL)) {
            // Start at connection establishment, not at the first S2E packet.
            // Otherwise an idle or E2S-only server connection never probes a
            // peer that disappeared without FIN/RST.
            flag_first_keepalive[channel] = ENABLE;
            xTimerChangePeriod(seg_keepalive_timer[channel],
                               seg_keepalive_interval_ticks(
                                   tcp_option->keepalive_wait_time), 0);
            xTimerStart(seg_keepalive_timer[channel], 0);
        }
    } else if (network_connection->working_state != ST_CONNECT) {
        seg_tcp_last_activity_ms[channel] = 0;
    }

    // Status indicator pins
    if (network_connection->working_state == ST_CONNECT) {
        if (device_option->device_eth_connect_data[channel][0] != 0) {
            struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option[channel]);

            if (network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE) {
                wizchip_mqtt_publish(&g_mqtt_config[channel], mqtt_option->pub_topic, mqtt_option->qos, device_option->device_eth_connect_data[channel], strlen((char *)device_option->device_eth_connect_data[channel]));
            }
#ifdef __USE_S2E_OVER_TLS__
            else if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                wiz_tls_write(&s2e_tlsContext[channel], device_option->device_eth_connect_data[channel], strlen((char *)device_option->device_eth_connect_data[channel]));
            }
#endif
            else {
                (void)seg_socket_send(seg_data_sock[channel],
                                      device_option->device_eth_connect_data[channel],
                                      strlen((char *)device_option->device_eth_connect_data[channel]),
                                      channel);
            }
        }

        if (device_option->device_serial_connect_data[channel][0] != 0) {
            platform_uart_puts((const char *)device_option->device_serial_connect_data[channel], strlen((const char *)device_option->device_serial_connect_data[channel]), channel);
        }

        set_connection_status_io(seg_status_pin[channel], ON);    // Status I/O pin to low
    }

    else if (prev_working_state == ST_CONNECT && network_connection->working_state == ST_OPEN) {
        if (device_option->device_serial_disconnect_data[channel][0] != 0) {
            platform_uart_puts((const char *)device_option->device_serial_disconnect_data[channel], strlen((const char *)device_option->device_serial_disconnect_data[channel]), channel);
        }
        // Status indicator pins
        set_connection_status_io(seg_status_pin[channel], OFF);    // Status I/O pin to high
    } else {
        set_connection_status_io(seg_status_pin[channel], OFF);    // Status I/O pin to high
    }
}

uint8_t get_device_status(int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    return network_connection->working_state;
}


void proc_SEG_udp(uint8_t sock, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol(channel);

    // Socket state
    uint8_t state = getSn_SR(sock);

    uint8_t flag = 0;

    switch (state) {
    case SOCK_UDP:
        seg_udp_transient_reset(channel);
        break;

    case SOCK_CLOSED:
        seg_udp_transient_reset(channel);
        seg_udp_peer_reset(channel);
        seg_wizchip_udp_send_reset(sock);
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            // UART Ring buffer clear
            data_buffer_flush(channel);
        }

        u2e_size[channel] = 0;
        e2u_size[channel] = 0;

        // If remote ip is multicast address, enable the multicasting
        if (network_connection->remote_ip[0] >= 224 && network_connection->remote_ip[0] <= 239) {
            uint8_t multicast_mac[6];
            multicast_mac[0] = 0x01;
            multicast_mac[1] = 0x00;
            multicast_mac[2] = 0x5e;
            multicast_mac[3] = network_connection->remote_ip[1] & 0x7F;
            multicast_mac[4] = network_connection->remote_ip[2];
            multicast_mac[5] = network_connection->remote_ip[3];
            setSn_DIPR(sock, network_connection->remote_ip);
            setSn_DPORT(sock, network_connection->remote_port);
            setSn_DHAR(sock, multicast_mac);
            flag |= SF_MULTI_ENABLE;
        }
        // Keep the socket in non-blocking mode for RX/TX-space checks. UDP SEND
        // completion itself is handled by seg_wizchip_udp_send_nonblocking(),
        // because ioLibrary sendto() still waits for SENDOK even with this flag.
        flag |= SF_IO_NONBLOCK;
        int8_t s = seg_socket_open(sock, Sn_MR_UDP, network_connection->local_port, flag);

        if (s == sock) {
            set_device_status(ST_UDP, channel);

            if (serial_data_packing->packing_time) {
                modeswitch_gap_time = serial_data_packing->packing_time; // replace the GAP time (default: 500ms)
            }

            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:UDP_MODE:SOCKOPEN\r\n");
            }
        }
        break;

    default:
        if (seg_udp_transient_timed_out(channel, state) == TRUE) {
            PRT_SEG(" > SEG:UDP_MODE:STATE_RECOVERY ch=%d\r\n", channel);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            set_device_status(ST_OPEN, channel);
            seg_udp_peer_reset(channel);
            seg_udp_transient_reset(channel);
        }
        break;
    }
}

void proc_SEG_tcp_client(uint8_t sock, int channel) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __serial_command *serial_command = (struct __serial_command *) & (get_DevConfig_pointer()->serial_command);

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol(channel);

    // Socket state
    uint8_t state = getSn_SR(sock);
    int ret;
    uint16_t reg_val;

    switch (state) {
    case SOCK_INIT:
        if (tcp_option->reconnection) {
            vTaskDelay(tcp_option->reconnection);
        }
        // TCP connect exception checker; e.g., dns failed / zero srcip ... and etc.
        if (check_tcp_connect_exception(channel) == ON) {
            return;
        }
        // TCP connect
        ret = seg_socket_connect(sock, network_connection->remote_ip, network_connection->remote_port);
        PRT_SEG(" > SEG:TCP_CLIENT_MODE:ConnectNetwork Err %d\r\n", ret);

#ifdef _SEG_DEBUG_
        PRT_SEG(" > SEG:TCP_CLIENT_MODE:CLIENT_CONNECTION\r\n");
#endif
        break;

    case SOCK_ESTABLISHED:
        if ((getSn_IR(sock) & Sn_IR_CON) ||
                (get_device_status(channel) != ST_CONNECT)) {
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // S2E: TCP client mode initialize after connection established (only once)
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // Interrupt clear
            reg_val = SIK_CONNECTED & 0x00FF; // except SIK_SENT(send OK) interrupt
            ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                seg_socket_getopt(sock, SO_DESTIP, &destip);
                seg_socket_getopt(sock, SO_DESTPORT, &destport);
                PRT_SEG(" > SEG:CONNECTED TO - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
            }

            if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
                data_buffer_flush(channel);    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity[channel] = SEG_DISABLE;
                if (ensure_channel_timer(&seg_inactivity_timer[channel],
                                         "seg_inactivity_timer",
                                         pdMS_TO_TICKS(tcp_option->inactivity * 1000),
                                         pdFALSE,
                                         channel,
                                         inactivity_timer_callback) == pdPASS) {
                    xTimerStart(seg_inactivity_timer[channel], 0);
                }
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer[channel] == NULL) {
                    ensure_channel_timer(&seg_keepalive_timer[channel],
                                         "seg_keepalive_timer",
                                         seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time), 0);
                }
            }
            set_device_status(ST_CONNECT, channel);
        }
        break;

    case SOCK_CLOSE_WAIT:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        seg_socket_disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        set_device_status(ST_OPEN, channel);
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);

        u2e_size[channel] = 0;
        e2u_size[channel] = 0;

        if (network_connection->fixed_local_port) {
            source_port = network_connection->local_port;
        } else {
            source_port = get_tcp_any_port();
        }


#ifdef _SEG_DEBUG_
        PRT_SEG(" > TCP CLIENT: client_any_port = %d\r\n", client_any_port);
#endif
        int8_t s = seg_socket_open(sock, Sn_MR_TCP, source_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

        if (s == sock) {
            if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                modeswitch_gap_time = serial_data_packing->packing_time;
            }

            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:TCP_CLIENT_MODE:SOCKOPEN\r\n");
            }

        } else {
            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:TCP_CLIENT_MODE:SOCKOPEN FAILED\r\n");
            }
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        }
        break;

    default:
        break;
    }
}

#ifdef __USE_S2E_OVER_TLS__
void proc_SEG_tcp_client_over_tls(uint8_t sock, int channel) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __network_option *network_option = (struct __network_option *) & (get_DevConfig_pointer()->network_option);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol(channel);

    // Socket state
    uint8_t state = getSn_SR(sock);

    int ret = 0;
    uint16_t reg_val;
    // Connection setup is independent for every data channel.  A single static
    // flag lets one channel consume another channel's TLS-established event.
    static uint8_t first_established[DEVICE_UART_CNT] = {0, };

    switch (state) {
    case SOCK_INIT:
        if (tcp_option->reconnection) {
            vTaskDelay(tcp_option->reconnection);
        }
        // TCP connect exception checker; e.g., dns failed / zero srcip ... and etc.
        if (check_tcp_connect_exception(channel) == ON) {
            return;
        }

        reg_val = 0;
        ctlsocket(sock, CS_SET_INTMASK, (void *)&reg_val);

        ret = seg_socket_connect(sock, network_connection->remote_ip, network_connection->remote_port);
        PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE:ConnectNetwork Err %d\r\n", ret);

        // Wait for TCP connection with timeout (1s * tcp_rcr_val)
        uint32_t timeout_ticks = pdMS_TO_TICKS(1000 * network_option->tcp_rcr_val);
        uint32_t elapsed_ticks = 0;
        const uint32_t poll_ticks = pdMS_TO_TICKS(10);

        while (elapsed_ticks < timeout_ticks) {
            if (getSn_SR(sock) == SOCK_ESTABLISHED) {
                break;
            }
            vTaskDelay(poll_ticks);
            elapsed_ticks += poll_ticks;
        }

        if (getSn_SR(sock) != SOCK_ESTABLISHED) {
            PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: TCP CONNECT TIMEOUT\r\n");
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            break;
        }

        reg_val = SIK_CONNECTED & 0x00FF;
        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

        ret = wiz_tls_connect(&s2e_tlsContext[channel],
                              (char *)network_connection->remote_ip,
                              (unsigned int)network_connection->remote_port,
                              channel);

#if 1
        reg_val = SIK_RECEIVED & 0x00FF;
        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

        reg_val =  SIK_RECEIVED & 0x00FF; // except SIK_SENT(send OK) interrupt
        ctlsocket(sock, CS_SET_INTMASK, (void *)&reg_val);

        ctlwizchip(CW_GET_INTRMASK, (void *)&reg_val);
#if (_WIZCHIP_ == W5100S)
        reg_val = (1 << sock);
#elif (_WIZCHIP_ == W5500)
        reg_val = ((1 << sock) << 8) | reg_val;
#endif
        ctlwizchip(CW_SET_INTRMASK, (void *)&reg_val);
#endif

        if (ret != 0) { // TLS connection failed
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: CONNECTION FAILED\r\n");
            }
            break;
        }
        first_established[channel] = 1;

        PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: TCP CLIENT CONNECTED\r\n");
        break;

    case SOCK_ESTABLISHED:
        //if(getSn_IR(sock) & Sn_IR_CON)
        if (first_established[channel]) {
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // S2E: TCP client mode initialize after connection established (only once)
            ///////////////////////////////////////////////////////////////////////////////////////////////////

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                seg_socket_getopt(sock, SO_DESTIP, &destip);
                seg_socket_getopt(sock, SO_DESTPORT, &destport);
                PRT_SEG(" > SEG:CONNECTED TO - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
            }

            if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
                data_buffer_flush(channel);    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity[channel] = SEG_DISABLE;
                if (ensure_channel_timer(&seg_inactivity_timer[channel],
                                         "seg_inactivity_timer",
                                         pdMS_TO_TICKS(tcp_option->inactivity * 1000),
                                         pdFALSE,
                                         channel,
                                         inactivity_timer_callback) == pdPASS) {
                    xTimerStart(seg_inactivity_timer[channel], 0);
                }
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer[channel] == NULL) {
                    ensure_channel_timer(&seg_keepalive_timer[channel],
                                         "seg_keepalive_timer",
                                         seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time), 0);
                }
            }

            first_established[channel] = 0;
            set_device_status(ST_CONNECT, channel);
        }
        break;

    case SOCK_CLOSE_WAIT:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        seg_socket_disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        set_device_status(ST_OPEN, channel);
        if (wiz_tls_init(&s2e_tlsContext[channel], (int *)sock, channel) > 0) {
            u2e_size[channel] = 0;
            e2u_size[channel] = 0;

            if (network_connection->fixed_local_port) {
                source_port = network_connection->local_port;
            } else {
                source_port = get_tcp_any_port();
            }

            PRT_SEG(" > TCP CLIENT over TLS: client_any_port = %d\r\n", client_any_port);
            int s = wiz_tls_socket(&s2e_tlsContext[channel], sock, source_port);
            if (s == sock) {
                // Replace the command mode switch code GAP time (default: 500ms)
                if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                    modeswitch_gap_time = serial_data_packing->packing_time;
                }

                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE:SOCKOPEN\r\n");
                }
                set_wiz_tls_init_state(ENABLE, channel);
            } else {
                PRT_SEG("wiz_tls_socket() failed\r\n");
                wiz_tls_deinit(&s2e_tlsContext[channel]);
                set_wiz_tls_init_state(DISABLE, channel);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            }
        } else {
            PRT_SEG("wiz_tls_init() failed\r\n");
            wiz_tls_deinit(&s2e_tlsContext[channel]);
            set_wiz_tls_init_state(DISABLE, channel);
        }
        break;

    default:
        break;
    }
}

#endif


void proc_SEG_mqtt_client(uint8_t sock, int channel) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __network_option *network_option = (struct __network_option *) & (get_DevConfig_pointer()->network_option);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;
    struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option[channel]);

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    uint8_t serial_mode = get_serial_communation_protocol(channel);
    uint8_t state = getSn_SR(sock);
    int ret;
    uint16_t reg_val;
    // MQTT connection completion must not be shared between channels.
    static uint8_t first_established[DEVICE_UART_CNT] = {0, };

    switch (state) {
    case SOCK_INIT:
        if (tcp_option->reconnection) {
            vTaskDelay(tcp_option->reconnection);
        }

        // MQTT connect exception checker; e.g., dns failed / zero srcip ... and etc.
        if (check_tcp_connect_exception(channel) == ON) {
            return;
        }

        reg_val = 0;
        ctlsocket(sock, CS_SET_INTMASK, (void *)&reg_val);

        // MQTT connect
        ret = seg_socket_connect(sock, network_connection->remote_ip, network_connection->remote_port);
        PRT_SEG(" > SEG:MQTT_CLIENT_MODE:ConnectNetwork Err %d\r\n", ret);

        // Wait for TCP connection with timeout (1s * tcp_rcr_val)
        uint32_t timeout_ticks = pdMS_TO_TICKS(1000 * network_option->tcp_rcr_val);
        uint32_t elapsed_ticks = 0;
        const uint32_t poll_ticks = pdMS_TO_TICKS(10);

        while (elapsed_ticks < timeout_ticks) {
            if (getSn_SR(sock) == SOCK_ESTABLISHED) {
                break;
            }
            vTaskDelay(poll_ticks);
            elapsed_ticks += poll_ticks;
        }

        if (getSn_SR(sock) != SOCK_ESTABLISHED) {
            PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: TCP CONNECT TIMEOUT\r\n");
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            break;
        }
        PRT_SEG(" > SEG:MQTT_CLIENT_MODE:TCP_CONNECTION\r\n");

        reg_val = SIK_ALL & 0x00FF;
        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

        ret = mqtt_transport_connect(&g_mqtt_config[channel], tcp_option->reconnection);
        if (ret < 0) {
            PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTConnect Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            break;
        }
        PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTT_CONNECTION\r\n");

        if (mqtt_option->sub_topic_0[0] != 0 && mqtt_option->sub_topic_0[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config[channel], mqtt_option->qos, (char *)mqtt_option->sub_topic_0);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_1[0] != 0 && mqtt_option->sub_topic_1[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config[channel], mqtt_option->qos, (char *)mqtt_option->sub_topic_1);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_2[0] != 0 && mqtt_option->sub_topic_2[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config[channel], mqtt_option->qos, (char *)mqtt_option->sub_topic_2);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                break;
            }
        }
        PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTSubscribed\r\n");
        first_established[channel] = 1;
        break;

    case SOCK_ESTABLISHED:
        if (first_established[channel]) {
            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                seg_socket_getopt(sock, SO_DESTIP, &destip);
                seg_socket_getopt(sock, SO_DESTPORT, &destport);
                PRT_SEG(" > SEG:CONNECTED TO - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
            }

            if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
                data_buffer_flush(channel);    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity[channel] = SEG_DISABLE;
                if (ensure_channel_timer(&seg_inactivity_timer[channel],
                                         "seg_inactivity_timer",
                                         pdMS_TO_TICKS(tcp_option->inactivity * 1000),
                                         pdFALSE,
                                         channel,
                                         inactivity_timer_callback) == pdPASS) {
                    xTimerStart(seg_inactivity_timer[channel], 0);
                }
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer[channel] == NULL) {
                    ensure_channel_timer(&seg_keepalive_timer[channel],
                                         "seg_keepalive_timer",
                                         seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time), 0);
                }
            }
            first_established[channel] = 0;
            set_device_status(ST_CONNECT, channel);
        }
        mqtt_transport_yield(&g_mqtt_config[channel]);
        break;

    case SOCK_CLOSE_WAIT:
        seg_socket_disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        set_device_status(ST_OPEN, channel);

        u2e_size[channel] = 0;
        e2u_size[channel] = 0;

        if (network_connection->fixed_local_port) {
            source_port = network_connection->local_port;
        } else {
            source_port = get_tcp_any_port();
        }

        PRT_SEG(" > MQTT CLIENT: client_any_port = %d\r\n", client_any_port);
        int8_t s = seg_socket_open(sock, Sn_MR_TCP, source_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

        if (s == sock) {
            // Replace the command mode switch code GAP time (default: 500ms)
            if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                modeswitch_gap_time = serial_data_packing->packing_time;
            }

            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:SOCKOPEN\r\n");
            }
        } else {
            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:SOCKOPEN FAILED\r\n");
            }
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            break;
        }
        void (*sub_callback)(uint8_t *, uint32_t);
        // Select per-channel MQTT subscribe callback.
        switch (channel) {
        case SEG_DATA0_CH: sub_callback = mqtt_subscribeMessageHandler0; break;
        case SEG_DATA1_CH: sub_callback = mqtt_subscribeMessageHandler1; break;
#if (DEVICE_UART_CNT > 2)
        case SEG_DATA2_CH: sub_callback = mqtt_subscribeMessageHandler2; break;
#endif
#if (DEVICE_UART_CNT > 3)
        case SEG_DATA3_CH: sub_callback = mqtt_subscribeMessageHandler3; break;
#endif
        default: sub_callback = mqtt_subscribeMessageHandler0; break;
        }
        ret = mqtt_transport_init(sock, &g_mqtt_config[channel], true, 1, g_recv_mqtt_buf[channel],
                                  DATA_BUF_SIZE, &g_transport_interface[channel], &g_network_context[channel],
                                  mqtt_option->client_id, mqtt_option->user_name, mqtt_option->password, mqtt_option->keepalive,
                                  sub_callback, channel);

        if (ret < 0) {
            PRT_SEG(" > SEG:MQTT_CLIENT_MODE:INITIALIZE FAILED\r\n");
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        }

        break;

    default:
        break;
    }
}

#ifdef __USE_S2E_OVER_TLS__
void proc_SEG_mqtts_client(uint8_t sock, int channel) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __network_option *network_option = (struct __network_option *) & (get_DevConfig_pointer()->network_option);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;
    struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option[channel]);

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    uint8_t serial_mode = get_serial_communation_protocol(channel);
    uint8_t state = getSn_SR(sock);
    int ret;
    uint16_t reg_val;
    // MQTTS connection completion must not be shared between channels.
    static uint8_t first_established[DEVICE_UART_CNT] = {0, };

    switch (state) {
    case SOCK_INIT:
        if (tcp_option->reconnection) {
            vTaskDelay(tcp_option->reconnection);
        }
        // MQTT connect exception checker; e.g., dns failed / zero srcip ... and etc.
        if (check_tcp_connect_exception(channel) == ON) {
            return;
        }

        reg_val = 0;
        ctlsocket(sock, CS_SET_INTMASK, (void *)&reg_val);

        ret = seg_socket_connect(sock, network_connection->remote_ip, network_connection->remote_port);
        PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE:ConnectNetwork Err %d\r\n", ret);

        // Wait for TCP connection with timeout (1s * tcp_rcr_val)
        uint32_t timeout_ticks = pdMS_TO_TICKS(1000 * network_option->tcp_rcr_val);
        uint32_t elapsed_ticks = 0;
        const uint32_t poll_ticks = pdMS_TO_TICKS(10);

        while (elapsed_ticks < timeout_ticks) {
            if (getSn_SR(sock) == SOCK_ESTABLISHED) {
                break;
            }
            vTaskDelay(poll_ticks);
            elapsed_ticks += poll_ticks;
        }

        if (getSn_SR(sock) != SOCK_ESTABLISHED) {
            PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: TCP CONNECT TIMEOUT\r\n");
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            break;
        }
        reg_val = SIK_ALL & 0x00FF;
        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

        PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: TCP CLIENT CONNECTED\r\n");

        ret = wiz_tls_connect(&s2e_tlsContext[channel],
                              (char *)network_connection->remote_ip,
                              (unsigned int)network_connection->remote_port,
                              channel);

        if (ret != 0) { // TLS connection failed
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE: CONNECTION FAILED\r\n");
            }
            break;
        }
        PRT_SEG(" > SEG:MQTTS_CLIENT_MODE: SSL CLIENT CONNECTED\r\n");

        // MQTTS connect
        ret = mqtt_transport_connect(&g_mqtt_config[channel], tcp_option->reconnection);
        if (ret < 0) {
            PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:ConnectNetwork Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            break;
        }

        PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTT_CONNECTION\r\n");

        if (mqtt_option->sub_topic_0[0] != 0 && mqtt_option->sub_topic_0[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config[channel], mqtt_option->qos, (char *)mqtt_option->sub_topic_0);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_1[0] != 0 && mqtt_option->sub_topic_1[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config[channel], mqtt_option->qos, (char *)mqtt_option->sub_topic_1);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_2[0] != 0 && mqtt_option->sub_topic_2[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config[channel], mqtt_option->qos, (char *)mqtt_option->sub_topic_2);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                break;
            }
        }
        PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTTSubscribed\r\n");
        first_established[channel] = 1;
        break;

    case SOCK_ESTABLISHED:
        if (first_established[channel]) {
            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                seg_socket_getopt(sock, SO_DESTIP, &destip);
                seg_socket_getopt(sock, SO_DESTPORT, &destport);
                PRT_SEG(" > SEG:CONNECTED TO - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
            }

            if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
                data_buffer_flush(channel);
            }

            if (tcp_option->inactivity) {
                flag_inactivity[channel] = SEG_DISABLE;
                if (ensure_channel_timer(&seg_inactivity_timer[channel],
                                         "seg_inactivity_timer",
                                         pdMS_TO_TICKS(tcp_option->inactivity * 1000),
                                         pdFALSE,
                                         channel,
                                         inactivity_timer_callback) == pdPASS) {
                    xTimerStart(seg_inactivity_timer[channel], 0);
                }
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer[channel] == NULL) {
                    ensure_channel_timer(&seg_keepalive_timer[channel],
                                         "seg_keepalive_timer",
                                         seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time), 0);
                }
            }

            first_established[channel] = 0;
            set_device_status(ST_CONNECT, channel);
        }
        mqtt_transport_yield(&g_mqtt_config[channel]);
        break;

    case SOCK_CLOSE_WAIT:
        seg_socket_disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        set_device_status(ST_OPEN, channel);

        if (wiz_tls_init(&s2e_tlsContext[channel], (int *)sock, channel) > 0) {

            u2e_size[channel] = 0;
            e2u_size[channel] = 0;

            if (network_connection->fixed_local_port) {
                source_port = network_connection->local_port;
            } else {
                source_port = get_tcp_any_port();
            }

            PRT_SEG(" > MQTTS_CLIENT_MODE:client_any_port = %d\r\n", client_any_port);

            int s = wiz_tls_socket(&s2e_tlsContext[channel], sock, source_port);
            if (s == sock) {
                // Replace the command mode switch code GAP time (default: 500ms)
                if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                    modeswitch_gap_time = serial_data_packing->packing_time;
                }

                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:SOCKOPEN\r\n");
                }
                set_wiz_tls_init_state(ENABLE, channel);
            } else {
                PRT_SEG("wiz_tls_socket() failed\r\n");
                wiz_tls_deinit(&s2e_tlsContext[channel]);
                set_wiz_tls_init_state(DISABLE, channel);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            }

            void (*sub_callback)(uint8_t *, uint32_t);
            // Select per-channel MQTT subscribe callback.
            switch (channel) {
            case SEG_DATA0_CH: sub_callback = mqtt_subscribeMessageHandler0; break;
            case SEG_DATA1_CH: sub_callback = mqtt_subscribeMessageHandler1; break;
#if (DEVICE_UART_CNT > 2)
            case SEG_DATA2_CH: sub_callback = mqtt_subscribeMessageHandler2; break;
#endif
#if (DEVICE_UART_CNT > 3)
            case SEG_DATA3_CH: sub_callback = mqtt_subscribeMessageHandler3; break;
#endif
            default: sub_callback = mqtt_subscribeMessageHandler0; break;
            }
            ret = mqtt_transport_init(sock, &g_mqtt_config[channel], true, 1, g_recv_mqtt_buf[channel],
                                      DATA_BUF_SIZE, &g_transport_interface[channel], &g_network_context[channel],
                                      mqtt_option->client_id, mqtt_option->user_name, mqtt_option->password, mqtt_option->keepalive,
                                      sub_callback, channel);
            if (ret < 0) {
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:INITIALIZE FAILED\r\n");
            }
        } else {
            PRT_SEGCP("wiz_tls_init() failed\r\n");
            wiz_tls_deinit(&s2e_tlsContext[channel]);
            set_wiz_tls_init_state(DISABLE, channel);
        }
        break;

    default:
        break;
    }
}
#endif // __USE_S2E_OVER_TLS__

void proc_SEG_tcp_server(uint8_t sock, int channel) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __serial_command *serial_command = (struct __serial_command *) & (get_DevConfig_pointer()->serial_command);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);

    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol(channel);

    // Socket state
    uint8_t state = getSn_SR(sock);
    uint16_t reg_val;

    switch (state) {
    case SOCK_INIT: {
        int8_t listen_rc;
        uint8_t listen_state;

        /* Recover a server left in INIT instead of ignoring it forever. */
        listen_rc = seg_socket_listen(sock);
        listen_state = getSn_SR(sock);
        if ((listen_rc != SOCK_OK) || (listen_state != SOCK_LISTEN)) {
            PRT_SEG(" > SEG:TCP_SERVER_MODE:LISTEN_RECOVERY_FAILED ch=%d\r\n", channel);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        }
        seg_tcp_server_transient_reset(channel);
        break;
    }

    case SOCK_LISTEN:
        seg_tcp_server_transient_reset(channel);
        break;

    case SOCK_ESTABLISHED:
        seg_tcp_server_transient_reset(channel);
        if ((getSn_IR(sock) & Sn_IR_CON) ||
                (get_device_status(channel) != ST_CONNECT)) {
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // S2E: TCP server mode initialize after connection established (only once)
            ///////////////////////////////////////////////////////////////////////////////////////////////////

            // Interrupt clear
            // setSn_IR(sock, Sn_IR_CON);
            // reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT) & 0x00FF; // except SIK_SENT(send OK) interrupt
            reg_val = SIK_CONNECTED & 0x00FF; // except SIK_SENT(send OK) interrupt
            ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                seg_socket_getopt(sock, SO_DESTIP, &destip);
                seg_socket_getopt(sock, SO_DESTPORT, &destport);
                PRT_SEG(" > SEG:CONNECTED ch=%d FROM - %d.%d.%d.%d : %d\r\n",
                        channel,
                        destip[0], destip[1], destip[2], destip[3], destport);
            }

            if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
                data_buffer_flush(channel);    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity[channel] = SEG_DISABLE;
                if (ensure_channel_timer(&seg_inactivity_timer[channel],
                                         "seg_inactivity_timer",
                                         pdMS_TO_TICKS(tcp_option->inactivity * 1000),
                                         pdFALSE,
                                         channel,
                                         inactivity_timer_callback) == pdPASS) {
                    xTimerStart(seg_inactivity_timer[channel], 0);
                }
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer[channel] == NULL) {
                    ensure_channel_timer(&seg_keepalive_timer[channel],
                                         "seg_keepalive_timer",
                                         seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time), 0);
                }
            }

            if (tcp_option->pw_connect_en) { // TCP server mode only (+ mixed_server)
                flag_auth_time[channel] = SEG_DISABLE;
                if (ensure_channel_timer(&seg_auth_timer[channel],
                                         "seg_auth_timer",
                                         pdMS_TO_TICKS(MAX_CONNECTION_AUTH_TIME),
                                         pdFALSE,
                                         channel,
                                         auth_timer_callback) == pdPASS) {
                    xTimerStart(seg_auth_timer[channel], 0);
                }
            }
            set_device_status(ST_CONNECT, channel);
        }
        break;

    case SOCK_CLOSE_WAIT:
        seg_tcp_server_transient_reset(channel);
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        seg_socket_disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        seg_tcp_server_transient_reset(channel);
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        set_device_status(ST_OPEN, channel);

        u2e_size[channel] = 0;
        e2u_size[channel] = 0;

        // Opened blocking, send() had no exit from its TX-free-space wait loop
        // (socket.c only returns SOCK_BUSY there when SF_IO_NONBLOCK is set), so
        // the task spun on the SPI bus and starved the other channels. With
        // non-blocking, send() returns SOCK_BUSY and uart_to_ether() retries on
        // the next pass; u2e_size and g_send_buf are left intact for that.
        //
        // Original:
        //     int8_t s = socket(sock, Sn_MR_TCP, network_connection->local_port, 0x00);
        int8_t s = seg_socket_open(sock, Sn_MR_TCP, network_connection->local_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

        if (s == sock) {
            // Replace the command mode switch code GAP time (default: 500ms)
            if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                modeswitch_gap_time = serial_data_packing->packing_time;
            }

            // TCP Server listen.  A successful socket() only reaches INIT; do
            // not report SOCKOPEN unless listen() actually reached LISTEN.
            int8_t listen_rc = seg_socket_listen(sock);
            uint8_t listen_state = getSn_SR(sock);

            if ((listen_rc == SOCK_OK) && (listen_state == SOCK_LISTEN)) {
                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_SERVER_MODE:SOCKOPEN ch=%d\r\n", channel);
                }
            } else {
                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_SERVER_MODE:SOCKOPEN FAILED ch=%d\r\n", channel);
                }
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            }
        } else {
            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:TCP_SERVER_MODE:SOCKOPEN FAILED ch=%d\r\n", channel);
            }
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        }
        break;

    default:
        if (seg_tcp_server_transient_timed_out(channel, state) == TRUE) {
            PRT_SEG(" > SEG:TCP_SERVER_MODE:STATE_RECOVERY ch=%d\r\n", channel);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            seg_tcp_server_transient_reset(channel);
        }
        break;
    }
}


void proc_SEG_tcp_mixed(uint8_t sock, int channel) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __network_option *network_option = (struct __network_option *) & (get_DevConfig_pointer()->network_option);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);

    uint16_t source_port = 0;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol(channel);

    // Socket state
    uint8_t state = getSn_SR(sock);
    int ret;
    uint16_t reg_val;

#ifdef MIXED_CLIENT_LIMITED_CONNECT
    static uint8_t reconnection_count[DEVICE_UART_CNT] = {0, };
#endif
    switch (state) {
    case SOCK_INIT:
        if (mixed_state[channel] == MIXED_CLIENT) {
            if (reconnection_count[channel] && tcp_option->reconnection) {
                vTaskDelay(tcp_option->reconnection);
            }

            // TCP connect exception checker; e.g., dns failed / zero srcip ... and etc.
            if (check_tcp_connect_exception(channel) == ON) {
#ifdef MIXED_CLIENT_LIMITED_CONNECT
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                reconnection_count[channel] = 0;
                data_buffer_flush(channel);
                mixed_state[channel] = MIXED_SERVER;
#endif
                return;
            }

            // TCP connect
            ret = seg_socket_connect(sock, network_connection->remote_ip, network_connection->remote_port);
            PRT_SEG(" > SEG:TCP_MIXED_MODE:ConnectNetwork Err %d\r\n", ret);

#ifdef MIXED_CLIENT_LIMITED_CONNECT
            reconnection_count[channel]++;

            if (reconnection_count[channel] >= network_option->tcp_rcr_val) {
                PRT_SEG("reconnection_count >= network_option->tcp_rcr_val\r\n");
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                reconnection_count[channel] = 0;
                data_buffer_flush(channel);
                mixed_state[channel] = MIXED_SERVER;
            }
#ifdef _SEG_DEBUG_
            if (reconnection_count[channel] != 0) {
                PRT_SEG(" > SEG:TCP_MIXED_MODE:CLIENT_CONNECTION [%d]\r\n", reconnection_count[channel]);
            } else {
                PRT_SEG(" > SEG:TCP_MIXED_MODE:CLIENT_CONNECTION_RETRY FAILED\r\n");
            }
#endif
#endif
        }
        break;

    case SOCK_LISTEN:
        break;

    case SOCK_ESTABLISHED:
        if ((getSn_IR(sock) & Sn_IR_CON) ||
                (get_device_status(channel) != ST_CONNECT)) {
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // S2E: TCP mixed (server or client) mode initialize after connection established (only once)
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            reg_val = SIK_CONNECTED & 0x00FF;
            ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                seg_socket_getopt(sock, SO_DESTIP, &destip);
                seg_socket_getopt(sock, SO_DESTPORT, &destport);

                if (mixed_state[channel] == MIXED_SERVER) {
                    PRT_SEG(" > SEG:CONNECTED FROM - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
                } else {
                    PRT_SEG(" > SEG:CONNECTED TO - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
                }
            }

            if (tcp_option->inactivity) {
                flag_inactivity[channel] = SEG_DISABLE;
                if (ensure_channel_timer(&seg_inactivity_timer[channel],
                                         "seg_inactivity_timer",
                                         pdMS_TO_TICKS(tcp_option->inactivity * 1000),
                                         pdFALSE,
                                         channel,
                                         inactivity_timer_callback) == pdPASS) {
                    xTimerStart(seg_inactivity_timer[channel], 0);
                }
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer[channel] == NULL) {
                    ensure_channel_timer(&seg_keepalive_timer[channel],
                                         "seg_keepalive_timer",
                                         seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], seg_keepalive_interval_ticks(tcp_option->keepalive_wait_time), 0);
                }
            }

            set_device_status(ST_CONNECT, channel);
            // Check the connection password auth timer
            if (mixed_state[channel] == MIXED_SERVER) {
                // Connection Password option: TCP server mode only (+ mixed_server)
                flag_auth_time[channel] = SEG_DISABLE;
                if (tcp_option->pw_connect_en == SEG_ENABLE) {
                    if (ensure_channel_timer(&seg_auth_timer[channel],
                                             "seg_auth_timer",
                                             pdMS_TO_TICKS(MAX_CONNECTION_AUTH_TIME),
                                             pdFALSE,
                                             channel,
                                             auth_timer_callback) == pdPASS) {
                        xTimerStart(seg_auth_timer[channel], 0);
                    }
                }
            } else {
                if (get_data_buffer_usedsize(channel) || u2e_size[channel]) {
                    xSemaphoreGive(seg_u2e_sem[channel]);
                }
                mixed_state[channel] = MIXED_SERVER;
            }

#ifdef MIXED_CLIENT_LIMITED_CONNECT
            reconnection_count[channel] = 0;
#endif
        }
        break;

    case SOCK_CLOSE_WAIT:
        PRT_SEG("case SOCK_CLOSE_WAIT\r\n");
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        seg_socket_disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        PRT_SEG("case SOCK_FIN_WAIT or SOCK_CLOSED\r\n");
        set_device_status(ST_OPEN, channel);
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);

        if (mixed_state[channel] == MIXED_SERVER) { // MIXED_SERVER
            u2e_size[channel] = 0;
            e2u_size[channel] = 0;
            data_buffer_flush(channel);

            int8_t s = seg_socket_open(sock, Sn_MR_TCP, network_connection->local_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

            if (s == sock) {
                // Replace the command mode switch code GAP time (default: 500ms)
                if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                    modeswitch_gap_time = serial_data_packing->packing_time;
                }

                // TCP Server listen
                seg_socket_listen(sock);

                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_MIXED_MODE:SERVER_SOCKOPEN\r\n");
                }
            } else {
                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_MIXED_MODE:SERVER_SOCKOPEN FAILED\r\n");
                }
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            }
        } else { // MIXED_CLIENT
            PRT_INFO(" > SEG:TCP_MIXED_MODE:CLIENT_SOCKCLOSED\r\n");
            e2u_size[channel] = 0;
            if (network_connection->fixed_local_port) {
                source_port = network_connection->local_port;
            } else {
                source_port = get_tcp_any_port();
            }

#ifdef _SEG_DEBUG_
            PRT_SEG(" > TCP CLIENT: any_port = %d\r\n", source_port);
#endif
            int8_t s = seg_socket_open(sock, Sn_MR_TCP, source_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

            if (s == sock) {
                // Replace the command mode switch code GAP time (default: 500ms)
                if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                    modeswitch_gap_time = serial_data_packing->packing_time;
                }

                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_MIXED_MODE:CLIENT_SOCKOPEN\r\n");
                }
            } else {
                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_MIXED_MODE:CLIENT_SOCKOPEN FAILED\r\n");
                }
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
            }
        }
        break;

    default:
        break;
    }
}

void uart_to_ether(uint8_t sock, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);
    struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option[channel]);

    uint16_t len;
    int16_t sent_len = 0;

    // UART ring buffer -> user's buffer

    // A staged UDP payload is one datagram. Do not append later UART bytes while
    // a previous SEND is still pending/busy; TCP is a byte stream and may keep
    // coalescing its unsent tail as before.
    if ((network_connection->working_mode == UDP_MODE) &&
            (u2e_size[channel] != 0)) {
        len = u2e_size[channel];
    } else {
        len = get_serial_data(channel);
    }

    if (len > 0) {
        serial_input_time[channel] = 0;
        enable_serial_input_timer[channel] = 0;
        flag_serial_input_time_elapse[channel] = SEG_DISABLE;
        if (seg_inactivity_timer[channel] != NULL) {
            xTimerReset(seg_inactivity_timer[channel], 0);
        }
        if ((serial_common->serial_debug_en == SEG_DEBUG_S2E) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
            debugSerial_dataTransfer(g_send_buf[channel], len, SEG_DEBUG_S2E);
        }

        if (network_connection->working_mode == UDP_MODE) {
            if ((network_connection->remote_ip[0] == 0x00) &&
                    (network_connection->remote_ip[1] == 0x00) &&
                    (network_connection->remote_ip[2] == 0x00) &&
                    (network_connection->remote_ip[3] == 0x00)) {
                if (udp_peer_valid[channel] == SEG_DISABLE) {
                    if (serial_common->serial_debug_en) {
                        PRT_SEG(" > SEG:UDP_MODE:DATA SEND FAILED - UDP Peer IP/Port required (0.0.0.0)\r\n");
                    }
                } else {
                    sent_len = seg_socket_sendto(sock, g_send_buf[channel], len,
                                                 udp_peer_ip[channel],
                                                 udp_peer_port[channel],
                                                 channel);    // UDP 1:N mode
                }
            } else {
                sent_len = seg_socket_sendto(sock, g_send_buf[channel], len,
                                             network_connection->remote_ip,
                                             network_connection->remote_port,
                                             channel);    // UDP 1:1 mode
            }
        } else if (network_connection->working_state == ST_CONNECT) {
            if (network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE) {
                sent_len = wizchip_mqtt_publish(&g_mqtt_config[channel], mqtt_option->pub_topic, mqtt_option->qos, g_send_buf[channel], len);
            }
#ifdef __USE_S2E_OVER_TLS__
            else if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                sent_len = wiz_tls_write(&s2e_tlsContext[channel], g_send_buf[channel], len);
            }
#endif
            else {
                // send() is all or nothing: it clamps the request to getSn_TxMAX() and
                // then, on a non-blocking socket, returns SOCK_BUSY until that whole
                // amount is free. DATA_BUF_SIZE equals the 2 KB socket transmit buffer,
                // so a saturated u2e_size asks for every byte of it and only succeeds
                // once nothing is left unacknowledged. get_serial_data() cannot grow the
                // request past that cap to make it fit either, so under sustained
                // traffic - where something is always in flight - the channel stops
                // sending for good: the ring buffer fills behind it and the peer is
                // never released. Offer what the socket has room for and keep the rest.
                //
                // Holding out for a minimum partial size was tried and reverted: it
                // stopped three channels outright at 630 s where taking whatever was
                // free only degraded them. socket.c allows one SEND in flight per
                // socket and returns SOCK_BUSY until SENDOK, so waiting for a larger
                // chunk only lengthens the gap between sends without making the next
                // one bigger.
                //
                // Original:
                //     sent_len = (int16_t)send(sock, g_send_buf[channel], len);
                // send() must be reached on every pass. It is the only thing that
                // acknowledges Sn_IR_SENDOK and clears socket.c's sock_is_sending, and
                // the chip does not refresh Sn_TX_FSR until it has been. Skipping the
                // call when the clamp produced zero left both set and the register
                // stuck at 0 for good - captured as tx_rd == tx_wr, so an empty
                // transmit buffer, reporting no free space, unchanged over twelve
                // minutes with SENDOK still raised.
                //
                // So clamp only when there is space to clamp to. With none reported,
                // hand over the full request: send() runs its SENDOK bookkeeping first
                // and then returns SOCK_BUSY, which is the state the next pass needs.
                sent_len = seg_socket_send_available(sock, g_send_buf[channel],
                                                     &len, channel);
            }

            if ((tcp_option->keepalive_en == ENABLE) &&
                    (flag_first_keepalive[channel] == DISABLE) &&
                    (seg_keepalive_timer[channel] != NULL)) {
                flag_first_keepalive[channel] = ENABLE;
                xTimerStart(seg_keepalive_timer[channel], 0);
            }
        }
        if (sent_len > 0) {
            u2e_size[channel] -= sent_len;
            // A short send leaves a tail, which has to lead the next request.
            if (u2e_size[channel]) {
                memmove(g_send_buf[channel], &g_send_buf[channel][sent_len], u2e_size[channel]);
            }
        }
    }
}

uint16_t get_serial_data(int channel) {
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);

    uint16_t i;
    uint16_t len;

    len = get_data_buffer_usedsize(channel);

    // Limit the copy before looking at a delimiter.  The old delimiter special
    // case wrote g_send_buf[DATA_BUF_SIZE] when a full staged buffer and one new
    // UART byte met here, corrupting the byte immediately after this channel's
    // 2 KB buffer and consuming the delimiter without increasing u2e_size.
    if (u2e_size[channel] >= DATA_BUF_SIZE) {
        return u2e_size[channel];
    }
    if (len > (DATA_BUF_SIZE - u2e_size[channel])) {
        len = DATA_BUF_SIZE - u2e_size[channel];
    }

    if ((!serial_data_packing->packing_time) &&
            (!serial_data_packing->packing_size) &&
            (!serial_data_packing->packing_delimiter[0])) { // No Data Packing tiem / size / delimiters.
        // ## 20150427 bugfix: Incorrect serial data storing (UART ring buffer to g_send_buf)
        for (i = 0; i < len; i++) {
            g_send_buf[channel][u2e_size[channel]++] = (uint8_t)data_buffer_getc(channel);
        }

        return u2e_size[channel];
    } else {
        /* Checking Data packing options */
        for (i = 0; i < len; i++) {
            g_send_buf[channel][u2e_size[channel]++] = (uint8_t)data_buffer_getc(channel);

            // Packing delimiter: character option
            if ((serial_data_packing->packing_delimiter[0] != 0x00) &&
                    (serial_data_packing->packing_delimiter[0] == g_send_buf[channel][u2e_size[channel] - 1])) {
                return u2e_size[channel];
            }

            // Packing delimiter: size option
            if ((serial_data_packing->packing_size != 0) && (serial_data_packing->packing_size == u2e_size[channel])) {
                return u2e_size[channel];
            }
        }
    }

    // Packing delimiter: time option
    if ((serial_data_packing->packing_time != 0) && (u2e_size[channel] != 0) && (flag_serial_input_time_elapse[channel] == SEG_ENABLE)) {
        if (get_data_buffer_usedsize(channel) == 0) {
            flag_serial_input_time_elapse[channel] = SEG_DISABLE;    // ##
        }

        return u2e_size[channel];
    }

    return 0;
}

// Transmit unit for DTR/DSR flow control. The RP2040 has no DTR/DSR hardware, so
// DSR can only be sampled between transmits; this bounds how much data can still
// be in flight after the peer deasserts it. 256 byte is about 5.5 ms at 460800 bps,
// against 44 ms when the whole DATA_BUF_SIZE block was sent at once.
#define E2S_DSR_CHUNK   256

static void ether_to_uart_unlocked(uint8_t sock, int channel) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);

    uint16_t len;
    uint16_t i;
    int32_t received;

    // This gate is what used to wedge E2S. platform_uart_cts_ready() once reported
    // ready unconditionally on the HW channels, so a peer holding CTS off stopped
    // the wire, the transmit DMA never completed, and platform_uart_tx_wait() below
    // never returned - taking the whole receive task with it. All four channels now
    // read their own CTS pin, so a stalled peer only parks this channel here and it
    // resumes without loss once CTS returns.
    //
    // Returning early on a busy DMA was tried and reverted: it fixed the wedge but
    // cost about 12 % of E2S throughput on every channel, because the task then
    // waits a whole tick between chunks instead of queueing the next one the moment
    // the DMA frees up. Any change here has to keep the pipe full - poll at finer
    // granularity, or bound the wait without letting recv() overwrite a buffer the
    // DMA is still reading.
    seg_recv_step[channel] = SEG_RECV_STEP_CTS;
    if (serial_option->flow_control == flow_rts_cts) {
        if (!platform_uart_cts_ready(channel)) {
            return;
        }
    } else if (serial_option->flow_control == flow_dtr_dsr) {
        // DSR takes CTS's place: the peer is only ready while it is asserted.
        if (get_flowcontrol_dsr_pin(channel) != IO_LOW) {
            return;
        }
    }

    do {
        // H/W Socket buffer -> User's buffer
        //
        // A previous pass may have left data the peer was not ready to accept
        // (see the flow_dtr_dsr branch below). Send that first: recv() assigns
        // rather than appends, so fetching now would overwrite it.
        if ((e2u_size[channel] == 0) &&
                !(network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE)) {
            seg_recv_step[channel] = SEG_RECV_STEP_RX_AVAIL;
            len = seg_socket_rx_available(sock, channel);
            if (len > DATA_BUF_SIZE) {
                len = DATA_BUF_SIZE;    // avoiding buffer overflow
            }

            if (len > 0) {
                if (seg_inactivity_timer[channel] != NULL) {
                    xTimerReset(seg_inactivity_timer[channel], 0);
                }

                // The previous transmit may still be reading g_recv_buf by DMA.
                seg_recv_step[channel] = SEG_RECV_STEP_TX_WAIT;
                platform_uart_tx_wait(channel);
                seg_recv_step[channel] = SEG_RECV_STEP_SOCK_RECV;

                if (network_connection->working_mode == UDP_MODE) {
                    uint8_t previous_peer_ip[4];
                    uint16_t previous_peer_port = udp_peer_port[channel];
                    uint8_t previous_peer_valid = udp_peer_valid[channel];

                    memcpy(previous_peer_ip, udp_peer_ip[channel],
                           sizeof(previous_peer_ip));
                    received = seg_socket_recvfrom(sock, g_recv_buf[channel], len,
                                                   udp_peer_ip[channel],
                                                   &udp_peer_port[channel],
                                                   channel);
                    if (received <= 0) {
                        // Socket teardown can race this receive task.  Never cast a
                        // negative ioLibrary error to uint16_t: that turns e.g. -7
                        // into a ~64 KB DMA transfer from a 2 KB channel buffer.
                        e2u_size[channel] = 0;
                        return;
                    }
                    e2u_size[channel] = (uint16_t)received;
                    udp_peer_valid[channel] = SEG_ENABLE;

                    if ((previous_peer_valid == SEG_DISABLE) ||
                            (memcmp(previous_peer_ip, udp_peer_ip[channel], 4) != 0) ||
                            (previous_peer_port != udp_peer_port[channel])) {
                        if (serial_common->serial_debug_en) {
                            PRT_SEG(" > UDP Peer ch=%d IP/Port: %d.%d.%d.%d : %d\r\n",
                                    channel,
                                    udp_peer_ip[channel][0], udp_peer_ip[channel][1],
                                    udp_peer_ip[channel][2], udp_peer_ip[channel][3],
                                    udp_peer_port[channel]);
                        }
                    }
                    //} else if (network_connection->working_state == ST_CONNECT) {
                } else {
#ifdef __USE_S2E_OVER_TLS__
                    if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                        uint16_t reg_val = SIK_RECEIVED & 0x00FF;

                        seg_socket_lock();
                        received = wiz_tls_read(&s2e_tlsContext[channel], g_recv_buf[channel], len);
                        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);
                        seg_socket_unlock();

                        if (received <= 0) {
                            e2u_size[channel] = 0;
                            return;
                        }
                        e2u_size[channel] = (uint16_t)received;
                    }
#endif
                    else {
                        received = seg_socket_recv(sock, g_recv_buf[channel], len,
                                                   channel);
                        if (received <= 0) {
                            e2u_size[channel] = 0;
                            return;
                        }
                        e2u_size[channel] = (uint16_t)received;
                    }
                }
            } else {
                break;
            }

            if ((network_connection->working_mode == TCP_SERVER_MODE) ||  \
                    ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state[channel] == MIXED_SERVER))) {
                // Connection password authentication
                if ((tcp_option->pw_connect_en == SEG_ENABLE) && (flag_connect_pw_auth[channel] == SEG_DISABLE)) {
                    uint16_t auth_consumed = 0;
                    uint8_t auth_result = check_connect_pw_auth(channel,
                                          g_recv_buf[channel], e2u_size[channel],
                                          &auth_consumed);

                    if (auth_result == SEG_AUTH_PENDING) {
                        e2u_size[channel] = 0;
                        return;
                    }
                    if (auth_result == SEG_AUTH_SUCCESS) {
                        flag_connect_pw_auth[channel] = SEG_ENABLE;
                        if (auth_consumed < e2u_size[channel]) {
                            e2u_size[channel] -= auth_consumed;
                            memmove(g_recv_buf[channel],
                                    &g_recv_buf[channel][auth_consumed],
                                    e2u_size[channel]);
                        } else {
                            e2u_size[channel] = 0;
                        }
                    } else {
                        flag_connect_pw_auth[channel] = SEG_DISABLE;
                        e2u_size[channel] = 0;
                    }

                    if (seg_auth_timer[channel] != NULL) {
                        xTimerStop(seg_auth_timer[channel], 0);
                    }
                    if (flag_connect_pw_auth[channel] == SEG_DISABLE) {
                        seg_socket_disconnect(sock);
                        return;
                    }
                }
            }
        }
        // Ethernet data transfer to DATA UART
        if (e2u_size[channel] != 0) {
            //////////////////////////////////////////////////////////////////////
#ifdef __USE_UART_485_422__
            if (serial_option->uart_interface != UART_IF_RS232_TTL) {
                if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                    debugSerial_dataTransfer(g_recv_buf[channel], e2u_size[channel], SEG_DEBUG_E2S);
                }

                uart_rs485_enable(channel);
                //for(i = 0; i < e2u_size[channel]; i++) platform_uart_putc(g_recv_buf[channel][i], channel);
                platform_uart_puts_dma(g_recv_buf[channel], e2u_size[channel], channel);
                uart_rs485_disable(channel);

                e2u_size[channel] = 0;
            }
            //////////////////////////////////////////////////////////////////////
            else if (serial_option->flow_control == flow_xon_xoff)
#else
            if (serial_option->flow_control == flow_xon_xoff)
#endif
            {
                if (isXON[channel] == SEG_ENABLE) {
                    if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                        debugSerial_dataTransfer(g_recv_buf[channel], e2u_size[channel], SEG_DEBUG_E2S);
                    }

                    for (i = 0; i < e2u_size[channel]; i++) {
                        platform_uart_putc(g_recv_buf[channel][i], channel);
                    }
                    e2u_size[channel] = 0;
                }
            } else if (serial_option->flow_control == flow_dtr_dsr) {
                if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                    debugSerial_dataTransfer(g_recv_buf[channel], e2u_size[channel], SEG_DEBUG_E2S);
                }

                // Sending the whole block in one transfer left up to DATA_BUF_SIZE
                // (44 ms at 460800 bps) in flight after the peer deasserted DSR,
                // which overran it under full load. Send in chunks and re-check DSR
                // between them.
                //
                // If the peer stops mid-block, return and keep the remainder for
                // the next pass rather than waiting here. Blocking would hold the
                // seg loop, which also has to service check_uart_flow_control()
                // (releases DTR) and uart_to_ether() (drains the ring buffer) -
                // starving both deadlocks the S2E direction. This matches how the
                // RTS/CTS path returns when CTS is deasserted.
                //
                // Original (shared with the branch below):
                //     platform_uart_puts_dma(g_recv_buf[channel], e2u_size[channel], channel);
                //     e2u_size[channel] = 0;
                uint16_t sent = 0;
                while (sent < e2u_size[channel]) {
                    if (get_flowcontrol_dsr_pin(channel) != IO_LOW) {
                        break;
                    }

                    uint16_t chunk = e2u_size[channel] - sent;
                    if (chunk > E2S_DSR_CHUNK) {
                        chunk = E2S_DSR_CHUNK;
                    }
                    platform_uart_puts_dma(&g_recv_buf[channel][sent], chunk, channel);
                    sent += chunk;
                }

                if (sent < e2u_size[channel]) {
                    // Move the untransmitted remainder to the head of the buffer.
                    // The transmit DMA reads from that buffer, so let it finish first.
                    platform_uart_tx_wait(channel);
                    e2u_size[channel] -= sent;
                    memmove(g_recv_buf[channel], &g_recv_buf[channel][sent], e2u_size[channel]);
                    return;
                }
                e2u_size[channel] = 0;
            } else {
                if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                    debugSerial_dataTransfer(g_recv_buf[channel], e2u_size[channel], SEG_DEBUG_E2S);
                }

                //for(i = 0; i < e2u_size[channel]; i++) platform_uart_putc(g_recv_buf[channel][i], channel);
                platform_uart_puts_dma(g_recv_buf[channel], e2u_size[channel], channel);
                e2u_size[channel] = 0;
            }
        }
    } while (e2u_size[channel]);
}

void ether_to_uart(uint8_t sock, int channel) {
    if ((channel < 0) || (channel >= DEVICE_UART_CNT)) {
        return;
    }

    if (seg_recv_sem[channel] != NULL) {
        xSemaphoreTake(seg_recv_sem[channel], portMAX_DELAY);
    }
    ether_to_uart_unlocked(sock, channel);
    if (seg_recv_sem[channel] != NULL) {
        xSemaphoreGive(seg_recv_sem[channel]);
    }
}

void ether_to_spi(uint8_t sock) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[SEG_DATA0_CH]);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[SEG_DATA0_CH]);
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[SEG_DATA0_CH]);
    struct __device_option *device_option = (struct __device_option *) & (get_DevConfig_pointer()->device_option);

    uint16_t len;
    uint16_t i;
    int32_t received;

    do {
        // H/W Socket buffer -> User's buffer
        if (!(network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE)) {
            len = seg_socket_rx_available(sock, SEG_DATA0_CH);
            if (len > DATA_BUF_SIZE) {
                len = DATA_BUF_SIZE;    // avoiding buffer overflow
            }

            if (len > 0) {
                if (network_connection->working_mode == UDP_MODE) {
                    uint8_t previous_peer_ip[4];
                    uint16_t previous_peer_port = udp_peer_port[SEG_DATA0_CH];
                    uint8_t previous_peer_valid = udp_peer_valid[SEG_DATA0_CH];

                    memcpy(previous_peer_ip, udp_peer_ip[SEG_DATA0_CH],
                           sizeof(previous_peer_ip));
                    received = seg_socket_recvfrom(sock, g_recv_buf[SEG_DATA0_CH], len,
                                                   udp_peer_ip[SEG_DATA0_CH],
                                                   &udp_peer_port[SEG_DATA0_CH],
                                                   SEG_DATA0_CH);
                    if (received <= 0) {
                        e2u_size[SEG_DATA0_CH] = 0;
                        return;
                    }
                    e2u_size[SEG_DATA0_CH] = (uint16_t)received;
                    udp_peer_valid[SEG_DATA0_CH] = SEG_ENABLE;

                    if ((previous_peer_valid == SEG_DISABLE) ||
                            (memcmp(previous_peer_ip, udp_peer_ip[SEG_DATA0_CH], 4) != 0) ||
                            (previous_peer_port != udp_peer_port[SEG_DATA0_CH])) {
                        if (serial_common->serial_debug_en) {
                            printf(" > UDP Peer ch=0 IP/Port: %d.%d.%d.%d : %d\r\n",
                                   udp_peer_ip[SEG_DATA0_CH][0], udp_peer_ip[SEG_DATA0_CH][1],
                                   udp_peer_ip[SEG_DATA0_CH][2], udp_peer_ip[SEG_DATA0_CH][3],
                                   udp_peer_port[SEG_DATA0_CH]);
                        }
                    }
                } else if (network_connection->working_state == ST_CONNECT) {
#ifdef __USE_S2E_OVER_TLS__
                    if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                        uint16_t reg_val = SIK_RECEIVED & 0x00FF;

                        seg_socket_lock();
                        received = wiz_tls_read(&s2e_tlsContext[SEG_DATA0_CH], g_recv_buf[SEG_DATA0_CH], len);
                        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);
                        seg_socket_unlock();

                        if (received <= 0) {
                            e2u_size[SEG_DATA0_CH] = 0;
                            return;
                        }
                        e2u_size[SEG_DATA0_CH] = (uint16_t)received;
                    }
#endif
                    else {
                        received = seg_socket_recv(sock, g_recv_buf[SEG_DATA0_CH], len,
                                                   SEG_DATA0_CH);
                        if (received <= 0) {
                            e2u_size[SEG_DATA0_CH] = 0;
                            return;
                        }
                        e2u_size[SEG_DATA0_CH] = (uint16_t)received;
                    }
                }
            } else {
                break;
            }

            if ((network_connection->working_mode == TCP_SERVER_MODE) ||  \
                    ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state[SEG_DATA0_CH] == MIXED_SERVER))) {
                // Connection password authentication
                if ((tcp_option->pw_connect_en == SEG_ENABLE) && (flag_connect_pw_auth[SEG_DATA0_CH] == SEG_DISABLE)) {
                    uint16_t auth_consumed = 0;
                    uint8_t auth_result = check_connect_pw_auth(SEG_DATA0_CH,
                                          g_recv_buf[SEG_DATA0_CH], e2u_size[SEG_DATA0_CH],
                                          &auth_consumed);

                    if (auth_result == SEG_AUTH_PENDING) {
                        e2u_size[SEG_DATA0_CH] = 0;
                        return;
                    }
                    if (auth_result == SEG_AUTH_SUCCESS) {
                        flag_connect_pw_auth[SEG_DATA0_CH] = SEG_ENABLE;
                        if (auth_consumed < e2u_size[SEG_DATA0_CH]) {
                            e2u_size[SEG_DATA0_CH] -= auth_consumed;
                            memmove(g_recv_buf[SEG_DATA0_CH],
                                    &g_recv_buf[SEG_DATA0_CH][auth_consumed],
                                    e2u_size[SEG_DATA0_CH]);
                        } else {
                            e2u_size[SEG_DATA0_CH] = 0;
                        }
                    } else {
                        flag_connect_pw_auth[SEG_DATA0_CH] = SEG_DISABLE;
                        e2u_size[SEG_DATA0_CH] = 0;
                    }

                    if (seg_auth_timer[SEG_DATA0_CH] != NULL) {
                        xTimerStop(seg_auth_timer[SEG_DATA0_CH], 0);
                    }
                    if (flag_connect_pw_auth[SEG_DATA0_CH] == SEG_DISABLE) {
                        seg_socket_disconnect(sock);
                        return;
                    }
                }
            }
        }
        // Ethernet data transfer to DATA UART
        if (e2u_size[SEG_DATA0_CH] != 0) {
            if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                debugSerial_dataTransfer(g_recv_buf[SEG_DATA0_CH], e2u_size[SEG_DATA0_CH], SEG_DEBUG_E2S);
            }
        }
    } while (0);
}

uint16_t get_tcp_any_port(void) {
    if (client_any_port) {
        if (client_any_port < 0xffff) {
            client_any_port++;
        } else {
            client_any_port = 0;
        }
    }

    if (client_any_port == 0) {
        // todo: gen random seed (srand + random value)
        client_any_port = (rand() % 10000) + 35000; // 35000 ~ 44999
    }

    return client_any_port;
}


uint8_t get_serial_communation_protocol(int channel) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);

    // SEG_SERIAL_PROTOCOL_NONE
    // SEG_SERIAL_MODBUS_RTU
    // SEG_SERIAL_MODBUS_ASCII

    return serial_option->protocol;
}


static uint8_t seg_keepalive_data_active(int channel) {
    return ((u2e_size[channel] != 0) ||
            (e2u_size[channel] != 0) ||
            (get_data_buffer_usedsize(channel) != 0) ||
            ((seg_s2e_last_send_len[channel] != 0) &&
             (seg_s2e_last_send_rc[channel] == SOCK_BUSY))) ? TRUE : FALSE;
}

uint8_t send_keepalive_packet_manual(uint8_t sock, int channel) {
    struct __tcp_option *tcp_option =
        (struct __tcp_option *)&get_DevConfig_pointer()->tcp_option[channel];
    uint8_t sent = FALSE;

    // SEND_KEEP uses the same per-socket command register as data SEND.  A mutex
    // around the two API calls only prevents their CPU-side transactions from
    // overlapping; send() returns while the hardware SEND is still pending.
    // Revalidate idleness while holding the API mutex so an active data stream
    // can never receive a blind timer-driven SEND_KEEP command.
    seg_socket_lock();
    uint32_t now = (uint32_t)millis();
    uint32_t last_activity = seg_tcp_last_activity_ms[channel];
    uint32_t idle_ms = (uint32_t)(now - last_activity);
    uint32_t wait_ms = seg_keepalive_interval_ms(
                           tcp_option->keepalive_wait_time);

    if ((get_device_status(channel) == ST_CONNECT) &&
            (getSn_SR(sock) == SOCK_ESTABLISHED) &&
            (last_activity != 0) &&
            (idle_ms >= wait_ms) &&
            !seg_keepalive_data_active(channel)) {
        sent = (setsockopt(sock, SO_KEEPALIVESEND, 0) == SOCK_OK) ? TRUE : FALSE;
    }
    seg_socket_unlock();

    return sent;
}


uint8_t process_socket_termination(uint8_t sock, uint32_t timeout, int channel, uint8_t mutex) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);

    int8_t ret;
    uint8_t sock_status = getSn_SR(sock);
    uint32_t tickStart = millis();

    timers_stop(channel);
    if (!(network_connection->working_mode == TCP_MIXED_MODE && mixed_state[channel] == MIXED_CLIENT)) {
        reset_SEG_timeflags(channel);
    }

#ifdef __USE_S2E_OVER_TLS__
    if (get_wiz_tls_init_state(channel) == ENABLE) {
        wiz_tls_close_notify(&s2e_tlsContext[channel]);
        PRT_SEG("wiz_tls_deinit\r\n");
        wiz_tls_deinit(&s2e_tlsContext[channel]);
        set_wiz_tls_init_state(DISABLE, channel);
    }
#endif

    if (sock_status == SOCK_CLOSED) {
        return sock;
    }
    if (mutex == TRUE) {
        xSemaphoreTake(seg_critical_sem[channel], portMAX_DELAY);
    }
    if (network_connection->working_mode != UDP_MODE) { // TCP_SERVER_MODE / TCP_CLIENT_MODE / TCP_MIXED_MODE
        if ((sock_status == SOCK_ESTABLISHED) || (sock_status == SOCK_CLOSE_WAIT)) {
            do {
                ret = seg_socket_disconnect(sock);
                if ((ret == SOCK_OK) || (ret == SOCKERR_TIMEOUT)) {
                    break;
                }
            } while ((millis() - tickStart) < timeout);
        }
    }

    seg_socket_close(sock);
    if (mutex == TRUE) {
        xSemaphoreGive(seg_critical_sem[channel]);
    }
    xSemaphoreGive(seg_sem[channel]);
    return sock;
}

uint8_t check_connect_pw_auth(int channel, uint8_t *buf, uint16_t len,
                              uint16_t *consumed) {
    struct __tcp_option *tcp_option;
    size_t expected_len;
    uint16_t i;

    if ((channel < 0) || (channel >= DEVICE_UART_CNT) ||
            (buf == NULL) || (consumed == NULL)) {
        return SEG_AUTH_FAILED;
    }

    tcp_option = (struct __tcp_option *)&get_DevConfig_pointer()->tcp_option[channel];
    expected_len = strlen(tcp_option->pw_connect);
    *consumed = 0;

    if (expected_len == 0) {
        connect_pw_progress[channel] = 0;
        return SEG_AUTH_SUCCESS;
    }

    for (i = 0; i < len; i++) {
        uint8_t progress = connect_pw_progress[channel];

        if ((progress >= expected_len) ||
                (buf[i] != (uint8_t)tcp_option->pw_connect[progress])) {
            connect_pw_progress[channel] = 0;
            return SEG_AUTH_FAILED;
        }

        connect_pw_progress[channel] = progress + 1U;
        *consumed = i + 1U;
        if (connect_pw_progress[channel] == expected_len) {
            connect_pw_progress[channel] = 0;
            return SEG_AUTH_SUCCESS;
        }
    }

    return SEG_AUTH_PENDING;
}


void init_trigger_modeswitch(uint8_t mode) {
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection;

    if (mode == DEVICE_AT_MODE) {
        opmode = DEVICE_AT_MODE;
        for (int i = 0; i < DEVICE_UART_CNT; i++) {
            set_device_status(ST_ATMODE, i);
        }

        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:AT Mode\r\n");
            platform_uart_puts((uint8_t *)"SEG:AT Mode\r\n", strlen("SEG:AT Mode\r\n"), SEG_DATA0_CH);
        }
    } else { // DEVICE_GW_MODE
        opmode = DEVICE_GW_MODE;

        for (int i = 0; i < DEVICE_UART_CNT; i++) {
            set_device_status(ST_OPEN, i);
            network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[i]);
            if (network_connection->working_mode == TCP_MIXED_MODE) {
                mixed_state[i] = MIXED_SERVER;
            }
        }

        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:GW Mode\r\n");
            platform_uart_puts((uint8_t *)"SEG:GW Mode\r\n", strlen("SEG:GW Mode\r\n"), SEG_DATA0_CH);
        }
    }

    for (int i = 0; i < DEVICE_UART_CNT; i++) {
        u2e_size[i] = 0;
        e2u_size[i] = 0;
        data_buffer_flush(i);
        reset_SEG_timeflags(i);
    }
}

uint8_t check_modeswitch_trigger(uint8_t ch) {
    struct __serial_command *serial_command = (struct __serial_command *) & (get_DevConfig_pointer()->serial_command);

    uint8_t modeswitch_failed = SEG_DISABLE;
    uint8_t ret = 0;

    if (opmode != DEVICE_GW_MODE) {
        return 0;
    }
    if (serial_command->serial_command == SEG_DISABLE) {
        return 0;
    }

    switch (triggercode_idx) {
    case 0:
        if ((ch == serial_command->serial_trigger[triggercode_idx]) && (modeswitch_time >= modeswitch_gap_time)) { // comparison succeed
            ch_tmp[triggercode_idx] = ch;
            triggercode_idx++;
            enable_modeswitch_timer = SEG_ENABLE;
        }
        break;

    case 1:
    case 2:
        if ((ch == serial_command->serial_trigger[triggercode_idx]) && (modeswitch_time <= modeswitch_gap_time)) { // comparison succeed
            ch_tmp[triggercode_idx] = ch;
            triggercode_idx++;
        } else { // comparison failed: invalid trigger code
            modeswitch_failed = SEG_ENABLE;
        }
        break;
    case 3:
        if (modeswitch_time < modeswitch_gap_time) { // comparison failed: end gap
            modeswitch_failed = SEG_ENABLE;
        }
        break;
    }

    if (modeswitch_failed == SEG_ENABLE) {
        restore_serial_data(triggercode_idx);
    }

    modeswitch_time = 0; // reset the inter-gap time count for each trigger code recognition (Allowable interval)
    ret = triggercode_idx;

    return ret;
}

// when serial command mode trigger code comparison failed
void restore_serial_data(uint8_t idx) {
    uint8_t i;

    for (i = 0; i < idx; i++) {
        put_byte_to_data_buffer(ch_tmp[i], SEG_DATA0_CH);
        ch_tmp[i] = 0x00;
    }

    enable_modeswitch_timer = SEG_DISABLE;
    triggercode_idx = 0;
}

uint8_t check_serial_store_permitted(uint8_t ch, int channel) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);

    uint8_t ret = SEG_DISABLE; // SEG_DISABLE: Doesn't put the serial data in a ring buffer

    switch (network_connection->working_state) {
    case ST_OPEN:
        if (network_connection->working_mode != TCP_MIXED_MODE) {
            return ret;
        }
    case ST_CONNECT:
    case ST_UDP:
    case ST_ATMODE:
        ret = SEG_ENABLE;
        break;
    default:
        break;
    }

    // Software flow control: Check the XON/XOFF start/stop commands
    // [Peer] -> [WIZnet Device]
    if ((ret == SEG_ENABLE) && (serial_option->flow_control == flow_xon_xoff)) {
        if (ch == UART_XON) {
            isXON[channel] = SEG_ENABLE;
            ret = SEG_DISABLE;
        } else if (ch == UART_XOFF) {
            isXON[channel] = SEG_DISABLE;
            ret = SEG_DISABLE;
        }
    }
    return ret;
}

void reset_SEG_timeflags(uint8_t channel) {
    // Timer disable
    enable_serial_input_timer[channel] = SEG_DISABLE;

    // Flag clear
    flag_serial_input_time_elapse[channel] = SEG_DISABLE;
    flag_send_keepalive[channel] = SEG_DISABLE;
    flag_first_keepalive[channel] = SEG_DISABLE;
    flag_auth_time[channel] = SEG_DISABLE;
    flag_connect_pw_auth[channel] = SEG_DISABLE; // TCP_SERVER_MODE only (+ MIXED_SERVER)
    connect_pw_progress[channel] = 0;
    isXON[channel] = SEG_ENABLE;
    flag_inactivity[channel] = SEG_DISABLE;

    // Timer value clear
    serial_input_time[channel] = 0;
}

void init_time_delimiter_timer(int channel) {
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[channel]);

    if (opmode == DEVICE_GW_MODE) {
        if (serial_data_packing->packing_time != 0) {
            if (enable_serial_input_timer[channel] == SEG_DISABLE) {
                enable_serial_input_timer[channel] = SEG_ENABLE;
            }
            serial_input_time[channel] = 0;
        }
    }
}

uint8_t check_tcp_connect_exception(int channel) {
    struct __network_option *network_option = (struct __network_option *)&get_DevConfig_pointer()->network_option;
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);

    uint8_t srcip[4] = {0, };
    uint8_t ret = OFF;

    getSIPR(srcip);

    // DNS failed
    if ((network_connection->dns_use == SEG_ENABLE) && (flag_process_dns_success[channel] != ON)) {
        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:CONNECTION FAILED - DNS Failed flag_process_dns_success[%d] = %d\r\n", channel, flag_process_dns_success[channel]);
        }
        ret = ON;
    }
    // if dhcp failed (0.0.0.0), this case do not connect to peer
    else if ((srcip[0] == 0x00) && (srcip[1] == 0x00) && (srcip[2] == 0x00) && (srcip[3] == 0x00)) {
        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:CONNECTION FAILED - Invalid IP address: Zero IP\r\n");
        }
        ret = ON;
    }
    // Destination zero IP
    else if ((network_connection->remote_ip[0] == 0x00) &&
             (network_connection->remote_ip[1] == 0x00) &&
             (network_connection->remote_ip[2] == 0x00) &&
             (network_connection->remote_ip[3] == 0x00)) {
        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:CONNECTION FAILED - Invalid Destination IP address: Zero IP\r\n");
        }
        ret = ON;
    }
    // Duplicate IP address
    else if ((srcip[0] == network_connection->remote_ip[0]) &&
             (srcip[1] == network_connection->remote_ip[1]) &&
             (srcip[2] == network_connection->remote_ip[2]) &&
             (srcip[3] == network_connection->remote_ip[3])) {
        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:CONNECTION FAILED - Duplicate IP address\r\n");
        }
        ret = ON;
    } else if ((srcip[0] == 192) && (srcip[1] == 168)) { // local IP address == Class C private IP
        // Static IP address obtained
        if ((network_option->dhcp_use == SEG_DISABLE) && ((network_connection->remote_ip[0] == 192) &&
                (network_connection->remote_ip[1] == 168))) {
            if (srcip[2] != network_connection->remote_ip[2]) { // Class C Private IP network mismatch
                if (serial_common->serial_debug_en)
                    PRT_SEG(" > SEG:CONNECTION FAILED - Invalid IP address range (%d.%d.[%d].%d)\r\n",
                            network_connection->remote_ip[0],
                            network_connection->remote_ip[1],
                            network_connection->remote_ip[2],
                            network_connection->remote_ip[3]);
                ret = ON;
            }
        }
    }

    return ret;
}

int wizchip_mqtt_publish(mqtt_config_t *mqtt_config, uint8_t *pub_topic, uint8_t qos, uint8_t *pub_data, uint32_t pub_data_len) {
    if (mqtt_transport_publish(mqtt_config, pub_topic, pub_data, pub_data_len, qos)) {
        return -1;
    }
    return pub_data_len;
}

void mqtt_subscribeMessageHandler0(uint8_t *data, uint32_t data_len) {
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);

    e2u_size[SEG_DATA0_CH] = data_len;
    memcpy(g_recv_buf[SEG_DATA0_CH], data, data_len);

    ether_to_uart(SEG_DATA0_SOCK, SEG_DATA0_CH);
}

void mqtt_subscribeMessageHandler1(uint8_t *data, uint32_t data_len) {
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);

    e2u_size[SEG_DATA1_CH] = data_len;
    memcpy(g_recv_buf[SEG_DATA1_CH], data, data_len);

#if 0
    if (serial_common->serial_debug_en) {
        PRT_INFO("Eth Recv len = %d : ", data_len);
        for (uint32_t i = 0; i < data_len; i++) {
            printf("0x%02X ", g_recv_buf[i], i);
        }
        printf("\r\n");
    }
#endif
    ether_to_uart(SEG_DATA1_SOCK, SEG_DATA1_CH);
}

#if (DEVICE_UART_CNT > 2)
// MQTT subscribe callback for DATA2 (WIZnet MQTT lib callback carries no channel arg)
void mqtt_subscribeMessageHandler2(uint8_t *data, uint32_t data_len) {
    e2u_size[SEG_DATA2_CH] = data_len;
    memcpy(g_recv_buf[SEG_DATA2_CH], data, data_len);
    ether_to_uart(SEG_DATA2_SOCK, SEG_DATA2_CH);
}
#endif
#if (DEVICE_UART_CNT > 3)
// MQTT subscribe callback for DATA3
void mqtt_subscribeMessageHandler3(uint8_t *data, uint32_t data_len) {
    e2u_size[SEG_DATA3_CH] = data_len;
    memcpy(g_recv_buf[SEG_DATA3_CH], data, data_len);
    ether_to_uart(SEG_DATA3_SOCK, SEG_DATA3_CH);
}
#endif

uint16_t debugSerial_dataTransfer(uint8_t * buf, uint16_t size, teDEBUGTYPE type) {
    uint16_t bytecnt = 0;

    if (getDeviceUptime_day() > 0) {
        printf(" [%ldd/%02d:%02d:%02d]", getDeviceUptime_day(), getDeviceUptime_hour(), getDeviceUptime_min(), getDeviceUptime_sec());
    } else {
        printf(" [%02d:%02d:%02d]", getDeviceUptime_hour(), getDeviceUptime_min(), getDeviceUptime_sec());
    }

    if ((type == SEG_DEBUG_S2E) || (type == SEG_DEBUG_E2S)) {
        printf("[%s][%04d] ", (type == SEG_DEBUG_S2E) ? "S2E" : "E2S", size);
        for (bytecnt = 0; bytecnt < size; bytecnt++) {
            printf("%02X ", buf[bytecnt]);
        }
        printf("\r\n");
    }

    return bytecnt;
}


void send_sid(uint8_t sock, uint8_t link_message) {
    DevConfig *dev_config = get_DevConfig_pointer();

    uint8_t buf[45] = {0, };
    uint8_t len = 0;

    switch (link_message) {
    case SEG_LINK_MSG_NONE:
        break;

    case SEG_LINK_MSG_DEVNAME:
        len = snprintf((char *)buf, sizeof(buf), "%s", dev_config->device_common.device_name);
        break;

    case SEG_LINK_MSG_MAC:
        len = snprintf((char *)buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                       dev_config->network_common.mac[0],
                       dev_config->network_common.mac[1],
                       dev_config->network_common.mac[2],
                       dev_config->network_common.mac[3],
                       dev_config->network_common.mac[4],
                       dev_config->network_common.mac[5]);
        break;

    case SEG_LINK_MSG_IP:
        len = snprintf((char *)buf, sizeof(buf), "%d.%d.%d.%d",
                       dev_config->network_common.local_ip[0],
                       dev_config->network_common.local_ip[1],
                       dev_config->network_common.local_ip[2],
                       dev_config->network_common.local_ip[3]);
        break;

    case SEG_LINK_MSG_DEVID:
        len = snprintf((char *)buf, sizeof(buf), "%s-%02X%02X%02X%02X%02X%02X",
                       dev_config->device_common.device_name,
                       dev_config->network_common.mac[0],
                       dev_config->network_common.mac[1],
                       dev_config->network_common.mac[2],
                       dev_config->network_common.mac[3],
                       dev_config->network_common.mac[4],
                       dev_config->network_common.mac[5]);
        break;


    case SEG_LINK_MSG_DEVALIAS:
        len = snprintf((char *)buf, sizeof(buf), "%s", dev_config->device_option.device_alias);
        break;

    case SEG_LINK_MSG_DEVGROUP:
        len = snprintf((char *)buf, sizeof(buf), "%s", dev_config->device_option.device_group);
        break;

    default:
        break;
    }

    if (len > 0) {
        seg_socket_send(sock, buf, len, SEG_DATA0_CH);
    }
}

// This function have to call every 1 millisecond by Timer IRQ handler routine.
void seg_timer_msec(void) {
    struct __serial_data_packing *serial_data_packing;
    struct __network_connection *network_connection;

    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    // Serial data packing time delimiter timer

    for (int i = 0; i < DEVICE_UART_CNT; i++) {
        serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing[i]);
        network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[i]);

        if (enable_serial_input_timer[i]) {
            if (serial_input_time[i] < serial_data_packing->packing_time) {
                serial_input_time[i]++;
            } else {
                serial_input_time[i] = 0;
                enable_serial_input_timer[i] = 0;
                flag_serial_input_time_elapse[i] = SEG_ENABLE;

                switch (network_connection->working_mode) {
                case TCP_CLIENT_MODE:
                case TCP_SERVER_MODE:
                case TCP_MIXED_MODE:
                case SSL_TCP_CLIENT_MODE:
                case UDP_MODE:
                case MQTT_CLIENT_MODE:
                case MQTTS_CLIENT_MODE:
                    xSemaphoreGiveFromISR(seg_u2e_sem[i], &xHigherPriorityTaskWoken);
                    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
                    break;
                }

            }
        }
    }
    // Mode switch timer: Time count routine (msec) (GW mode <-> Serial command mode, for s/w mode switch trigger code)
    if (modeswitch_time < modeswitch_gap_time) {
        modeswitch_time++;
    }

    if ((enable_modeswitch_timer) && (modeswitch_time >= modeswitch_gap_time)) {
        // result of command mode trigger code comparison
        if (triggercode_idx == 3) {
            sw_modeswitch_at_mode_on = SEG_ENABLE;  // success
            xSemaphoreGiveFromISR(segcp_uart_sem, &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
        } else {
            restore_serial_data(triggercode_idx);    // failed
        }

        triggercode_idx = 0;
        enable_modeswitch_timer = SEG_DISABLE;
    }
}

void seg_task(void *argument)  {

    while (1) {
        if (get_net_status() == NET_LINK_DISCONNECTED) {
            PRT_SEGCP("get_net_status() != NET_LINK_DISCONNECTED\r\n");
            xSemaphoreTake(net_seg_sem[SEG_DATA0_CH], portMAX_DELAY);
        }
        do_seg(SEG_DATA0_SOCK, SEG_DATA0_CH);
        do_seg(SEG_DATA1_SOCK, SEG_DATA1_CH);
        xSemaphoreTake(seg_sem[SEG_DATA0_CH], pdMS_TO_TICKS(10));
    }
}

void seg_ch_task(void *argument)  {
    int channel = (int)(uintptr_t)argument;
    uint8_t sock = seg_data_sock[channel];

    while (1) {
        seg_task_heartbeat_ms[channel] = (uint32_t)millis();
        seg_postmortem_mark(SEG_PM_TASK_SEG, (uint8_t)channel);
        if (get_net_status() == NET_LINK_DISCONNECTED) {
            PRT_SEGCP("get_net_status() != NET_LINK_DISCONNECTED\r\n");
            xSemaphoreTake(net_seg_sem[channel], portMAX_DELAY);
        }
        xSemaphoreTake(seg_critical_sem[channel], portMAX_DELAY);
        do_seg(sock, channel);
        xSemaphoreGive(seg_critical_sem[channel]);
        xSemaphoreTake(seg_sem[channel], pdMS_TO_TICKS(10));
    }
}

// seg_ch_task sits at SEG_TASK_PRIORITY while this task runs at the clamped ceiling,
// so releasing seg_critical_sem below does not hand over the CPU: this loop comes back
// around and retakes it before the lower priority task is ever scheduled. A 12 hour
// soak left one channel that way for ten hours.
//
// Two consequences, handled at the end of the drain. check_uart_flow_control() used to
// run only from do_seg(), so the peer stayed blocked for as long as seg_ch_task was
// kept out - the ring buffer drained to empty with RTS still asserted and no producer
// left to restart the channel. The buffer level changes here, so the deassert belongs
// here too, inside the critical section so it cannot race the do_seg() call. Hardware
// handshakes only: XON/XOFF writes a byte to the UART, which that call already owns.
//
// Flow control does not cover the rest of do_seg() though - seg_ch_task also drives
// socket state, so a channel that never yields stops noticing disconnects. taskYIELD()
// is no help, it only reaches equal or higher priority, so a tick of blocking is the
// only way down. Spent only once seg_ch_task is visibly behind, which costs nothing
// while it is keeping up.
#define SEG_U2E_YIELD_WINDOW_MS 50U

void seg_ch_u2e_task(void *argument)  {
    int channel = (int)(uintptr_t)argument;
    uint8_t sock = seg_data_sock[channel];
    uint8_t serial_mode = get_serial_communation_protocol(channel);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);

    while (1) {
        if (get_net_status() == NET_LINK_DISCONNECTED) {
            PRT_SEGCP("get_net_status() != NET_LINK_DISCONNECTED\r\n");
            xSemaphoreTake(net_seg_u2e_sem[channel], portMAX_DELAY);
        }

        xSemaphoreTake(seg_u2e_sem[channel], portMAX_DELAY);
        // Outside the diagnostics guard: the postmortem names the task that was
        // last running when the watchdog fired, and a breadcrumb this task never
        // leaves makes it name a different one.
        seg_postmortem_mark(SEG_PM_TASK_U2E, (uint8_t)channel);
        switch (serial_mode) {
        case SEG_SERIAL_PROTOCOL_NONE :
            xSemaphoreTake(seg_critical_sem[channel], portMAX_DELAY);
            if (get_data_buffer_usedsize(channel) || u2e_size[channel]) {
                if ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state[channel] == MIXED_SERVER) && (ST_OPEN == get_device_status(channel))) {
                    mixed_state[channel] = MIXED_CLIENT;
                    process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                    xSemaphoreGive(seg_sem[channel]);
                } else if ((ST_CONNECT == get_device_status(channel)) || network_connection->working_mode == UDP_MODE) {
                    uart_to_ether(sock, channel);
                }
            }
            if ((serial_option->flow_control == flow_rts_cts) ||
                    (serial_option->flow_control == flow_dtr_dsr)) {
                check_uart_flow_control(serial_option->flow_control, channel);
            }
            xSemaphoreGive(seg_critical_sem[channel]);
            if ((uint32_t)((uint32_t)millis() - seg_task_heartbeat_ms[channel]) >= SEG_U2E_YIELD_WINDOW_MS) {
                vTaskDelay(1);
            }
            break;

        case SEG_SERIAL_MODBUS_RTU : {
            uint8_t mb_finish_flag = 0;
            uint8_t mb_retry_count = 0;

            while (1) {
                RTU_Uart_RX(channel);
                if (mb_state_rtu_finish[channel] == TRUE) {
                    mb_state_rtu_finish[channel] = FALSE;
                    mb_finish_flag = mbRTUtoTCP(sock, channel);

                    if (mb_finish_flag == FALSE && mb_retry_count < MB_RETRY_MAX) {
                        mb_retry_count++;
                        PRT_INFO("RTU Retry: %d\r\n", mb_retry_count);
                        mbRTURetransmit(channel);
                    } else {
                        mb_retry_count = 0;
                    }
                }
                break;
            }
            break;
        }

        case SEG_SERIAL_MODBUS_ASCII :
            ASCII_Uart_RX(channel);
            if (mb_state_ascii_finish[channel] == TRUE) {
                mb_state_ascii_finish[channel] = FALSE;
                mbASCIItoTCP(sock, channel);
            }
            break;
        }
#ifdef __USE_WATCHDOG__
        device_wdt_reset();
#endif
    }
}

void seg_ch_recv_task(void *argument)  {
    int channel = (int)(uintptr_t)argument;
    uint8_t sock = seg_data_sock[channel];
    uint8_t serial_mode = get_serial_communation_protocol(channel);

    while (1) {
        seg_postmortem_mark(SEG_PM_TASK_RECV, (uint8_t)channel);
        seg_recv_step[channel] = SEG_RECV_STEP_HEAD;
        switch (serial_mode) {
        case SEG_SERIAL_PROTOCOL_NONE :
            ether_to_uart(sock, channel);
            break;

        case SEG_SERIAL_MODBUS_RTU :
            mbTCPtoRTU(sock, channel);
            break;

        case SEG_SERIAL_MODBUS_ASCII :
            mbTCPtoASCII(sock, channel);
            break;
        }
        vTaskDelay(1);
        seg_recv_step[channel] = SEG_RECV_STEP_TAIL;
#ifdef __USE_WATCHDOG__
        device_wdt_reset();
#endif
    }
}

void timers_stop(uint8_t channel) {
    if (seg_inactivity_timer[channel] != NULL) {
        xTimerStop(seg_inactivity_timer[channel], 0);
    }

    if (seg_keepalive_timer[channel] != NULL) {
        xTimerStop(seg_keepalive_timer[channel], 0);
    }

    if (seg_auth_timer[channel] != NULL) {
        xTimerStop(seg_auth_timer[channel], 0);
    }
}

void keepalive_timer_callback(TimerHandle_t xTimer) {
    int timer_id = (int)pvTimerGetTimerID(xTimer);

    if ((timer_id < 0) || (timer_id >= DEVICE_UART_CNT)) {
        return;
    }
    flag_send_keepalive[timer_id] = SEG_ENABLE;
    xSemaphoreGive(seg_timer_sem);
}

void inactivity_timer_callback(TimerHandle_t xTimer) {
    int timer_id = (int)pvTimerGetTimerID(xTimer);

    if ((timer_id < 0) || (timer_id >= DEVICE_UART_CNT)) {
        return;
    }
    flag_inactivity[timer_id] = SEG_ENABLE;
    xSemaphoreGive(seg_timer_sem);
}

void auth_timer_callback(TimerHandle_t xTimer) {
    int timer_id = (int)pvTimerGetTimerID(xTimer);

    if ((timer_id < 0) || (timer_id >= DEVICE_UART_CNT)) {
        return;
    }
    flag_auth_time[timer_id] = SEG_ENABLE;
    xSemaphoreGive(seg_timer_sem);
}

static void seg_keepalive_timer_restart(TimerHandle_t timer, uint32_t delay_ms) {
    TickType_t delay_ticks;

    if (timer == NULL) {
        return;
    }
    if (delay_ms == 0) {
        delay_ms = 1;
    }
    delay_ticks = pdMS_TO_TICKS(delay_ms);
    if (delay_ticks == 0) {
        delay_ticks = 1;
    }
    xTimerChangePeriod(timer, delay_ticks, 0);
    xTimerStart(timer, 0);
}

#if SEG_S2E_STALL_RECOVERY_ENABLE
// Polls every channel from an independent task so it remains available when a
// channel's data tasks stop making useful progress.
void seg_s2e_monitor_task(void *argument) {
    (void)argument;

    // Take over ioLibrary's critical-section callbacks so the SPI lock's holder is
    // recorded. Safe here: wizchip_cris_initialize() already ran during startup and
    // is never called again, and both callbacks use the same semaphore as before.
    //
    // Outside the diagnostics guard because the idle hook reads the result: with the
    // default callbacks installed the owner is permanently NULL, so the postmortem's
    // SPI-lock reason records zeros and says nothing. The added cost is two counter
    // updates, one timer read and a task-name pointer per critical section - no
    // stdio and no extra lock, unlike the instrumentation that had to be switched off.
    reg_wizchip_cris_cbfunc(seg_wiz_cris_enter, seg_wiz_cris_exit);

    for (;;) {
#if (DEVICE_UART_CNT > 2)
        // DATA2/DATA3 share one PIO RX consumer task. This poll is normally
        // silent and emits a snapshot only when that shared path stops making
        // progress for long enough to explain a simultaneous two-channel stall.
        pio_uart_rx_diag_poll();
#endif
        for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
#if SEG_S2E_STALL_RECOVERY_ENABLE
            seg_s2e_recovery_poll(ch);  // permanent raw-TCP send stall
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
#endif

void seg_timer_task(void *argument)  {
    struct __tcp_option *tcp_option;

    while (1) {
        xSemaphoreTake(seg_timer_sem, portMAX_DELAY);

        for (int i = 0; i < DEVICE_UART_CNT; i++) {
            tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[i]);
            if ((flag_inactivity[i] == SEG_ENABLE)) {
                flag_inactivity[i] = SEG_DISABLE;
#ifdef _SEG_DEBUG_
                PRT_SEG(" > INACTIVITY TIMER: TIMEOUT\r\n");
#endif
                process_socket_termination(seg_data_sock[i], SOCK_TERMINATION_DELAY, i, TRUE);
            }

            if (flag_send_keepalive[i] == SEG_ENABLE) {
                flag_send_keepalive[i] = SEG_DISABLE;

                if ((tcp_option->keepalive_en == SEG_ENABLE) &&
                        (get_device_status(i) == ST_CONNECT) &&
                        (seg_keepalive_timer[i] != NULL)) {
                    uint32_t now = (uint32_t)millis();
                    uint32_t last_activity = seg_tcp_last_activity_ms[i];
                    uint32_t wait_ms = seg_keepalive_interval_ms(
                                           tcp_option->keepalive_wait_time);
                    uint32_t retry_ms = seg_keepalive_interval_ms(
                                            tcp_option->keepalive_retry_time);
                    uint32_t idle_ms;
                    uint32_t next_ms;

                    if (last_activity == 0) {
                        seg_tcp_mark_activity(i);
                        last_activity = seg_tcp_last_activity_ms[i];
                    }
                    idle_ms = (uint32_t)(now - last_activity);

                    if (idle_ms < wait_ms) {
                        // Normal traffic itself proves liveness.  Defer the first
                        // probe until a full keepalive wait interval has elapsed.
                        next_ms = wait_ms - idle_ms;
                    } else {
                        // Revalidated under seg_socket_sem inside the helper.  If
                        // data became active meanwhile, no keepalive is issued.
                        (void)send_keepalive_packet_manual(seg_data_sock[i], i);
                        next_ms = retry_ms;
                    }
                    seg_keepalive_timer_restart(seg_keepalive_timer[i], next_ms);
                }
            }

            // Check the connection password auth timer
            if (tcp_option->pw_connect_en == SEG_ENABLE) {
                if ((flag_auth_time[i] == SEG_ENABLE) && (flag_connect_pw_auth[i] == SEG_DISABLE)) {
                    flag_auth_time[i] = SEG_DISABLE;

#ifdef _SEG_DEBUG_
                    printf(" > CONNECTION PW: AUTH TIMEOUT\r\n");
#endif
                    process_socket_termination(seg_data_sock[i], SOCK_TERMINATION_DELAY, i, TRUE);
                }
            }
        }
    }
}
