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
extern xSemaphoreHandle seg_sem[DEVICE_UART_CNT];
extern xSemaphoreHandle wizchip_critical_sem;

extern TimerHandle_t seg_inactivity_timer[DEVICE_UART_CNT];
extern TimerHandle_t seg_keepalive_timer[DEVICE_UART_CNT];
extern TimerHandle_t seg_auth_timer[DEVICE_UART_CNT];
extern TimerHandle_t spi_reset_timer;

uint16_t u2e_size[DEVICE_UART_CNT] = {0, };
uint16_t e2u_size[DEVICE_UART_CNT] = {0, };

// Stamped by seg_ch_task once per loop and read by seg_ch_u2e_task, which runs at a
// higher priority and otherwise has no way to tell that it is starving the task it
// shares seg_critical_sem with.
static volatile uint32_t seg_task_heartbeat_ms[DEVICE_UART_CNT];

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

// UDP: Peer netinfo
uint8_t peerip[4] = {0, };
uint8_t peerip_tmp[4] = {0xff, };
uint16_t peerport = 0;

// XON/XOFF (Software flow control) flag, Serial data can be transmitted to peer when XON enabled.
uint8_t isXON = SEG_ENABLE;

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

uint8_t check_connect_pw_auth(uint8_t * buf, uint16_t len);
uint8_t check_tcp_connect_exception(int channel);
void reset_SEG_timeflags(uint8_t channel);

uint16_t get_tcp_any_port(void);

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

    // Status indicator pins
    if (network_connection->working_state == ST_CONNECT) {
        if (device_option->device_eth_connect_data[channel][0] != 0) {
            struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option[channel]);
            struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);

            if (network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE) {
                wizchip_mqtt_publish(&g_mqtt_config[channel], mqtt_option->pub_topic, mqtt_option->qos, device_option->device_eth_connect_data[channel], strlen((char *)device_option->device_eth_connect_data[channel]));
            }
#ifdef __USE_S2E_OVER_TLS__
            else if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                wiz_tls_write(&s2e_tlsContext[channel], device_option->device_eth_connect_data[channel], strlen((char *)device_option->device_eth_connect_data[channel]));
            }
#endif
            else {
                (int16_t)send(seg_data_sock[channel], device_option->device_eth_connect_data[channel], strlen((char *)device_option->device_eth_connect_data[channel]));
            }

            if ((tcp_option->keepalive_en == ENABLE) &&
                    (flag_first_keepalive[channel] == DISABLE) &&
                    (seg_keepalive_timer[channel] != NULL)) {
                flag_first_keepalive[channel] = ENABLE;
                xTimerStart(seg_keepalive_timer[channel], 0);
            }
        }

        if (device_option->device_serial_connect_data[channel] != 0) {
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
        break;

    case SOCK_CLOSED:
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
        // flag was assembled above and then thrown away: socket() got a literal 0,
        // so multicast was never enabled and the socket opened blocking - sendto()
        // then sits in ioLibrary's SENDOK polling loop instead of returning SOCK_BUSY.
        // SOCK_IO_NONBLOCK is also the wrong name here; it belongs to setsockopt().
        // Both constants happen to be 1, so only the discarded flag changed behaviour.
        //
        // Original:
        //     flag |= SOCK_IO_NONBLOCK;
        //     int8_t s = socket(sock, Sn_MR_UDP, network_connection->local_port, 0);
        flag |= SF_IO_NONBLOCK;
        int8_t s = socket(sock, Sn_MR_UDP, network_connection->local_port, flag);

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
        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
        PRT_SEG(" > SEG:TCP_CLIENT_MODE:ConnectNetwork Err %d\r\n", ret);

#ifdef _SEG_DEBUG_
        PRT_SEG(" > SEG:TCP_CLIENT_MODE:CLIENT_CONNECTION\r\n");
#endif
        break;

    case SOCK_ESTABLISHED:
        if (getSn_IR(sock) & Sn_IR_CON) {
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // S2E: TCP client mode initialize after connection established (only once)
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // Interrupt clear
            reg_val = SIK_CONNECTED & 0x00FF; // except SIK_SENT(send OK) interrupt
            ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                getsockopt(sock, SO_DESTIP, &destip);
                getsockopt(sock, SO_DESTPORT, &destport);
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
                                         pdMS_TO_TICKS(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }
            set_device_status(ST_CONNECT, channel);
        }
        break;

    case SOCK_CLOSE_WAIT:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        disconnect(sock);
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
        int8_t s = socket(sock, Sn_MR_TCP, source_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

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
    static uint8_t first_established;

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

        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
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
        first_established = 1;

        PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: TCP CLIENT CONNECTED\r\n");
        break;

    case SOCK_ESTABLISHED:
        //if(getSn_IR(sock) & Sn_IR_CON)
        if (first_established) {
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // S2E: TCP client mode initialize after connection established (only once)
            ///////////////////////////////////////////////////////////////////////////////////////////////////

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                getsockopt(sock, SO_DESTIP, &destip);
                getsockopt(sock, SO_DESTPORT, &destport);
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
                                         pdMS_TO_TICKS(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }

            first_established = 0;
            set_device_status(ST_CONNECT, channel);
        }
        break;

    case SOCK_CLOSE_WAIT:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        disconnect(sock);
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
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
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
    static uint8_t first_established;

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
        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
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
        first_established = 1;
        break;

    case SOCK_ESTABLISHED:
        if (first_established) {
            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                getsockopt(sock, SO_DESTIP, &destip);
                getsockopt(sock, SO_DESTPORT, &destport);
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
                                         pdMS_TO_TICKS(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }
            first_established = 0;
            set_device_status(ST_CONNECT, channel);
        }
        mqtt_transport_yield(&g_mqtt_config[channel]);
        break;

    case SOCK_CLOSE_WAIT:
        disconnect(sock);
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
        int8_t s = socket(sock, Sn_MR_TCP, source_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

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
    static uint8_t first_established;

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

        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
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
        first_established = 1;
        break;

    case SOCK_ESTABLISHED:
        if (first_established) {
            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                getsockopt(sock, SO_DESTIP, &destip);
                getsockopt(sock, SO_DESTPORT, &destport);
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
                                         pdMS_TO_TICKS(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }

            first_established = 0;
            set_device_status(ST_CONNECT, channel);
        }
        mqtt_transport_yield(&g_mqtt_config[channel]);
        break;

    case SOCK_CLOSE_WAIT:
        disconnect(sock);
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
    case SOCK_INIT:
        break;

    case SOCK_LISTEN:
        break;

    case SOCK_ESTABLISHED:
        if (getSn_IR(sock) & Sn_IR_CON) {
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
                getsockopt(sock, SO_DESTIP, &destip);
                getsockopt(sock, SO_DESTPORT, &destport);
                PRT_SEG(" > SEG:CONNECTED FROM - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
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
                                         pdMS_TO_TICKS(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
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
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
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
        int8_t s = socket(sock, Sn_MR_TCP, network_connection->local_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

        if (s == sock) {
            // Replace the command mode switch code GAP time (default: 500ms)
            if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                modeswitch_gap_time = serial_data_packing->packing_time;
            }

            // TCP Server listen
            listen(sock);

            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:TCP_SERVER_MODE:SOCKOPEN\r\n");
            }
        } else {
            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:TCP_SERVER_MODE:SOCKOPEN FAILED\r\n");
            }
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
        }
        break;

    default:
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
    static uint8_t reconnection_count = 0;
#endif
    switch (state) {
    case SOCK_INIT:
        if (mixed_state[channel] == MIXED_CLIENT) {
            if (reconnection_count && tcp_option->reconnection) {
                vTaskDelay(tcp_option->reconnection);
            }

            // TCP connect exception checker; e.g., dns failed / zero srcip ... and etc.
            if (check_tcp_connect_exception(channel) == ON) {
#ifdef MIXED_CLIENT_LIMITED_CONNECT
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                reconnection_count = 0;
                data_buffer_flush(channel);
                mixed_state[channel] = MIXED_SERVER;
#endif
                return;
            }

            // TCP connect
            ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
            PRT_SEG(" > SEG:TCP_MIXED_MODE:ConnectNetwork Err %d\r\n", ret);

#ifdef MIXED_CLIENT_LIMITED_CONNECT
            reconnection_count++;

            if (reconnection_count >= network_option->tcp_rcr_val) {
                PRT_SEG("reconnection_count >= network_option->tcp_rcr_val\r\n");
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, channel, FALSE);
                reconnection_count = 0;
                data_buffer_flush(channel);
                mixed_state[channel] = MIXED_SERVER;
            }
#ifdef _SEG_DEBUG_
            if (reconnection_count != 0) {
                PRT_SEG(" > SEG:TCP_MIXED_MODE:CLIENT_CONNECTION [%d]\r\n", reconnection_count);
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
        if (getSn_IR(sock) & Sn_IR_CON) {
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            // S2E: TCP mixed (server or client) mode initialize after connection established (only once)
            ///////////////////////////////////////////////////////////////////////////////////////////////////
            reg_val = SIK_CONNECTED & 0x00FF;
            ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                getsockopt(sock, SO_DESTIP, &destip);
                getsockopt(sock, SO_DESTPORT, &destport);

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
                                         pdMS_TO_TICKS(tcp_option->keepalive_wait_time),
                                         pdFALSE,
                                         channel,
                                         keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer[channel]) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer[channel], 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer[channel], pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
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
                                             pdTRUE,
                                             channel,
                                             auth_timer_callback) == pdPASS) {
                        xTimerStart(seg_auth_timer[channel], 0);
                    }
                }
            } else {
                if (get_data_buffer_usedsize(channel) || u2e_size) {
                    xSemaphoreGive(seg_u2e_sem[channel]);
                }
                mixed_state[channel] = MIXED_SERVER;
            }

#ifdef MIXED_CLIENT_LIMITED_CONNECT
            reconnection_count = 0;
#endif
        }
        break;

    case SOCK_CLOSE_WAIT:
        PRT_SEG("case SOCK_CLOSE_WAIT\r\n");
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            drain_socket_to_uart(sock, channel);
        }
        disconnect(sock);
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

            int8_t s = socket(sock, Sn_MR_TCP, network_connection->local_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

            if (s == sock) {
                // Replace the command mode switch code GAP time (default: 500ms)
                if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                    modeswitch_gap_time = serial_data_packing->packing_time;
                }

                // TCP Server listen
                listen(sock);

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
            int8_t s = socket(sock, Sn_MR_TCP, source_port, (SF_TCP_NODELAY | SF_IO_NONBLOCK));

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

    len = get_serial_data(channel);

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
                if ((peerip[0] == 0x00) && (peerip[1] == 0x00) && (peerip[2] == 0x00) && (peerip[3] == 0x00)) {
                    if (serial_common->serial_debug_en) {
                        PRT_SEG(" > SEG:UDP_MODE:DATA SEND FAILED - UDP Peer IP/Port required (0.0.0.0)\r\n");
                    }
                } else {
                    sent_len = (int16_t)sendto(sock, g_send_buf[channel], len, peerip, peerport);    // UDP 1:N mode
                }
            } else {
                sent_len = (int16_t)sendto(sock, g_send_buf[channel], len, network_connection->remote_ip, network_connection->remote_port);    // UDP 1:1 mode
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
                // Original:
                //     sent_len = (int16_t)send(sock, g_send_buf[channel], len);
                uint16_t freesize = getSn_TX_FSR(sock);

                if (len > freesize) {
                    len = freesize;
                }

                if (len > 0) {
                    sent_len = (int16_t)send(sock, g_send_buf[channel], len);
                }
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

    if ((len + u2e_size[channel]) >= DATA_BUF_SIZE) { // Avoiding u2e buffer (g_send_buf) overflow
        /* Checking Data packing option: character delimiter */
        if ((serial_data_packing->packing_delimiter[0] != 0x00) && (len == 1)) {
            g_send_buf[channel][u2e_size[channel]] = (uint8_t)data_buffer_getc(channel);
            if (serial_data_packing->packing_delimiter[0] == g_send_buf[channel][u2e_size[channel]]) {
                return u2e_size[channel];
            }
        }

        // serial data length value update for avoiding u2e buffer overflow
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

void ether_to_uart(uint8_t sock, int channel) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[channel]);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[channel]);
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[channel]);

    uint16_t len;
    uint16_t i;
    uint16_t reg_val;

    // The E2S stall lives here: platform_uart_cts_ready() reports ready
    // unconditionally on the HW channels because the PL011 gates CTS itself, so a
    // peer holding CTS off stops the wire, the transmit DMA never completes, and the
    // platform_uart_tx_wait() below never returns - taking the whole receive task
    // with it. Every observed E2S stall was DATA0 or DATA1 for that reason; the PIO
    // channels read their CTS pin and do return early here.
    //
    // Returning early on a busy DMA was tried and reverted: it fixed the wedge but
    // cost about 12 % of E2S throughput on every channel, because the task then
    // waits a whole tick between chunks instead of queueing the next one the moment
    // the DMA frees up. A working fix has to keep the pipe full - poll at finer
    // granularity, or bound the wait without letting recv() overwrite a buffer the
    // DMA is still reading.
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
            len = getSn_RX_RSR(sock);
            if (len > DATA_BUF_SIZE) {
                len = DATA_BUF_SIZE;    // avoiding buffer overflow
            }

            if (len > 0) {
                if (seg_inactivity_timer[channel] != NULL) {
                    xTimerReset(seg_inactivity_timer[channel], 0);
                }

                // The previous transmit may still be reading g_recv_buf by DMA.
                platform_uart_tx_wait(channel);

                if (network_connection->working_mode == UDP_MODE) {
                    e2u_size[channel] = recvfrom(sock, g_recv_buf[channel], len, peerip, &peerport);

                    if (memcmp(peerip_tmp, peerip, 4) !=  0) {
                        memcpy(peerip_tmp, peerip, 4);
                        if (serial_common->serial_debug_en) {
                            PRT_SEG(" > UDP Peer IP/Port: %d.%d.%d.%d : %d\r\n", peerip[0], peerip[1], peerip[2], peerip[3], peerport);
                        }
                    }
                    //} else if (network_connection->working_state == ST_CONNECT) {
                } else {
#ifdef __USE_S2E_OVER_TLS__
                    if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                        e2u_size[channel] = wiz_tls_read(&s2e_tlsContext[channel], g_recv_buf[channel], len);
                    }
#endif
                    else {
                        e2u_size[channel] = recv(sock, g_recv_buf[channel], len);
                    }
                }
                reg_val = SIK_RECEIVED & 0x00FF;
                ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);
            } else {
                break;
            }

            if ((network_connection->working_mode == TCP_SERVER_MODE) ||  \
                    ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state[channel] == MIXED_SERVER))) {
                // Connection password authentication
                if ((tcp_option->pw_connect_en == SEG_ENABLE) && (flag_connect_pw_auth[channel] == SEG_DISABLE)) {
                    if (check_connect_pw_auth(g_recv_buf[channel], len) == SEG_ENABLE) {
                        flag_connect_pw_auth[channel] = SEG_ENABLE;
                    } else {
                        flag_connect_pw_auth[channel] = SEG_DISABLE;
                    }
                    e2u_size[channel] = 0;

                    if (seg_auth_timer[channel] != NULL) {
                        xTimerStop(seg_auth_timer[channel], 0);
                    }
                    if (flag_connect_pw_auth[channel] == SEG_DISABLE) {
                        disconnect(sock);
                        return;
                    }
                }
            }
        }
        // Ethernet data transfer to DATA UART
        if (e2u_size[channel] != 0) {
            //////////////////////////////////////////////////////////////////////
#ifdef __USE_UART_485_422__
            if ((serial_option->uart_interface == UART_IF_RS422) ||
                    (serial_option->uart_interface == UART_IF_RS485)) {
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
                if (isXON == SEG_ENABLE) {
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

void ether_to_spi(uint8_t sock) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option[SEG_DATA0_CH]);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection[SEG_DATA0_CH]);
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option[SEG_DATA0_CH]);
    struct __device_option *device_option = (struct __device_option *) & (get_DevConfig_pointer()->device_option);

    uint16_t len;
    uint16_t i;
    uint16_t reg_val;

    do {
        // H/W Socket buffer -> User's buffer
        if (!(network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE)) {
            len = getSn_RX_RSR(sock);
            if (len > DATA_BUF_SIZE) {
                len = DATA_BUF_SIZE;    // avoiding buffer overflow
            }

            if (len > 0) {
                if (network_connection->working_mode == UDP_MODE) {
                    e2u_size[SEG_DATA0_CH] = recvfrom(sock, g_recv_buf[SEG_DATA0_CH], len, peerip, &peerport);

                    if (memcmp(peerip_tmp, peerip, 4) !=  0) {
                        memcpy(peerip_tmp, peerip, 4);
                        if (serial_common->serial_debug_en) {
                            printf(" > UDP Peer IP/Port: %d.%d.%d.%d : %d\r\n", peerip[0], peerip[1], peerip[2], peerip[3], peerport);
                        }
                    }
                } else if (network_connection->working_state == ST_CONNECT) {
#ifdef __USE_S2E_OVER_TLS__
                    if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                        e2u_size[SEG_DATA0_CH] = wiz_tls_read(&s2e_tlsContext[SEG_DATA0_CH], g_recv_buf[SEG_DATA0_CH], len);
                    }
#endif
                    else {
                        e2u_size[SEG_DATA0_CH] = recv(sock, g_recv_buf[SEG_DATA0_CH], len);
                    }
                }
                reg_val = SIK_RECEIVED & 0x00FF;
                ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);
            } else {
                break;
            }

            if ((network_connection->working_mode == TCP_SERVER_MODE) ||  \
                    ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state[SEG_DATA0_CH] == MIXED_SERVER))) {
                // Connection password authentication
                if ((tcp_option->pw_connect_en == SEG_ENABLE) && (flag_connect_pw_auth[SEG_DATA0_CH] == SEG_DISABLE)) {
                    if (check_connect_pw_auth(g_recv_buf[SEG_DATA0_CH], len) == SEG_ENABLE) {
                        flag_connect_pw_auth[SEG_DATA0_CH] = SEG_ENABLE;
                    } else {
                        flag_connect_pw_auth[SEG_DATA0_CH] = SEG_DISABLE;
                    }
                    e2u_size[SEG_DATA0_CH] = 0;

                    if (seg_auth_timer[SEG_DATA0_CH] != NULL) {
                        xTimerStop(seg_auth_timer[SEG_DATA0_CH], 0);
                    }
                    if (flag_connect_pw_auth[SEG_DATA0_CH] == SEG_DISABLE) {
                        disconnect(sock);
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

void send_keepalive_packet_manual(uint8_t sock) {
    setsockopt(sock, SO_KEEPALIVESEND, 0);
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
                ret = disconnect(sock);
                if ((ret == SOCK_OK) || (ret == SOCKERR_TIMEOUT)) {
                    break;
                }
            } while ((millis() - tickStart) < timeout);
        }
    }

    close(sock);
    if (mutex == TRUE) {
        xSemaphoreGive(seg_critical_sem[channel]);
    }
    xSemaphoreGive(seg_sem[channel]);
    return sock;
}

uint8_t check_connect_pw_auth(uint8_t * buf, uint16_t len) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);

    uint8_t ret = SEG_DISABLE;
    uint8_t pwbuf[11] = {0,};

    if (len >= sizeof(pwbuf)) {
        len = sizeof(pwbuf) - 1;
    }

    memcpy(pwbuf, buf, len);
    if ((len == strlen(tcp_option->pw_connect)) && (memcmp(tcp_option->pw_connect, pwbuf, len) == 0)) {
        ret = SEG_ENABLE; // Connection password auth success
    }

#ifdef _SEG_DEBUG_
    PRT_SEG(" > Connection password: %s, len: %d\r\n", tcp_option->pw_connect, strlen(tcp_option->pw_connect));
    PRT_SEG(" > Entered password: %s, len: %d\r\n", pwbuf, len);
    PRT_SEG(" >> Auth %s\r\n", ret ? "success" : "failed");
#endif

    return ret;
}

void init_trigger_modeswitch(uint8_t mode) {
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection;

    if (mode == DEVICE_AT_MODE) {
        opmode = DEVICE_AT_MODE;
        set_device_status(ST_ATMODE, SEG_DATA0_CH);
        set_device_status(ST_ATMODE, SEG_DATA1_CH);

        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:AT Mode\r\n");
            platform_uart_puts((uint8_t *)"SEG:AT Mode\r\n", strlen("SEG:AT Mode\r\n"), SEG_DATA0_CH);
        }
    } else { // DEVICE_GW_MODE
        opmode = DEVICE_GW_MODE;
        set_device_status(ST_OPEN, SEG_DATA0_CH);
        set_device_status(ST_OPEN, SEG_DATA1_CH);

        for (int i = 0; i < DEVICE_UART_CNT; i++) {
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

    u2e_size[SEG_DATA0_CH] = 0;
    u2e_size[SEG_DATA1_CH] = 0;
    data_buffer_flush(SEG_DATA0_CH);
    data_buffer_flush(SEG_DATA1_CH);
    reset_SEG_timeflags(SEG_DATA0_CH);
    reset_SEG_timeflags(SEG_DATA1_CH);
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
            isXON = SEG_ENABLE;
            ret = SEG_DISABLE;
        } else if (ch == UART_XOFF) {
            isXON = SEG_DISABLE;
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
        send(sock, buf, len);
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
                //#ifdef _SEG_DEBUG_
                //            PRT_SEG(" >> send_keepalive_packet\r\n");
                //#endif
                flag_send_keepalive[i] = SEG_DISABLE;
                send_keepalive_packet_manual(seg_data_sock[i]);    // <-> send_keepalive_packet_auto()

                if (seg_keepalive_timer[i] != NULL) {
                    if (xTimerGetPeriod(seg_keepalive_timer[i]) != pdMS_TO_TICKS(tcp_option->keepalive_retry_time)) {
                        xTimerChangePeriod(seg_keepalive_timer[i], pdMS_TO_TICKS(tcp_option->keepalive_retry_time), 0);
                    }
                    xTimerStart(seg_keepalive_timer[i], 0);
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
