#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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
#include "spiHandler.h"

//#include <semphr.h>

#include "port_common.h"

// TLS support
#ifdef __USE_S2E_OVER_TLS__
#include "SSLInterface.h"
wiz_tls_context s2e_tlsContext;
#endif

#include "mqtt_transport_interface.h"

#include "netHandler.h"
#include "seg.h"

//Modbus supprot
#include "mb.h"
#include "mbrtu.h"
#include "mbascii.h"
#include "mbserial.h"

/* Private define ------------------------------------------------------------*/
#ifndef SERIAL_MODE
#define SERIAL_MODE SEG_SERIAL_PROTOCOL_NONE
#endif

#ifndef OPMODE
#define OPMODE TCP_CLIENT_MODE
#endif

// Ring Buffer
BUFFER_DECLARATION(data0_rx);

/* Private variables ---------------------------------------------------------*/
uint8_t opmode = DEVICE_GW_MODE;
uint8_t sw_modeswitch_at_mode_on = SEG_DISABLE;

// static variables for function: check_modeswitch_trigger()
uint8_t triggercode_idx;
uint8_t ch_tmp[3];

// Gateway mode <-> command mode switch gap time
uint8_t enable_modeswitch_timer = SEG_DISABLE;
volatile uint16_t modeswitch_time = 0;
volatile uint16_t modeswitch_gap_time = DEFAULT_MODESWITCH_INTER_GAP;

uint8_t mixed_state = MIXED_SERVER;
uint16_t client_any_port = 0;

uint8_t enable_serial_input_timer = SEG_DISABLE;
volatile uint16_t serial_input_time = 0;
uint8_t flag_serial_input_time_elapse = SEG_DISABLE; // for Time delimiter

// flags
uint8_t flag_connect_pw_auth = SEG_DISABLE; // TCP_SERVER_MODE only
uint8_t flag_auth_time = SEG_DISABLE; // TCP_SERVER_MODE only
uint8_t flag_send_keepalive = SEG_DISABLE;
uint8_t flag_first_keepalive = SEG_DISABLE;
uint8_t flag_inactivity = SEG_DISABLE;

// User's buffer / size idx
extern uint8_t g_send_buf[DATA_BUF_SIZE];
extern uint8_t g_recv_buf[DATA_BUF_SIZE];
extern uint8_t g_recv_mqtt_buf[DATA_BUF_SIZE];

/*the flag of modbus*/
extern volatile uint8_t mb_state_rtu_finish;
extern volatile uint8_t mb_state_ascii_finish;

extern xSemaphoreHandle seg_e2u_sem;
extern xSemaphoreHandle seg_u2e_sem;
extern xSemaphoreHandle seg_spi_pending_sem;
extern xSemaphoreHandle seg_sem;
extern xSemaphoreHandle net_seg_sem;
extern xSemaphoreHandle seg_timer_sem;
extern xSemaphoreHandle segcp_uart_sem;
extern xSemaphoreHandle seg_critical_sem;

extern TimerHandle_t seg_inactivity_timer;
extern TimerHandle_t seg_keepalive_timer;
extern TimerHandle_t seg_auth_timer;
extern TimerHandle_t spi_reset_timer;

extern TaskHandle_t seg_mqtt_yield_task_handle;

int u2e_size = 0;
int e2u_size = 0;

// UDP: Peer netinfo
uint8_t peerip[4] = {0, };
uint8_t peerip_tmp[4] = {0xff, };
uint16_t peerport = 0;

// XON/XOFF (Software flow control) flag, Serial data can be transmitted to peer when XON enabled.
uint8_t isXON = SEG_ENABLE;

char * str_working[] = {"TCP_CLIENT_MODE", "TCP_SERVER_MODE", "TCP_MIXED_MODE", "UDP_MODE", "SSL_TCP_CLIENT_MODE", "MQTT_CLIENT_MODE", "MQTTS_CLIENT_MODE"};

NetworkContext_t g_network_context;
TransportInterface_t g_transport_interface;
mqtt_config_t g_mqtt_config;

/* Private functions prototypes (defined in seg_proc.c) ----------------------*/
static void seg_u2e_none(void);
static void seg_u2e_modbus_rtu(void);
static void seg_u2e_modbus_ascii(void);
static void seg_recv_none(void);
static void seg_recv_modbus_rtu(void);
static void seg_recv_modbus_ascii(void);

extern void proc_SEG_tcp_client(uint8_t sock);
extern void proc_SEG_tcp_server(uint8_t sock);
extern void proc_SEG_tcp_mixed(uint8_t sock);
extern void proc_SEG_udp(uint8_t sock);
extern void proc_SEG_mqtt_client(uint8_t sock);
extern void proc_SEG_mqtts_client(uint8_t sock);

#ifdef __USE_S2E_OVER_TLS__
extern void proc_SEG_tcp_client_over_tls(uint8_t sock);
#endif

/* OPMODE: compile-time operation mode selection */
#if OPMODE == TCP_CLIENT_MODE
static seg_proc_fn_t seg_proc_fn = proc_SEG_tcp_client;
#elif OPMODE == TCP_SERVER_MODE
static seg_proc_fn_t seg_proc_fn = proc_SEG_tcp_server;
#elif OPMODE == TCP_MIXED_MODE
static seg_proc_fn_t seg_proc_fn = proc_SEG_tcp_mixed;
#elif OPMODE == UDP_MODE
static seg_proc_fn_t seg_proc_fn = proc_SEG_udp;
#elif OPMODE == MQTT_CLIENT_MODE
static seg_proc_fn_t seg_proc_fn = proc_SEG_mqtt_client;
#elif OPMODE == SSL_TCP_CLIENT_MODE
static seg_proc_fn_t seg_proc_fn = proc_SEG_tcp_client_over_tls;
#elif OPMODE == MQTTS_CLIENT_MODE
static seg_proc_fn_t seg_proc_fn = proc_SEG_mqtts_client;
#else
#error "OPMODE is not defined or invalid. Use CMake: -DOPMODE=TCP_CLIENT_MODE"
#endif

typedef void (*seg_u2e_fn_t)(void);
#if SERIAL_MODE == SEG_SERIAL_PROTOCOL_NONE
static seg_u2e_fn_t seg_u2e_fn = seg_u2e_none;
#elif SERIAL_MODE == SEG_SERIAL_MODBUS_RTU
static seg_u2e_fn_t seg_u2e_fn = seg_u2e_modbus_rtu;
#elif SERIAL_MODE == SEG_SERIAL_MODBUS_ASCII
static seg_u2e_fn_t seg_u2e_fn = seg_u2e_modbus_ascii;
#else
#error "SERIAL_MODE is not defined or invalid. Use CMake: -DSERIAL_MODE=SEG_SERIAL_PROTOCOL_NONE"
#endif

typedef void (*seg_recv_fn_t)(void);
#if SERIAL_MODE == SEG_SERIAL_PROTOCOL_NONE
static seg_recv_fn_t seg_recv_fn = seg_recv_none;
#elif SERIAL_MODE == SEG_SERIAL_MODBUS_RTU
static seg_recv_fn_t seg_recv_fn = seg_recv_modbus_rtu;
#elif SERIAL_MODE == SEG_SERIAL_MODBUS_ASCII
static seg_recv_fn_t seg_recv_fn = seg_recv_modbus_ascii;
#else
#error "SERIAL_MODE is not defined or invalid. Use CMake: -DSERIAL_MODE=SEG_SERIAL_PROTOCOL_NONE"
#endif


/* Public & Private functions ------------------------------------------------*/

void do_seg(uint8_t sock) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option);

    if (opmode == DEVICE_GW_MODE) {
        seg_proc_fn(sock);

        // XON/XOFF Software flow control: Check the Buffer usage and Send the start/stop commands
        // [WIZnet Device] -> [Peer]
        if ((serial_option->flow_control == flow_xon_xoff) || (serial_option->flow_control == flow_rts_cts)) {
            check_uart_flow_control(serial_option->flow_control);
        }
    }
}

void set_device_status(teDEVSTATUS status) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
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

    if (network_connection->working_state == ST_CONNECT) {
        if (device_option->device_eth_connect_data[0] != 0) {
            struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option);
            struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);

            if (network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE) {
                wizchip_mqtt_publish(&g_mqtt_config, mqtt_option->pub_topic, mqtt_option->qos, device_option->device_eth_connect_data, strlen((char *)device_option->device_eth_connect_data));
            }
#ifdef __USE_S2E_OVER_TLS__
            else if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                wiz_tls_write(&s2e_tlsContext, device_option->device_eth_connect_data, strlen((char *)device_option->device_eth_connect_data));
            }
#endif
            else {
                (int16_t)send(SEG_DATA0_SOCK, device_option->device_eth_connect_data, strlen((char *)device_option->device_eth_connect_data));
            }

            if (tcp_option->keepalive_en == ENABLE && flag_first_keepalive == DISABLE) {
                flag_first_keepalive = ENABLE;
                xTimerStart(seg_keepalive_timer, 0);
            }
        }

        if (device_option->device_serial_connect_data[0] != 0) {
            platform_uart_puts((const char *)device_option->device_serial_connect_data, strlen((const char *)device_option->device_serial_connect_data));
        }

        // Status indicator pins
#if (DEVICE_BOARD_NAME == PLATYPUS_S2E)
        set_connection_status_io(STATUS_TCPCONNECT_PIN, OFF); // Status I/O pin to low
#else
        set_connection_status_io(STATUS_TCPCONNECT_PIN, ON); // Status I/O pin to high
#endif
    }

    else if (prev_working_state == ST_CONNECT && network_connection->working_state == ST_OPEN) {
        if (device_option->device_serial_disconnect_data[0] != 0) {
            platform_uart_puts((const char *)device_option->device_serial_disconnect_data, strlen((const char *)device_option->device_serial_disconnect_data));
        }
#if (DEVICE_BOARD_NAME == PLATYPUS_S2E)
        // Status indicator pins
        set_connection_status_io(STATUS_TCPCONNECT_PIN, ON); // Status I/O pin to low
    } else {
        set_connection_status_io(STATUS_TCPCONNECT_PIN, ON);    // Status I/O pin to high
    }
#else
        // Status indicator pins
        set_connection_status_io(STATUS_TCPCONNECT_PIN, OFF); // Status I/O pin to low
    } else {
        set_connection_status_io(STATUS_TCPCONNECT_PIN, OFF);    // Status I/O pin to high
    }
#endif
}

uint8_t get_device_status(void) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    return network_connection->working_state;
}

// This function have to call every 1 millisecond by Timer IRQ handler routine.
void seg_timer_msec(void) {
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);

    signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    // Serial data packing time delimiter timer
    if (enable_serial_input_timer) {
        if (serial_input_time < serial_data_packing->packing_time) {
            serial_input_time++;
        } else {
            serial_input_time = 0;
            enable_serial_input_timer = 0;
            flag_serial_input_time_elapse = SEG_ENABLE;

            switch (network_connection->working_mode) {
            case TCP_CLIENT_MODE:
            case TCP_SERVER_MODE:
            case TCP_MIXED_MODE:
            case SSL_TCP_CLIENT_MODE:
            case UDP_MODE:
            case MQTT_CLIENT_MODE:
            case MQTTS_CLIENT_MODE:
                xSemaphoreGiveFromISR(seg_u2e_sem, &xHigherPriorityTaskWoken);
                portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
                break;
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
            sw_modeswitch_at_mode_on = SEG_ENABLE;  // success}
#ifdef ENABLE_SEGCP
            xSemaphoreGiveFromISR(segcp_uart_sem, &xHigherPriorityTaskWoken);
            portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif
        } else {
            restore_serial_data(triggercode_idx);    // failed
        }

        triggercode_idx = 0;
        enable_modeswitch_timer = SEG_DISABLE;
    }
}


/*
    main Task for Serial to Ethernet Gateway
    - Check the network link status and do the S2E data transfer task trigger
    - Check the mode switch trigger (AT mode <-> GW mode) and do the mode switch task trigger
    seg_task() -> task init

    seg_u2e_task() -> Serial to Ethernet data transfer task
    seg_recv_task() -> Ethernet to Serial data transfer task
    seg_timer_task() -> Timer event handling task
    (keep-alive, inactivity, connection password authentication, etc)

*/
void seg_task(void *argument)  {

    while (1) {
        if (get_net_status() == NET_LINK_DISCONNECTED) {
            PRT_SEGCP("get_net_status() != NET_LINK_DISCONNECTED\r\n");
            xSemaphoreTake(net_seg_sem, portMAX_DELAY);
        }
        xSemaphoreTake(seg_critical_sem, portMAX_DELAY);
        do_seg(SEG_DATA0_SOCK);
        xSemaphoreGive(seg_critical_sem);
        xSemaphoreTake(seg_sem, pdMS_TO_TICKS(10));
        //vTaskDelay(pdMS_TO_TICKS(10)); // wait for 10ms
    }
}

/* seg_u2e handlers */
static void seg_u2e_none(void) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    xSemaphoreTake(seg_critical_sem, portMAX_DELAY);
    if (get_data_buffer_usedsize() || u2e_size) {
        if ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state == MIXED_SERVER) && (ST_OPEN == get_device_status())) {
            process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY, FALSE);
            mixed_state = MIXED_CLIENT;
            xSemaphoreGive(seg_sem);
        } else if ((ST_CONNECT == get_device_status()) || network_connection->working_mode == UDP_MODE) {
            uart_to_ether(SEG_DATA0_SOCK);
        }
    }
    xSemaphoreGive(seg_critical_sem);
}

static void seg_u2e_modbus_rtu(void) {
    RTU_Uart_RX();
    if (mb_state_rtu_finish == TRUE) {
        mb_state_rtu_finish = FALSE;
        mbRTUtoTCP(SEG_DATA0_SOCK);
    }
}

static void seg_u2e_modbus_ascii(void) {
    ASCII_Uart_RX();
    if (mb_state_ascii_finish == TRUE) {
        mb_state_ascii_finish = FALSE;
        mbASCIItoTCP(SEG_DATA0_SOCK);
    }
}




void seg_u2e_task(void *argument)  {
    PRT_SEG("Running Task\r\n");
    while (1) {
        xSemaphoreTake(seg_u2e_sem, portMAX_DELAY);
        seg_u2e_fn();
#ifdef __USE_WATCHDOG__
        device_wdt_reset();
#endif
    }
}

/* seg_recv handlers */
static void seg_recv_none(void) {
    if (get_uart_spi_if()) {
        ether_to_spi(SEG_DATA0_SOCK);
    } else {
        ether_to_uart(SEG_DATA0_SOCK);
    }
}

static void seg_recv_modbus_rtu(void) {
    mbTCPtoRTU(SEG_DATA0_SOCK);
}

static void seg_recv_modbus_ascii(void) {
    mbTCPtoASCII(SEG_DATA0_SOCK);
}


void seg_recv_task(void *argument)  {
    while (1) {
        xSemaphoreTake(seg_e2u_sem, portMAX_DELAY);
        seg_recv_fn();
#ifdef __USE_WATCHDOG__
        device_wdt_reset();
#endif
    }
}

void timers_stop(void) {
    if (seg_inactivity_timer != NULL) {
        xTimerStop(seg_inactivity_timer, 0);
    }

    if (seg_keepalive_timer != NULL) {
        xTimerStop(seg_keepalive_timer, 0);
    }

    if (seg_auth_timer != NULL) {
        xTimerStop(seg_auth_timer, 0);
    }
}

void keepalive_timer_callback(TimerHandle_t xTimer) {
    flag_send_keepalive = SEG_ENABLE;
    xSemaphoreGive(seg_timer_sem);
}

void inactivity_timer_callback(TimerHandle_t xTimer) {
    flag_inactivity = SEG_ENABLE;
    xSemaphoreGive(seg_timer_sem);
}

void auth_timer_callback(TimerHandle_t xTimer) {
    flag_auth_time = SEG_ENABLE;
    xSemaphoreGive(seg_timer_sem);
}

void seg_timer_task(void *argument)  {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);

    while (1) {
        xSemaphoreTake(seg_timer_sem, portMAX_DELAY);
        if ((flag_inactivity == SEG_ENABLE)) {
#ifdef _SEG_DEBUG_
            PRT_SEG(" > INACTIVITY TIMER: TIMEOUT\r\n");
            flag_inactivity = SEG_DISABLE;
#endif
            process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY, TRUE);
        }

        if (flag_send_keepalive == SEG_ENABLE) {
            flag_send_keepalive = SEG_DISABLE;
            send_keepalive_packet_manual(SEG_DATA0_SOCK); // <-> send_keepalive_packet_auto()
            if (xTimerGetPeriod(seg_keepalive_timer) != pdMS_TO_TICKS(tcp_option->keepalive_retry_time)) {
                xTimerChangePeriod(seg_keepalive_timer, pdMS_TO_TICKS(tcp_option->keepalive_retry_time), 0);
            }
            xTimerStart(seg_keepalive_timer, 0);
        }

        // Check the connection password auth timer
        if (tcp_option->pw_connect_en == SEG_ENABLE) {
            if ((flag_auth_time == SEG_ENABLE) && (flag_connect_pw_auth == SEG_DISABLE)) {
                flag_auth_time = SEG_DISABLE;

#ifdef _SEG_DEBUG_
                PRT_SEG(" > CONNECTION PW: AUTH TIMEOUT\r\n");
#endif
                process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY, TRUE);
            }
        }
    }
}

void seg_mqtt_yield_task(void *argument)  {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    while (1) {
        if (network_connection->working_state == ST_CONNECT) {
            mqtt_transport_yield(&g_mqtt_config);
        }
        vTaskDelay(1);
    }
}
