#ifndef SEG_H_
#define SEG_H_

#include <stdint.h>
#include "common.h"
#include "WIZnet_board.h"
#include "port_common.h"
#include "mqtt_transport_interface.h"

#define _SEG_DEBUG_


///////////////////////////////////////////////////////////////////////////////////////////////////////

#define SEG_DATA0_UART      DATA0_UART_PORTNUM // Data UART selector
#define SEG_DEBUG_UART      DEBUG_UART_PORTNUM // Debug UART

#define SEG_DATA_BUF_SIZE   4096 // UART Ring buffer size

///////////////////////////////////////////////////////////////////////////////////////////////////////
#define SOCK_TERMINATION_DELAY 10 //ms

#define DEFAULT_MODESWITCH_INTER_GAP        500 // 500ms (0.5sec)

//#define MIXED_CLIENT_INFINITY_CONNECT
#ifndef MIXED_CLIENT_INFINITY_CONNECT
#define MIXED_CLIENT_LIMITED_CONNECT    //  TCP_MIXED_MODE: TCP CLIENT - limited count of connection retries
#define MAX_RECONNECTION_COUNT          10
#endif

#define MAX_CONNECTION_AUTH_TIME            5000 // 5000ms (5sec)

///////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE           2048
#endif

#define SEG_DISABLE                 0
#define SEG_ENABLE                  1

#define SEG_KILOBYTE                   (1024)
#define SEG_MEGABYTE                   (SEG_KILOBYTE * 1024)

/* Globals defined in seg.c */
extern uint8_t opmode;
extern uint8_t flag_process_dhcp_success;
extern uint8_t flag_process_dns_success;
extern char * str_working[];

extern int u2e_size;
extern int e2u_size;
extern uint8_t peerip[4];
extern uint8_t peerip_tmp[4];
extern uint16_t peerport;
extern uint8_t isXON;
extern uint8_t sw_modeswitch_at_mode_on;
extern uint8_t enable_modeswitch_timer;
extern volatile uint16_t modeswitch_time;
extern volatile uint16_t modeswitch_gap_time;
extern uint8_t mixed_state;
extern uint16_t client_any_port;
extern uint8_t enable_serial_input_timer;
extern volatile uint16_t serial_input_time;
extern uint8_t flag_serial_input_time_elapse;
extern uint8_t flag_connect_pw_auth;
extern uint8_t flag_auth_time;
extern uint8_t flag_send_keepalive;
extern uint8_t flag_first_keepalive;
extern uint8_t flag_inactivity;
extern uint8_t triggercode_idx;
extern uint8_t ch_tmp[3];

extern uint8_t g_send_buf[DATA_BUF_SIZE];
extern uint8_t g_recv_buf[DATA_BUF_SIZE];
extern uint8_t g_recv_mqtt_buf[DATA_BUF_SIZE];
extern NetworkContext_t g_network_context;
extern TransportInterface_t g_transport_interface;
extern mqtt_config_t g_mqtt_config;

/* FreeRTOS handles */
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

typedef enum {SEG_UART_RX, SEG_UART_TX, SEG_ETHER_RX, SEG_ETHER_TX, SEG_ALL} teDATADIR;
typedef enum {
    SEG_DEBUG_DISABLED = 0,
    SEG_DEBUG_ENABLED  = 1,
    SEG_DEBUG_S2E      = 2,
    SEG_DEBUG_E2S      = 3,
    SEG_DEBUG_ALL      = 4
} teDEBUGTYPE;

#define SEG_SERIAL_PROTOCOL_NONE 0
#define SEG_SERIAL_MODBUS_RTU    1
#define SEG_SERIAL_MODBUS_ASCII  2

/* Remote monitor option */
enum {
    SEG_REMOTE_MONITOR_NONE   = 0,
    SEG_REMOTE_MONITOR_S2E    = 1,
    SEG_REMOTE_MONITOR_E2S    = 2,
    SEG_REMOTE_MONITOR_ALL    = 3,
};

/* Auto message - The first data packet from device */
// 0: No massage (default)
// 1: Send device type(name) when TCP connected
// 2: Send device MAC address when TCP connected
// 3: Send device IP address when TCP connected
// 4: Send device ID(device name + MAC) when TCP connected
// 5: Send device alias when TCP connected
// 6: Send device group when TCP connected

enum {
    SEG_LINK_MSG_NONE       = 0,
    SEG_LINK_MSG_DEVNAME    = 1,
    SEG_LINK_MSG_MAC        = 2,
    SEG_LINK_MSG_IP         = 3,
    SEG_LINK_MSG_DEVID      = 4,
    SEG_LINK_MSG_DEVALIAS   = 5,
    SEG_LINK_MSG_DEVGROUP   = 6,
};


typedef void (*seg_proc_fn_t)(uint8_t sock);

/* Internal function prototypes shared across seg*.c files */
void uart_to_ether(uint8_t sock);
void ether_to_uart(uint8_t sock);
uint16_t get_serial_data(void);
void restore_serial_data(uint8_t idx);
uint8_t check_connect_pw_auth(uint8_t * buf, uint16_t len);
uint8_t check_tcp_connect_exception(void);
void reset_SEG_timeflags(void);
uint16_t get_tcp_any_port(void);

// Serial to Ethernet function handler; call by main loop
void do_seg(uint8_t sock);

// Timer for S2E core operations
void seg_timer_sec(void);
void seg_timer_msec(void);

void init_trigger_modeswitch(uint8_t mode);

void set_device_status(teDEVSTATUS status);

uint8_t get_device_status(void);
uint8_t get_serial_communation_protocol(void);

uint8_t process_socket_termination(uint8_t sock, uint32_t timeout, uint8_t mutex);

// Send Keep-alive packet manually (once)
void send_keepalive_packet_manual(uint8_t sock);

//These functions must be located in UART Rx IRQ Handler.
uint8_t check_serial_store_permitted(uint8_t ch);
uint8_t check_modeswitch_trigger(uint8_t ch);	        // Serial command mode switch trigger code (3-bytes) checker
void init_time_delimiter_timer(void); 		// Serial data packing option [Time]: Timer enable function for Time delimiter

// Send Auto-message function
void send_sid(uint8_t sock, uint8_t link_message);

// Serial debug messages for verifying data transfer
uint16_t debugSerial_dataTransfer(uint8_t * buf, uint16_t size, teDEBUGTYPE type);

// MQTT sub handler
void mqtt_subscribeMessageHandler(uint8_t *data, uint32_t data_len);

int wizchip_mqtt_publish(mqtt_config_t *mqtt_config, uint8_t *pub_topic, uint8_t qos, uint8_t *pub_data, uint32_t pub_data_len);

void seg_task(void *argument);
void seg_u2e_task(void *argument);
void seg_recv_task(void *argument);
void timers_stop(void);
void keepalive_timer_callback(TimerHandle_t xTimer);
void inactivity_timer_callback(TimerHandle_t xTimer);
void auth_timer_callback(TimerHandle_t xTimer);
void seg_timer_task(void *argument);

void ether_to_spi(uint8_t sock);
void seg_spi_data_transfer_task(void);

void seg_mqtt_yield_task(void *argument);

#endif /* SEG_H_ */

