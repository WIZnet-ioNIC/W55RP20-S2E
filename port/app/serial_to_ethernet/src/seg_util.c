/* seg_util.c - Utility functions: connection management, timers, mode switch */

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

#include "port_common.h"

#ifdef __USE_S2E_OVER_TLS__
#include "SSLInterface.h"
extern wiz_tls_context s2e_tlsContext;
#endif

#include "netHandler.h"

/* Private define ------------------------------------------------------------*/
#ifndef SERIAL_MODE
#define SERIAL_MODE SEG_SERIAL_PROTOCOL_NONE
#endif

/* Public & Private functions ------------------------------------------------*/

uint8_t process_socket_termination(uint8_t sock, uint32_t timeout, uint8_t mutex) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);

    int8_t ret;
    uint8_t sock_status = getSn_SR(sock);
    uint32_t tickStart = millis();

    timers_stop();
    if (!(network_connection->working_mode == TCP_MIXED_MODE && mixed_state == MIXED_CLIENT)) {
        reset_SEG_timeflags();
    }

#ifdef __USE_S2E_OVER_TLS__
    if (get_wiz_tls_init_state() == ENABLE) {
        wiz_tls_close_notify(&s2e_tlsContext);
        PRT_SEG("wiz_tls_deinit\r\n");
        wiz_tls_deinit(&s2e_tlsContext);
        set_wiz_tls_init_state(DISABLE);
    }
#endif

    if (sock_status == SOCK_CLOSED) {
        return sock;
    }
    if (mutex == TRUE) {
        xSemaphoreTake(seg_critical_sem, portMAX_DELAY);
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
        xSemaphoreGive(seg_critical_sem);
    }
    xSemaphoreGive(seg_sem);
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

uint8_t check_tcp_connect_exception(void) {
    struct __network_option *network_option = (struct __network_option *)&get_DevConfig_pointer()->network_option;
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);

    uint8_t srcip[4] = {0, };
    uint8_t ret = OFF;

    getSIPR(srcip);

    // DNS failed
    if ((network_connection->dns_use == SEG_ENABLE) && (flag_process_dns_success != ON)) {
        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:CONNECTION FAILED - DNS Failed flag_process_dns_success = %d\r\n", flag_process_dns_success);
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

void init_trigger_modeswitch(uint8_t mode) {
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);

    if (mode == DEVICE_AT_MODE) {
        opmode = DEVICE_AT_MODE;
        set_device_status(ST_ATMODE);

        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:AT Mode\r\n");
            platform_uart_puts((uint8_t *)"SEG:AT Mode\r\n", strlen("SEG:AT Mode\r\n"));
        }
    } else { // DEVICE_GW_MODE
        opmode = DEVICE_GW_MODE;
        set_device_status(ST_OPEN);

        if (network_connection->working_mode == TCP_MIXED_MODE) {
            mixed_state = MIXED_SERVER;
        }

        if (serial_common->serial_debug_en) {
            PRT_SEG(" > SEG:GW Mode\r\n");
            platform_uart_puts((uint8_t *)"SEG:GW Mode\r\n", strlen("SEG:GW Mode\r\n"));
        }
    }

    u2e_size = 0;
    data_buffer_flush();
    reset_SEG_timeflags();
    //xTimerReset(seg_inactivity_timer, 0);
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
        put_byte_to_data_buffer(ch_tmp[i]);
        ch_tmp[i] = 0x00;
    }

    enable_modeswitch_timer = SEG_DISABLE;
    triggercode_idx = 0;
}

uint8_t check_serial_store_permitted(uint8_t ch) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option);

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

uint8_t get_serial_communation_protocol(void) {
    return SERIAL_MODE;
}

void send_keepalive_packet_manual(uint8_t sock) {
    setsockopt(sock, SO_KEEPALIVESEND, 0);
}

void init_time_delimiter_timer(void) {
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);

    if (opmode == DEVICE_GW_MODE) {
        if (serial_data_packing->packing_time != 0) {
            if (enable_serial_input_timer == SEG_DISABLE) {
                enable_serial_input_timer = SEG_ENABLE;
            }
            serial_input_time = 0;
        }
    }
}

void reset_SEG_timeflags(void) {
    // Timer disable
    enable_serial_input_timer = SEG_DISABLE;

    // Flag clear
    flag_serial_input_time_elapse = SEG_DISABLE;
    flag_send_keepalive = SEG_DISABLE;
    flag_first_keepalive = SEG_DISABLE;
    flag_auth_time = SEG_DISABLE;
    flag_connect_pw_auth = SEG_DISABLE; // TCP_SERVER_MODE only (+ MIXED_SERVER)
    flag_inactivity = SEG_DISABLE;

    // Timer value clea
    serial_input_time = 0;
}
