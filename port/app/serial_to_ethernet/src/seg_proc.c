/* seg_proc.c - Network protocol state-machine handlers (proc_SEG_*) */

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

void proc_SEG_udp(uint8_t sock) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol();

    // Socket state
    uint8_t state = getSn_SR(sock);

    uint8_t flag = 0;

    switch (state) {
    case SOCK_UDP:
        break;

    case SOCK_CLOSED:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            // UART Ring buffer clear
            data_buffer_flush();
        }

        u2e_size = 0;
        e2u_size = 0;

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
        flag |= SOCK_IO_NONBLOCK;
        int8_t s = socket(sock, Sn_MR_UDP, network_connection->local_port, flag);

        if (s == sock) {
            set_device_status(ST_UDP);

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

void proc_SEG_tcp_client(uint8_t sock) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __serial_command *serial_command = (struct __serial_command *) & (get_DevConfig_pointer()->serial_command);

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol();

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
        if (check_tcp_connect_exception() == ON) {
            return;
        }
        // TCP connect
        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
        if (ret < 0) {
            PRT_SEG(" > SEG:TCP_CLIENT_MODE:ConnectNetwork Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            break;
        }
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
                data_buffer_flush();    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity = SEG_DISABLE;
                if (seg_inactivity_timer == NULL) {
                    seg_inactivity_timer = xTimerCreate("seg_inactivity_timer", pdMS_TO_TICKS(tcp_option->inactivity * 1000), pdFALSE, 0, inactivity_timer_callback);
                }
                xTimerStart(seg_inactivity_timer, 0);
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer == NULL) {
                    seg_keepalive_timer = xTimerCreate("seg_keepalive_timer", pdMS_TO_TICKS(tcp_option->keepalive_wait_time), pdFALSE, 0, keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer, 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer, pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }
            set_device_status(ST_CONNECT);
        }
        break;

    case SOCK_CLOSE_WAIT:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            while (getSn_RX_RSR(sock) || e2u_size) {
                ether_to_uart(sock); // receive remaining packets
            }
        }
        //process_socket_termination(sock, SOCK_TERMINATION_DELAY);
        disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        set_device_status(ST_OPEN);
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);

        u2e_size = 0;
        e2u_size = 0;

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
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
        }
        break;

    default:
        break;
    }
}

#ifdef __USE_S2E_OVER_TLS__
void proc_SEG_tcp_client_over_tls(uint8_t sock) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol();

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
        if (check_tcp_connect_exception() == ON) {
            return;
        }

        reg_val = 0;
        ctlsocket(sock, CS_SET_INTMASK, (void *)&reg_val);

        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
        if (ret < 0) {
            PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE:ConnectNetwork Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            break;
        }

        reg_val = SIK_CONNECTED & 0x00FF;
        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

        ret = wiz_tls_connect(&s2e_tlsContext,
                              (char *)network_connection->remote_ip,
                              (unsigned int)network_connection->remote_port);

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
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
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
                data_buffer_flush();    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity = SEG_DISABLE;
                if (seg_inactivity_timer == NULL) {
                    seg_inactivity_timer = xTimerCreate("seg_inactivity_timer", pdMS_TO_TICKS(tcp_option->inactivity * 1000), pdFALSE, 0, inactivity_timer_callback);
                }
                xTimerStart(seg_inactivity_timer, 0);
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer == NULL) {
                    seg_keepalive_timer = xTimerCreate("seg_keepalive_timer", pdMS_TO_TICKS(tcp_option->keepalive_wait_time), pdFALSE, 0, keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer, 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer, pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }
            first_established = 0;
            set_device_status(ST_CONNECT);
        }
        break;

    case SOCK_CLOSE_WAIT:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            while (getSn_RX_RSR(sock) || e2u_size) {
                ether_to_uart(sock);    // receive remaining packets
            }
        }
        //process_socket_termination(sock, SOCK_TERMINATION_DELAY); // including disconnect(sock) function
        disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
        set_device_status(ST_OPEN);
        if (wiz_tls_init(&s2e_tlsContext, (int *)sock) > 0) {
            u2e_size = 0;
            e2u_size = 0;

            if (network_connection->fixed_local_port) {
                source_port = network_connection->local_port;
            } else {
                source_port = get_tcp_any_port();
            }

            PRT_SEG(" > TCP CLIENT over TLS: client_any_port = %d\r\n", client_any_port);
            int s = wiz_tls_socket(&s2e_tlsContext, sock, source_port);
            if (s == sock) {
                // Replace the command mode switch code GAP time (default: 500ms)
                if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                    modeswitch_gap_time = serial_data_packing->packing_time;
                }

                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE:SOCKOPEN\r\n");
                }
                set_wiz_tls_init_state(ENABLE);
            } else {
                PRT_SEG("wiz_tls_socket() failed\r\n");
                wiz_tls_deinit(&s2e_tlsContext);
                set_wiz_tls_init_state(DISABLE);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            }
        } else {
            PRT_SEG("wiz_tls_init() failed\r\n");
            wiz_tls_deinit(&s2e_tlsContext);
            set_wiz_tls_init_state(DISABLE);
        }
        break;

    default:
        break;
    }
}

#endif

void proc_SEG_mqtt_client(uint8_t sock) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;
    struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option);

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    uint8_t serial_mode = get_serial_communation_protocol();
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
        if (check_tcp_connect_exception() == ON) {
            return;
        }

        reg_val = 0;
        ctlsocket(sock, CS_SET_INTMASK, (void *)&reg_val);

        // MQTT connect
        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
        if (ret < 0) {
            PRT_SEG(" > SEG:MQTT_CLIENT_MODE:ConnectNetwork Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            break;
        }
        PRT_SEG(" > SEG:MQTT_CLIENT_MODE:TCP_CONNECTION\r\n");

        reg_val = SIK_ALL & 0x00FF;
        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

        ret = mqtt_transport_connect(&g_mqtt_config, tcp_option->reconnection);
        if (ret < 0) {
            PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTConnect Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            break;
        }
        PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTT_CONNECTION\r\n");

        if (mqtt_option->sub_topic_0[0] != 0 && mqtt_option->sub_topic_0[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config, mqtt_option->qos, (char *)mqtt_option->sub_topic_0);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_1[0] != 0 && mqtt_option->sub_topic_1[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config, mqtt_option->qos, (char *)mqtt_option->sub_topic_1);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_2[0] != 0 && mqtt_option->sub_topic_2[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config, mqtt_option->qos, (char *)mqtt_option->sub_topic_2);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTT_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
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
                data_buffer_flush();    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity = SEG_DISABLE;
                if (seg_inactivity_timer == NULL) {
                    seg_inactivity_timer = xTimerCreate("seg_inactivity_timer", pdMS_TO_TICKS(tcp_option->inactivity * 1000), pdFALSE, 0, inactivity_timer_callback);
                }
                xTimerStart(seg_inactivity_timer, 0);
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer == NULL) {
                    seg_keepalive_timer = xTimerCreate("seg_keepalive_timer", pdMS_TO_TICKS(tcp_option->keepalive_wait_time), pdFALSE, 0, keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer, 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer, pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }
            first_established = 0;
            set_device_status(ST_CONNECT);
        }
        //mqtt_transport_yield(&g_mqtt_config);
        break;

    case SOCK_CLOSE_WAIT:
        disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
        set_device_status(ST_OPEN);

        u2e_size = 0;
        e2u_size = 0;

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
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            break;
        }

        ret = mqtt_transport_init(sock, &g_mqtt_config, true, 0, g_recv_mqtt_buf,
                                  DATA_BUF_SIZE, &g_transport_interface, &g_network_context,
                                  mqtt_option->client_id, mqtt_option->user_name, mqtt_option->password, mqtt_option->keepalive, mqtt_subscribeMessageHandler);
        if (ret < 0) {
            PRT_SEG(" > SEG:MQTT_CLIENT_MODE:INITIALIZE FAILED\r\n");
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
        }
        break;

    default:
        break;
    }
}

#ifdef __USE_S2E_OVER_TLS__
void proc_SEG_mqtts_client(uint8_t sock) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;
    struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option);

    uint16_t source_port;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    uint8_t serial_mode = get_serial_communation_protocol();
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
        if (check_tcp_connect_exception() == ON) {
            return;
        }

        reg_val = 0;
        ctlsocket(sock, CS_SET_INTMASK, (void *)&reg_val);

        ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
        if (ret < 0) {
            PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE:ConnectNetwork Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            break;
        }
        reg_val = SIK_ALL & 0x00FF;
        ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

        PRT_SEG(" > SEG:TCP_CLIENT_OVER_TLS_MODE: TCP CLIENT CONNECTED\r\n");

        ret = wiz_tls_connect(&s2e_tlsContext,
                              (char *)network_connection->remote_ip,
                              (unsigned int)network_connection->remote_port);

        if (ret != 0) { // TLS connection failed
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            if (serial_common->serial_debug_en) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE: CONNECTION FAILED\r\n");
            }
            break;
        }
        PRT_SEG(" > SEG:MQTTS_CLIENT_MODE: SSL CLIENT CONNECTED\r\n");

        // MQTTS connect
        ret = mqtt_transport_connect(&g_mqtt_config, tcp_option->reconnection);
        if (ret < 0) {
            PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:ConnectNetwork Err %d\r\n", ret);
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            break;
        }

        PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTT_CONNECTION\r\n");

        if (mqtt_option->sub_topic_0[0] != 0 && mqtt_option->sub_topic_0[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config, mqtt_option->qos, (char *)mqtt_option->sub_topic_0);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_1[0] != 0 && mqtt_option->sub_topic_1[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config, mqtt_option->qos, (char *)mqtt_option->sub_topic_1);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
                break;
            }
        }
        if (mqtt_option->sub_topic_2[0] != 0 && mqtt_option->sub_topic_2[0] != 0xFF) {
            ret = mqtt_transport_subscribe(&g_mqtt_config, mqtt_option->qos, (char *)mqtt_option->sub_topic_2);
            if (ret < 0) {
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:MQTTSubscribe Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
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
                data_buffer_flush();
            }

            if (tcp_option->inactivity) {
                flag_inactivity = SEG_DISABLE;
                if (seg_inactivity_timer == NULL) {
                    seg_inactivity_timer = xTimerCreate("seg_inactivity_timer", pdMS_TO_TICKS(tcp_option->inactivity * 1000), pdFALSE, 0, inactivity_timer_callback);
                }
                xTimerStart(seg_inactivity_timer, 0);
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer == NULL) {
                    seg_keepalive_timer = xTimerCreate("seg_keepalive_timer", pdMS_TO_TICKS(tcp_option->keepalive_wait_time), pdFALSE, 0, keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer, 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer, pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }
            first_established = 0;
            set_device_status(ST_CONNECT);
        }
        mqtt_transport_yield(&g_mqtt_config);
        break;

    case SOCK_CLOSE_WAIT:
        disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
        set_device_status(ST_OPEN);

        if (wiz_tls_init(&s2e_tlsContext, (int *)sock) > 0) {
            u2e_size = 0;
            e2u_size = 0;

            if (network_connection->fixed_local_port) {
                source_port = network_connection->local_port;
            } else {
                source_port = get_tcp_any_port();
            }

            PRT_SEG(" > MQTTS_CLIENT_MODE:client_any_port = %d\r\n", client_any_port);
            int s = wiz_tls_socket(&s2e_tlsContext, sock, source_port);

            if (s == sock) {
                // Replace the command mode switch code GAP time (default: 500ms)
                if ((serial_command->serial_command == SEG_ENABLE) && serial_data_packing->packing_time) {
                    modeswitch_gap_time = serial_data_packing->packing_time;
                }

                if (serial_common->serial_debug_en) {
                    PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:SOCKOPEN\r\n");
                }
                set_wiz_tls_init_state(ENABLE);
            } else {
                PRT_SEG("wiz_tls_socket() failed\r\n");
                wiz_tls_deinit(&s2e_tlsContext);
                set_wiz_tls_init_state(DISABLE);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            }
            ret = mqtt_transport_init(sock, &g_mqtt_config, true, 1, g_recv_mqtt_buf,
                                      DATA_BUF_SIZE, &g_transport_interface, &g_network_context,
                                      mqtt_option->client_id, mqtt_option->user_name, mqtt_option->password, mqtt_option->keepalive, mqtt_subscribeMessageHandler);
            if (ret < 0) {
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
                PRT_SEG(" > SEG:MQTTS_CLIENT_MODE:INITIALIZE FAILED\r\n");
            }
        } else {
            PRT_SEGCP("wiz_tls_init() failed\r\n");
            wiz_tls_deinit(&s2e_tlsContext);
            set_wiz_tls_init_state(DISABLE);
        }
        break;

    default:
        break;
    }
}
#endif // __USE_S2E_OVER_TLS__

void proc_SEG_tcp_server(uint8_t sock) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_command *serial_command = (struct __serial_command *) & (get_DevConfig_pointer()->serial_command);
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);

    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol();

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
            reg_val = SIK_CONNECTED & 0x00FF; // except SIK_SENT(send OK) interrupt
            ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            // Serial debug message printout
            if (serial_common->serial_debug_en) {
                getsockopt(sock, SO_DESTIP, &destip);
                getsockopt(sock, SO_DESTPORT, &destport);
                PRT_SEG(" > SEG:CONNECTED FROM - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
            }

            if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
                data_buffer_flush();    // UART Ring buffer clear
            }

            if (tcp_option->inactivity) {
                flag_inactivity = SEG_DISABLE;
                if (seg_inactivity_timer == NULL) {
                    seg_inactivity_timer = xTimerCreate("seg_inactivity_timer", pdMS_TO_TICKS(tcp_option->inactivity * 1000), pdFALSE, 0, inactivity_timer_callback);
                }
                xTimerStart(seg_inactivity_timer, 0);
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer == NULL) {
                    seg_keepalive_timer = xTimerCreate("seg_keepalive_timer", pdMS_TO_TICKS(tcp_option->keepalive_wait_time), pdFALSE, 0, keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer, 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer, pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }

            if (tcp_option->pw_connect_en) { // TCP server mode only (+ mixed_server)
                flag_auth_time = SEG_DISABLE;
                if (seg_auth_timer == NULL) {
                    seg_auth_timer = xTimerCreate("seg_auth_timer", pdMS_TO_TICKS(MAX_CONNECTION_AUTH_TIME), pdFALSE, 0, auth_timer_callback);
                }
                xTimerStart(seg_auth_timer, 0);
            }
            set_device_status(ST_CONNECT);
        }
        break;

    case SOCK_CLOSE_WAIT:
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            while (getSn_RX_RSR(sock) || e2u_size) {
                ether_to_uart(sock);    // receive remaining packets
            }
        }
        disconnect(sock);
        //process_socket_termination(sock, SOCK_TERMINATION_DELAY);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
        set_device_status(ST_OPEN);

        u2e_size = 0;
        e2u_size = 0;

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
            process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
        }
        break;

    default:
        break;
    }
}

void proc_SEG_tcp_mixed(uint8_t sock) {
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __network_option *network_option = (struct __network_option *) & (get_DevConfig_pointer()->network_option);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __serial_command *serial_command = (struct __serial_command *)&get_DevConfig_pointer()->serial_command;
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);

    uint16_t source_port = 0;
    uint8_t destip[4] = {0, };
    uint16_t destport = 0;

    // Serial communication mode
    uint8_t serial_mode = get_serial_communation_protocol();

    // Socket state
    uint8_t state = getSn_SR(sock);
    int ret;
    uint16_t reg_val;

#ifdef MIXED_CLIENT_LIMITED_CONNECT
    static uint8_t reconnection_count = 0;
#endif
    switch (state) {
    case SOCK_INIT:
        if (mixed_state == MIXED_CLIENT) {
            if (reconnection_count && tcp_option->reconnection) {
                vTaskDelay(tcp_option->reconnection);
            }

            // TCP connect exception checker; e.g., dns failed / zero srcip ... and etc.
            if (check_tcp_connect_exception() == ON) {
#ifdef MIXED_CLIENT_LIMITED_CONNECT
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
                reconnection_count = 0;
                data_buffer_flush();
                mixed_state = MIXED_SERVER;
#endif
                return;
            }

            // TCP connect
            ret = connect(sock, network_connection->remote_ip, network_connection->remote_port);
            if (ret < 0) {
                PRT_SEG(" > SEG:TCP_MIXED_MODE:ConnectNetwork Err %d\r\n", ret);
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            }

#ifdef MIXED_CLIENT_LIMITED_CONNECT
            reconnection_count++;

            if (reconnection_count >= network_option->tcp_rcr_val) {
                PRT_SEG("reconnection_count >= network_option->tcp_rcr_val\r\n");
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
                reconnection_count = 0;
                data_buffer_flush();
                mixed_state = MIXED_SERVER;
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

                if (mixed_state == MIXED_SERVER) {
                    PRT_SEG(" > SEG:CONNECTED FROM - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
                } else {
                    PRT_SEG(" > SEG:CONNECTED TO - %d.%d.%d.%d : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
                }
            }

            if (tcp_option->inactivity) {
                flag_inactivity = SEG_DISABLE;
                if (seg_inactivity_timer == NULL) {
                    seg_inactivity_timer = xTimerCreate("seg_inactivity_timer", pdMS_TO_TICKS(tcp_option->inactivity * 1000), pdFALSE, 0, inactivity_timer_callback);
                }
                xTimerStart(seg_inactivity_timer, 0);
            }

            if (tcp_option->keepalive_en) {
                if (seg_keepalive_timer == NULL) {
                    seg_keepalive_timer = xTimerCreate("seg_keepalive_timer", pdMS_TO_TICKS(tcp_option->keepalive_wait_time), pdFALSE, 0, keepalive_timer_callback);
                } else {
                    if (xTimerIsTimerActive(seg_keepalive_timer) == pdTRUE) {
                        xTimerStop(seg_keepalive_timer, 0);
                    }
                    xTimerChangePeriod(seg_keepalive_timer, pdMS_TO_TICKS(tcp_option->keepalive_wait_time), 0);
                }
            }

            set_device_status(ST_CONNECT);
            // Check the connection password auth timer
            if (mixed_state == MIXED_SERVER) {
                // Connection Password option: TCP server mode only (+ mixed_server)
                data_buffer_flush();
                flag_auth_time = SEG_DISABLE;
                if (tcp_option->pw_connect_en == SEG_ENABLE) {
                    if (seg_auth_timer == NULL) {
                        seg_auth_timer = xTimerCreate("seg_auth_timer", pdMS_TO_TICKS(MAX_CONNECTION_AUTH_TIME), pdTRUE, 0, auth_timer_callback);
                    }
                    xTimerStart(seg_auth_timer, 0);
                }
            } else {
                if (get_data_buffer_usedsize() || u2e_size) {
                    xSemaphoreGive(seg_u2e_sem);
                }
                mixed_state = MIXED_SERVER;
            }

#ifdef MIXED_CLIENT_LIMITED_CONNECT
            reconnection_count = 0;
#endif
        }
        break;

    case SOCK_CLOSE_WAIT:
        PRT_SEG("case SOCK_CLOSE_WAIT\r\n");
        if (serial_mode == SEG_SERIAL_PROTOCOL_NONE) {
            while (getSn_RX_RSR(sock) || e2u_size) {
                ether_to_uart(sock);    // receive remaining packets
            }
        }
        //process_socket_termination(sock, SOCK_TERMINATION_DELAY);
        disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        PRT_SEG("case SOCK_FIN_WAIT or SOCK_CLOSED\r\n");
        set_device_status(ST_OPEN);
        process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);

        if (mixed_state == MIXED_SERVER) { // MIXED_SERVER
            u2e_size = 0;
            e2u_size = 0;
            data_buffer_flush();

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
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            }
        } else { // MIXED_CLIENT
            PRT_INFO(" > SEG:TCP_MIXED_MODE:CLIENT_SOCKCLOSED\r\n");
            e2u_size = 0;
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
                process_socket_termination(sock, SOCK_TERMINATION_DELAY, FALSE);
            }
        }
        break;

    default:
        break;
    }
}

int wizchip_mqtt_publish(mqtt_config_t *mqtt_config, uint8_t *pub_topic, uint8_t qos, uint8_t *pub_data, uint32_t pub_data_len) {
    if (mqtt_transport_publish(mqtt_config, pub_topic, pub_data, pub_data_len, qos)) {
        return -1;
    }
    return pub_data_len;
}

void mqtt_subscribeMessageHandler(uint8_t *data, uint32_t data_len) {
    e2u_size = data_len;
    memcpy(g_recv_buf, data, data_len);

    if (get_uart_spi_if()) {
        ether_to_spi(SEG_DATA0_SOCK);
    } else {
        ether_to_uart(SEG_DATA0_SOCK);
    }
}
