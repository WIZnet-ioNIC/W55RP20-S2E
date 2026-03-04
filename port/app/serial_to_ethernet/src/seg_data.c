/* seg_data.c - UART <-> Ethernet data transfer functions */

#include <stdint.h>
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
#include "spiHandler.h"

#include "port_common.h"

#ifdef __USE_S2E_OVER_TLS__
#include "SSLInterface.h"
extern wiz_tls_context s2e_tlsContext;
#endif

#include "netHandler.h"

/* Public & Private functions ------------------------------------------------*/

void uart_to_ether(uint8_t sock) {
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __serial_common *serial_common = (struct __serial_common *)&get_DevConfig_pointer()->serial_common;
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
    struct __mqtt_option *mqtt_option = (struct __mqtt_option *) & (get_DevConfig_pointer()->mqtt_option);

    uint16_t len;
    int16_t sent_len = 0;

    // UART ring buffer -> user's buffer

    if (get_uart_spi_if()) {
        len = u2e_size;
    } else {
        len = get_serial_data();
    }

    if (len > 0) {
        serial_input_time = 0;
        enable_serial_input_timer = 0;
        flag_serial_input_time_elapse = SEG_DISABLE;

        if (seg_inactivity_timer != NULL) {
            xTimerReset(seg_inactivity_timer, 0);
        }

        if ((serial_common->serial_debug_en == SEG_DEBUG_S2E) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
            debugSerial_dataTransfer(g_send_buf, len, SEG_DEBUG_S2E);
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
                    sent_len = (int16_t)sendto(sock, g_send_buf, len, peerip, peerport);    // UDP 1:N mode
                }
            } else {
                sent_len = (int16_t)sendto(sock, g_send_buf, len, network_connection->remote_ip, network_connection->remote_port);    // UDP 1:1 mode
            }
        } else if (network_connection->working_state == ST_CONNECT) {
            if (network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE) {
                sent_len = wizchip_mqtt_publish(&g_mqtt_config, mqtt_option->pub_topic, mqtt_option->qos, g_send_buf, len);
            }
#ifdef __USE_S2E_OVER_TLS__
            else if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                sent_len = wiz_tls_write(&s2e_tlsContext, g_send_buf, len);
            }
#endif
            else {
                sent_len = (int16_t)send(sock, g_send_buf, len);
            }

            if (tcp_option->keepalive_en == ENABLE && flag_first_keepalive == DISABLE) {
                flag_first_keepalive = ENABLE;
                xTimerStart(seg_keepalive_timer, 0);
            }
        }
        if (sent_len > 0) {
            u2e_size -= sent_len;
        }

        if (get_uart_spi_if()) {
            spi_send_ack();
            irq_set_enabled(SPI0_IRQ, true);
        }
    }
}

uint16_t get_serial_data(void) {
    struct __serial_data_packing *serial_data_packing = (struct __serial_data_packing *) & (get_DevConfig_pointer()->serial_data_packing);

    uint16_t i;
    uint16_t len;

    len = get_data_buffer_usedsize();

    if ((len + u2e_size) >= DATA_BUF_SIZE) { // Avoiding u2e buffer (g_send_buf) overflow
        /* Checking Data packing option: character delimiter */
        if ((serial_data_packing->packing_delimiter[0] != 0x00) && (len == 1)) {
            g_send_buf[u2e_size] = (uint8_t)data_buffer_getc();
            if (serial_data_packing->packing_delimiter[0] == g_send_buf[u2e_size]) {
                return u2e_size;
            }
        }

        // serial data length value update for avoiding u2e buffer overflow
        len = DATA_BUF_SIZE - u2e_size;
    }

    if ((!serial_data_packing->packing_time) &&
            (!serial_data_packing->packing_size) &&
            (!serial_data_packing->packing_delimiter[0])) { // No Data Packing tiem / size / delimiters.
        // ## 20150427 bugfix: Incorrect serial data storing (UART ring buffer to g_send_buf)
        for (i = 0; i < len; i++) {
            g_send_buf[u2e_size++] = (uint8_t)data_buffer_getc();
        }

        return u2e_size;
    } else {
        /* Checking Data packing options */
        for (i = 0; i < len; i++) {
            g_send_buf[u2e_size++] = (uint8_t)data_buffer_getc();

            // Packing delimiter: character option
            if ((serial_data_packing->packing_delimiter[0] != 0x00) &&
                    (serial_data_packing->packing_delimiter[0] == g_send_buf[u2e_size - 1])) {
                return u2e_size;
            }

            // Packing delimiter: size option
            if ((serial_data_packing->packing_size != 0) && (serial_data_packing->packing_size == u2e_size)) {
                return u2e_size;
            }
        }
    }

    // Packing delimiter: time option
    if ((serial_data_packing->packing_time != 0) && (u2e_size != 0) && (flag_serial_input_time_elapse)) {
        if (get_data_buffer_usedsize() == 0) {
            flag_serial_input_time_elapse = SEG_DISABLE;    // ##
        }

        return u2e_size;
    }

    return 0;
}

void ether_to_uart(uint8_t sock) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);

    uint16_t len;
    uint16_t i;
    uint16_t reg_val;

    if (serial_option->flow_control == flow_rts_cts) {
#ifdef __USE_GPIO_HARDWARE_FLOWCONTROL__
        if (get_uart_cts_pin() != UART_CTS_LOW) {
            return;    // DATA0 (RS-232) only
        }
#else
        ; // check the CTS reg
#endif
    }

    do {
        // H/W Socket buffer -> User's buffer
        if (!(network_connection->working_mode == MQTT_CLIENT_MODE || network_connection->working_mode == MQTTS_CLIENT_MODE)) {
            len = getSn_RX_RSR(sock);
            if (len > DATA_BUF_SIZE) {
                len = DATA_BUF_SIZE;    // avoiding buffer overflow
            }

            if (len > 0) {
                if (seg_inactivity_timer != NULL) {
                    xTimerReset(seg_inactivity_timer, 0);
                }

                if (network_connection->working_mode == UDP_MODE) {
                    e2u_size = recvfrom(sock, g_recv_buf, len, peerip, &peerport);

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
                        e2u_size = wiz_tls_read(&s2e_tlsContext, g_recv_buf, len);
                    } else {
                        e2u_size = recv(sock, g_recv_buf, len);
                    }
                }
#else
                    e2u_size = recv(sock, g_recv_buf, len);
                }
#endif
                if (e2u_size < 0) {
                    e2u_size = 0;
                }
                reg_val = SIK_RECEIVED & 0x00FF;
                ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            } else {
                break;
            }

            if ((network_connection->working_mode == TCP_SERVER_MODE) ||  \
                    ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state == MIXED_SERVER))) {
                // Connection password authentication
                if ((tcp_option->pw_connect_en == SEG_ENABLE) && (flag_connect_pw_auth == SEG_DISABLE)) {
                    if (check_connect_pw_auth(g_recv_buf, len) == SEG_ENABLE) {
                        flag_connect_pw_auth = SEG_ENABLE;
                    } else {
                        flag_connect_pw_auth = SEG_DISABLE;
                    }
                    e2u_size = 0;

                    xTimerStop(seg_auth_timer, 0);
                    if (flag_connect_pw_auth == SEG_DISABLE) {
                        disconnect(sock);
                        return;
                    }
                }
            }
        }
        // Ethernet data transfer to DATA UART
        if (e2u_size != 0) {
            if (serial_option->dsr_en == SEG_ENABLE) // DTR / DSR handshake (flow control)
                if (get_flowcontrol_dsr_pin() == IO_HIGH) {
                    return;
                }
            //////////////////////////////////////////////////////////////////////
#ifdef __USE_UART_485_422__
            if ((serial_option->uart_interface == UART_IF_RS422) ||
                    (serial_option->uart_interface == UART_IF_RS485)) {
                if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                    debugSerial_dataTransfer(g_recv_buf, e2u_size, SEG_DEBUG_E2S);
                }

                uart_rs485_enable();
                for (i = 0; i < e2u_size; i++) {
                    platform_uart_putc(g_recv_buf[i]);
                }
                uart_rs485_disable();

                e2u_size = 0;
            }
            //////////////////////////////////////////////////////////////////////
            else if (serial_option->flow_control == flow_xon_xoff)
#else
            if (serial_option->flow_control == flow_xon_xoff)
#endif
            {
                if (isXON == SEG_ENABLE) {
                    if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                        debugSerial_dataTransfer(g_recv_buf, e2u_size, SEG_DEBUG_E2S);
                    }

                    for (i = 0; i < e2u_size; i++) {
                        platform_uart_putc(g_recv_buf[i]);
                    }
                    e2u_size = 0;
                }
            } else {
                if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                    debugSerial_dataTransfer(g_recv_buf, e2u_size, SEG_DEBUG_E2S);
                }

                for (i = 0; i < e2u_size; i++) {
                    platform_uart_putc(g_recv_buf[i]);
                }
                e2u_size = 0;
            }
        }
    } while (e2u_size);
}

void ether_to_spi(uint8_t sock) {
    struct __serial_option *serial_option = (struct __serial_option *) & (get_DevConfig_pointer()->serial_option);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);
    struct __network_connection *network_connection = (struct __network_connection *) & (get_DevConfig_pointer()->network_connection);
    struct __tcp_option *tcp_option = (struct __tcp_option *) & (get_DevConfig_pointer()->tcp_option);
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
                    e2u_size = recvfrom(sock, g_recv_buf, len, peerip, &peerport);

                    if (memcmp(peerip_tmp, peerip, 4) !=  0) {
                        memcpy(peerip_tmp, peerip, 4);
                        if (serial_common->serial_debug_en) {
                            printf(" > UDP Peer IP/Port: %d.%d.%d.%d : %d\r\n", peerip[0], peerip[1], peerip[2], peerip[3], peerport);
                        }
                    }
                } else if (network_connection->working_state == ST_CONNECT) {
#ifdef __USE_S2E_OVER_TLS__
                    if (network_connection->working_mode == SSL_TCP_CLIENT_MODE) {
                        e2u_size = wiz_tls_read(&s2e_tlsContext, g_recv_buf, len);
                    }
#endif
                    else {
                        e2u_size = recv(sock, g_recv_buf, len);
                    }
                }
                reg_val = SIK_RECEIVED & 0x00FF;
                ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&reg_val);

            } else {
                break;
            }

            if ((network_connection->working_mode == TCP_SERVER_MODE) ||  \
                    ((network_connection->working_mode == TCP_MIXED_MODE) && (mixed_state == MIXED_SERVER))) {
                // Connection password authentication
                if ((tcp_option->pw_connect_en == SEG_ENABLE) && (flag_connect_pw_auth == SEG_DISABLE)) {
                    if (check_connect_pw_auth(g_recv_buf, len) == SEG_ENABLE) {
                        flag_connect_pw_auth = SEG_ENABLE;
                    } else {
                        flag_connect_pw_auth = SEG_DISABLE;
                    }
                    e2u_size = 0;

                    xTimerStop(seg_auth_timer, 0);
                    if (flag_connect_pw_auth == SEG_DISABLE) {
                        disconnect(sock);
                        return;
                    }
                }
            }
        }
        // Ethernet data transfer to DATA UART
        if (e2u_size != 0) {
            if ((serial_common->serial_debug_en == SEG_DEBUG_E2S) || (serial_common->serial_debug_en == SEG_DEBUG_ALL)) {
                debugSerial_dataTransfer(g_recv_buf, e2u_size, SEG_DEBUG_E2S);
            }
            uint32_t period_ms = 5 + (e2u_size * 5) / 1024;
            if (period_ms > 10) {
                period_ms = 10;
            }
            uint8_t header[4];

            header[0] = SPI_SLAVE_WRITE_LEN_CMD;
            memcpy(&header[1], &e2u_size, 2);
            header[3] = SPI_DUMMY;
            memcpy(get_data_buffer_ptr(), header, 4);
            memcpy(get_data_buffer_ptr() + 4, g_recv_buf, e2u_size);
            xTimerChangePeriod(spi_reset_timer, pdMS_TO_TICKS(period_ms), 0);
            xTimerStart(spi_reset_timer, 0);
            GPIO_Output_Reset(DATA0_SPI_INT_PIN);
        }
    } while (0);
}

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
