#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

#include "port_common.h"
#include "common.h"
#include "WIZnet_board.h"

#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"
#include "ConfigData.h"
#include "storageHandler.h"
#include "deviceHandler.h"
#include "SSLInterface.h"

#include "seg.h"
#include "segcp.h"
#include "util.h"
#include "bufferHandler.h"
#include "uartHandler.h"
#include "gpioHandler.h"
#include "timerHandler.h"
#include "netHandler.h"


/* Private define ------------------------------------------------------------*/

#define END_CERT "-----END CERTIFICATE-----"
#define END_PKEY "-----END RSA PRIVATE KEY-----"

/* Private functions ---------------------------------------------------------*/
uint16_t uart_get_commandline(uint8_t* buf, uint16_t maxSize);

/*
    The ioLibrary protects individual SPI transfers, but socket.c also updates
    process-wide software state (sock_is_sending and sock_io_mode).  SEGCP runs
    in its own UDP/TCP tasks, so every complete ioLibrary operation must share
    the same application-level mutex as the four data sockets.

    Keep these wrappers local to SEGCP.  The lock is deliberately not held while
    parsing a command or waiting for more input; only one ioLibrary call is
    serialized at a time.
*/
static uint8_t segcp_socket_status(uint8_t sock) {
    uint8_t status;

    seg_wizchip_api_lock();
    status = getSn_SR(sock);
    seg_wizchip_api_unlock();

    return status;
}

static uint8_t segcp_socket_interrupt(uint8_t sock) {
    uint8_t interrupt;

    seg_wizchip_api_lock();
    interrupt = getSn_IR(sock);
    seg_wizchip_api_unlock();

    return interrupt;
}

static uint16_t segcp_socket_rx_available(uint8_t sock) {
    uint16_t available;

    seg_wizchip_api_lock();
    available = getSn_RX_RSR(sock);
    seg_wizchip_api_unlock();

    return available;
}

static int32_t segcp_socket_recv(uint8_t sock, uint8_t *buf, uint16_t len) {
    int32_t received;

    seg_wizchip_api_lock();
    received = recv(sock, buf, len);
    seg_wizchip_api_unlock();

    return received;
}

static int32_t segcp_socket_recvfrom(uint8_t sock, uint8_t *buf, uint16_t len,
                                     uint8_t *addr, uint16_t *port) {
    int32_t received;

    seg_wizchip_api_lock();
    received = recvfrom(sock, buf, len, addr, port);
    seg_wizchip_api_unlock();

    return received;
}

static int32_t segcp_socket_send(uint8_t sock, uint8_t *buf, uint16_t len) {
    int32_t sent;

    seg_wizchip_api_lock();
    sent = send(sock, buf, len);
    seg_wizchip_api_unlock();

    return sent;
}

static int32_t segcp_socket_sendto(uint8_t sock, uint8_t *buf, uint16_t len,
                                   uint8_t *addr, uint16_t port) {
    return seg_wizchip_udp_send_nonblocking(sock, buf, len, addr, port);
}

static int8_t segcp_socket_open(uint8_t sock, uint8_t protocol,
                                uint16_t port, uint8_t flag) {
    int8_t result;

    if (protocol == Sn_MR_UDP) {
        seg_wizchip_udp_send_reset(sock);
    }
    seg_wizchip_api_lock();
    result = socket(sock, protocol, port, flag);
    seg_wizchip_api_unlock();

    return result;
}

static int8_t segcp_socket_listen(uint8_t sock) {
    int8_t result;

    seg_wizchip_api_lock();
    result = listen(sock);
    seg_wizchip_api_unlock();

    return result;
}

static int8_t segcp_socket_disconnect(uint8_t sock) {
    int8_t result;

    seg_wizchip_api_lock();
    result = disconnect(sock);
    seg_wizchip_api_unlock();

    return result;
}

static int8_t segcp_socket_close(uint8_t sock) {
    int8_t result;

    seg_wizchip_api_lock();
    result = close(sock);
    seg_wizchip_api_unlock();
    seg_wizchip_udp_send_reset(sock);

    return result;
}

// A reply that cannot be handed to the chip is simply lost: the request has
// already been taken out of the receive buffer, so the tool's retry arrives as
// a fresh one and meets the same refusal.  Several ways for that to become
// permanent share this one symptom - transmit free space that never returns
// because a send was booked and never completed, or a command register that
// never clears - and none of them leave the socket in a state the status switch
// would reopen, so the device answers no search again until it is power cycled
// while its data channels carry on.
//
// Rather than tell those causes apart, notice that replies have stopped leaving
// and reopen the socket, which clears all of them.  Only a run of failures
// counts: a single refusal is ordinary back-pressure.
#define SEGCP_UDP_REPLY_STUCK_MS 10000U

static uint32_t segcp_udp_reply_refused_since_ms;

static void segcp_udp_reply_sent(int32_t result) {
    uint32_t now = (uint32_t)millis();

    if (result > 0) {
        segcp_udp_reply_refused_since_ms = 0;
        return;
    }

    if (segcp_udp_reply_refused_since_ms == 0) {
        // Never zero, so the next refusal can tell "first" from "still".
        segcp_udp_reply_refused_since_ms = (now != 0) ? now : 1;
        return;
    }

    if ((now - segcp_udp_reply_refused_since_ms) >= SEGCP_UDP_REPLY_STUCK_MS) {
        segcp_udp_reply_refused_since_ms = 0;
        segcp_socket_close(SEGCP_UDP_SOCK);
    }
}

static int8_t segcp_socket_clear_interrupt(uint8_t sock, uint16_t interrupt) {
    int8_t result;

    seg_wizchip_api_lock();
    result = ctlsocket(sock, CS_CLR_INTERRUPT, (void *)&interrupt);
    seg_wizchip_api_unlock();

    return result;
}

/* Private variables ---------------------------------------------------------*/
uint8_t gSEGCPREQ[CONFIG_BUF_SIZE];
uint8_t gSEGCPREP[CONFIG_BUF_SIZE];
uint8_t tpar[SEGCP_PARAM_MAX * 2];  //for parsing config data

static uint8_t SEGCP_UART = SEG_DATA0_UART; // default: SEG_DATA0_UART

uint8_t * strDEVSTATUS[]  = {"BOOT", "OPEN", "CONNECT", "UPGRADE", "ATMODE", "UDP", 0};

// [K!]: Hidden command, Erase the MAC address and configuration data
#if (DEVICE_UART_CNT > 1)
uint8_t * tbSEGCPCMD[] = {"MC", "VR", "MN", "IM", "OP", "CP", "DG", "KA", "KI", "KE",
                          "RI", "LI", "SM", "GW", "DS", "DH", "LP", "RP", "RH", "BR",
                          "DB", "PR", "SB", "FL", "PO", "IT", "PT", "PS", "PD", "TE",
                          "SS", "NP", "SP", "MA", "PW", "SV", "EX", "RT", "UN", "ST",
                          "FR", "EC", "GA", "GB", "GC", "GD", "CA", "CB", "CC", "CD",
                          "SC", "S0", "S1", "RX", "UI", "TR", "QU", "QP", "QC", "QK",
                          "PU", "U0", "U1", "U2", "QO", "RC", "CE", "OC", "LC", "PK",
                          "UF", "FW", "SO", "SD", "DD", "QS", "EN", "EI", "AO", "QL",
                          "QH", "AP", "EB", "ED", "EP", "ES", "EF", "ND", "NS", "AT",
                          "RV", "RR", "RA", "RS", "RE", "RO", "EO", "RD", "RF", "SE",
                          "EE",
#if (DEVICE_UART_CNT > 2)
                          "GS", "WN", "WI", "TO", "GL", "GH", "TP", "WB", "WD", "WP", "WS", "WF", "HD", "HS",
                          "TT", "XV", "XR", "XA", "XS", "XE", "XO", "WO", "XD", "XF", "WE", // ch2 (25)
#endif
#if (DEVICE_UART_CNT > 3)
                          "CS", "YN", "YI", "JO", "CL", "CH", "JP", "YB", "YD", "YP", "YS", "YF", "UD", "US",
                          "JT", "ZV", "ZR", "ZA", "ZS", "ZE", "ZO", "YO", "ZD", "ZF", "YE", // ch3 (25)
#endif
                          0
                         };

#if 0
uint8_t * tbSEGCPCMD[] = {"MC", "VR", "MN", "IM", "OP", "DD", "CP", "PO", "DG", "KA",
                          "KI", "KE", "RI", "LI", "SM", "GW", "DS", "PI", "PP", "DX",
                          "DP", "DI", "DW", "DH", "LP", "RP", "RH", "BR", "DB", "PR",
                          "SB", "FL", "IT", "PT", "PS", "PD", "TE", "SS", "NP", "SP",
                          "LG", "ER", "FW", "MA", "PW", "SV", "EX", "RT", "UN", "ST",
                          "FR", "EC", "K!", "UE", "GA", "GB", "GC", "GD", "CA", "CB",
                          "CC", "CD", "SC", "S0", "S1", "RX", "FS", "FC", "FP", "FD",
                          "FH", "UI", "QS", "QO", "QH", "QP", "QL", "RV", "RA", "RS",
                          "RE", "RR", "EN", "EI", "EB", "ED", "EP", "ES", "EF", "E0",
                          "E1", "NT", "NS", "ND", "CR", "NR", "AB", "TR", "BU", "LF",
                          "AE", "AP", "MB", "SE", "CE", "CT", "N0", "N1", "N2", "AL",
                          "GR", "AM", "QF", "MM", "CS", "CM", "C0", "C1", "C2", "C3",
                          0
                         };
#endif
#else

uint8_t * tbSEGCPCMD[] = {"MC", "VR", "MN", "IM", "OP", "CP", "DG", "KA", "KI", "KE",
                          "RI", "LI", "SM", "GW", "DS", "DH", "LP", "RP", "RH", "BR",
                          "DB", "PR", "SB", "FL", "PO", "IT", "PT", "PS", "PD", "TE",
                          "SS", "NP", "SP", "MA", "PW", "SV", "EX", "RT", "UN", "ST",
                          "FR", "EC", "GA", "GB", "GC", "GD", "CA", "CB", "CC", "CD",
                          "SC", "S0", "S1", "RX", "UI", "TR", "QU", "QP", "QC", "QK",
                          "PU", "U0", "U1", "U2", "QO", "RC", "CE", "OC", "LC", "PK",
                          "UF", "FW", "SO", 0
                         };

#endif
uint8_t * tbSEGCPERR[] = {"ERNULL", "ERNOTAVAIL", "ERNOPARAM", "ERIGNORED", "ERNOCOMMAND", "ERINVALIDPARAM", "ERNOPRIVILEGE"};

// Keep-alive timer values for TCP unicast search function
uint8_t enable_configtool_keepalive_timer = SEGCP_DISABLE;
volatile uint16_t configtool_keepalive_time = 0;
uint8_t flag_send_configtool_keepalive = SEGCP_DISABLE;

extern uint8_t sw_modeswitch_at_mode_on;
extern uint8_t flag_process_dhcp_success;

extern xSemaphoreHandle net_segcp_udp_sem;
extern xSemaphoreHandle net_segcp_tcp_sem;
extern xSemaphoreHandle segcp_uart_sem;

#ifdef __USE_S2E_OVER_TLS__
extern wiz_tls_context s2e_tlsContext[DEVICE_UART_CNT];
#endif


void do_segcp_udp(void) {
    uint16_t segcp_ret = 0;

    segcp_ret = proc_SEGCP_udp(gSEGCPREQ, gSEGCPREP);
    segcp_ret_handler(segcp_ret);
}

void do_segcp_tcp(void) {
    uint16_t segcp_ret = 0;

    segcp_ret |= proc_SEGCP_tcp(gSEGCPREQ, gSEGCPREP);
    segcp_ret_handler(segcp_ret);
}

void do_segcp_serial(void) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint16_t segcp_ret = 0;

    // Process the serial AT command mode

    segcp_ret = proc_SEGCP_serial(gSEGCPREQ, gSEGCPREP);
    if (segcp_ret & SEGCP_RET_ERR)
        if (dev_config->serial_common.serial_debug_en) {
            PRT_ERR(" > SEGCP:ERROR:%04X\r\n", segcp_ret);
        }
    segcp_ret_handler(segcp_ret);
}

void segcp_ret_handler(uint16_t segcp_ret) {
    DevConfig *dev_config = get_DevConfig_pointer();

    uint8_t ret = 0;

    if (segcp_ret && ((segcp_ret & SEGCP_RET_ERR) != SEGCP_RET_ERR)) { // Command parsing success
        if (segcp_ret & SEGCP_RET_SWITCH) {
            if (opmode == DEVICE_GW_MODE) {
                init_trigger_modeswitch(DEVICE_AT_MODE);    // DEVICE_GW_MODE -> DEVICE_AT_MODE
            } else {
                init_trigger_modeswitch(DEVICE_GW_MODE);    // DEVICE_AT_MODE -> DEVICE_GW_MODE
            }
        }

        if (segcp_ret & SEGCP_RET_FACTORY) {
            device_set_factory_default();
        } else if (segcp_ret & SEGCP_RET_SAVE) {
            PRT_SEGCP("segcp_ret & SEGCP_RET_SAVE\r\n");
            save_DevConfig_to_storage();
        } else if (segcp_ret & SEGCP_RET_ERASE_EEPROM) {
            PRT_SEGCP("segcp_ret & SEGCP_RET_ERASE_EEPROM\r\n");
            erase_storage(STORAGE_MAC);
            erase_storage(STORAGE_CONFIG);
        }
        if (segcp_ret & SEGCP_RET_FWUP) {
            teDEVSTATUS status_bak[DEVICE_UART_CNT];

            for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
                status_bak[ch] = (teDEVSTATUS)get_device_status(ch);
                set_device_status(ST_UPGRADE, ch);
            }

            if ((segcp_ret & SEGCP_RET_FWUP_BANK) == segcp_ret) {
                ret = device_bank_update(); // BANK Firmware update by Configuration tool
            } else {
                ret = DEVICE_FWUP_RET_FAILED;
            }

            if (ret == DEVICE_FWUP_RET_SUCCESS) {
                for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
                    set_device_status(ST_OPEN, ch);
                }

                save_DevConfig_to_storage();

                device_reboot();
            } else {
                // Clear the firmware update flags and size
                dev_config->firmware_update.fwup_size = 0;
                dev_config->firmware_update.fwup_flag = SEGCP_DISABLE;
                dev_config->firmware_update.fwup_server_flag = SEGCP_DISABLE;
                for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
                    set_device_status(status_bak[ch], ch);
                }
                seg_wizchip_api_lock();
                close(SOCK_FWUPDATE);
                seg_wizchip_api_unlock();

                if (dev_config->serial_common.serial_debug_en) {
                    printf(" > SEGCP:UPDATE:FAILED\r\n");
                }
            }
        }

        if (segcp_ret & SEGCP_RET_REBOOT) {
            PRT_SEGCP("segcp_ret & SEGCP_RET_REBOOT\r\n");
            if (opmode == DEVICE_AT_MODE)
                if (dev_config->serial_common.serial_debug_en) {
                    platform_uart_puts((uint8_t *)"REBOOT\r\n", 8, SEG_DATA0_CH);
                }
            device_reboot();
        }
    }
}

void set_segcp_uart(uint8_t uartNum) {
    SEGCP_UART = uartNum;
}


uint8_t get_segcp_uart(void) {
    return SEGCP_UART;
}


// Copy a command parameter into a fixed configuration field. The request buffer
// is far larger than any of these fields, so a parameter that does not fit is
// refused rather than truncated: a silently shortened password or topic is worse
// than a rejected command, and the write would land in the neighbouring field.
static void segcp_store_string(void *field, uint32_t size, const uint8_t *param,
                               uint16_t *ret) {
    uint32_t len = strlen((const char *)param);

    if (len > (size - 1)) {
        *ret |= SEGCP_RET_ERR_INVALIDPARAM;
        return;
    }
    memcpy(field, param, len);
    ((uint8_t *)field)[len] = 0;
}

uint8_t parse_SEGCP(uint8_t * pmsg, uint8_t * param) {
    uint8_t** pcmd;
    uint8_t cmdnum = 0;
    uint8_t i;
    uint32_t len;

    *param = 0;

    for (pcmd = tbSEGCPCMD; *pcmd != 0; pcmd++) {
        if (!strncmp((char *)pmsg, *pcmd, strlen(*pcmd))) {
            break;
        }
    }

    if (*pcmd == 0) {
        return SEGCP_UNKNOWN;
    }

    cmdnum = (uint8_t)(pcmd - tbSEGCPCMD);

    if (cmdnum == (uint8_t)SEGCP_MA) {
        if ((pmsg[8] == '\r') && (pmsg[9] == '\n')) {
            memcpy(param, (uint8_t*)&pmsg[2], 6);
        } else {
            return SEGCP_UNKNOWN;
        }
    } else if (cmdnum == (uint8_t)SEGCP_PW) {
        // The index is a byte and the destination is param[SEGCP_PARAM_MAX * 2],
        // so stop before either can wrap. A token that reaches the bound without
        // its delimiter is malformed and rejected below.
        for (i = 0; (i < (SEGCP_PARAM_MAX - 1)) && (pmsg[2 + i] != '\r'); i++) {
            param[i] = pmsg[2 + i];
        }

        if ((pmsg[2 + i] == '\r') && (pmsg[2 + i + 1] == '\n')) {
            param[i] = 0; param[i + 1] = 0;
        } else {
            return SEGCP_UNKNOWN;
        }
    }

    else if ((cmdnum == (uint8_t)SEGCP_OC) || (cmdnum == (uint8_t)SEGCP_LC) || \
             (cmdnum == (uint8_t)SEGCP_PK)) { //|| (cmdnum == (uint8_t)SEGCP_UP))
        len = strlen(pmsg);

        if (*(pmsg + len) == NULL) {
            if ((*(pmsg + len + 1) == '\n') || (*(pmsg + len + 1) == NULL)) {
                *(pmsg + len) = '\r';
                *(pmsg + len + 1) = '\n';
            } else {
                *(pmsg + len) = '\n';
            }
        }
        *param = 1;
        return cmdnum;
    } else {
        // A request token can be almost as long as the whole request buffer,
        // which is several times the size of param.
        if (strlen((const char *)&pmsg[2]) > ((SEGCP_PARAM_MAX * 2) - 1)) {
            return SEGCP_UNKNOWN;
        }
        strcpy(param, (uint8_t*)&pmsg[2]);
    }
    return cmdnum;
}

uint16_t proc_SEGCP(uint8_t* segcp_req, uint8_t* segcp_rep, uint8_t segcp_privilege) {
    DevConfig *dev_config = get_DevConfig_pointer();

    //uint8_t  i = 0;
    uint16_t ret = 0;
    int ret_2;
    uint8_t  cmdnum = 0;
    uint8_t* treq;

    char * trep = segcp_rep;
    uint16_t param_len = 0;

#ifdef __USE_USERS_GPIO__
    uint8_t  io_num = 0;
    uint8_t  io_type = 0;
    uint8_t  io_dir = 0;
#endif

    uint8_t  tmp_byte = 0;
    uint16_t tmp_int = 0;
    uint32_t tmp_long = 0;

    uint16_t tmp_port;
    uint8_t tmp_ip[4];
    uint8_t param[SEGCP_PARAM_MAX * 2];

    uint32_t len;
    uint8_t *tmp_ptr;
    uint8_t *temp_buf;

    //PRT_SEGCP("SEGCP_REQ : %s\r\n",segcp_req);
    // trep is a pointer, so sizeof() cleared four bytes instead of the buffer.
    memset(segcp_rep, 0, CONFIG_BUF_SIZE);
    treq = strtok(segcp_req, SEGCP_DELIMETER);

    while (treq) {
        // One request can carry many short reads whose answers are far longer
        // than the commands, so stop before the reply runs past its buffer.
        if ((uint32_t)(trep - (char *)segcp_rep) > (CONFIG_BUF_SIZE - SEGCP_REPLY_HEADROOM)) {
            ret |= SEGCP_RET_ERR_IGNORED;
            break;
        }
        //PRT_SEGCP("SEGCP_REQ_TOK : %s\r\n",treq);
        if ((cmdnum = parse_SEGCP(treq, param)) != SEGCP_UNKNOWN) {
            param_len = strlen((const char *)param);

            if (*param == 0) {
                memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                trep += SEGCP_CMD_MAX;

                switch ((teSEGCPCMDNUM)cmdnum) {
                case SEGCP_MC: sprintf(trep, "%02X:%02X:%02X:%02X:%02X:%02X",
                                           dev_config->network_common.mac[0], dev_config->network_common.mac[1], dev_config->network_common.mac[2],
                                           dev_config->network_common.mac[3], dev_config->network_common.mac[4], dev_config->network_common.mac[5]);
                    break;
                case SEGCP_VR:
                    if (strcmp(STR_VERSION_STATUS, "Stable") == 0) {
                        sprintf(trep, "%d.%d.%d", dev_config->device_common.fw_ver[0],
                                dev_config->device_common.fw_ver[1],
                                dev_config->device_common.fw_ver[2]); // Standard stable version
                    } else if (strcmp(STR_VERSION_STATUS, "Develop") == 0) {
                        // Develop version
                        sprintf(trep, "%d.%d.%ddev", dev_config->device_common.fw_ver[0],
                                dev_config->device_common.fw_ver[1],
                                dev_config->device_common.fw_ver[2]);
                    } else {
                        // Custom version
                        sprintf(trep, "%d.%d.%d%s", dev_config->device_common.fw_ver[0],
                                dev_config->device_common.fw_ver[1],
                                dev_config->device_common.fw_ver[2], STR_VERSION_STATUS);
                    }
                    break;
                case SEGCP_MN: sprintf(trep, "%s", dev_config->device_common.device_name);
                    break;
                case SEGCP_IM: sprintf(trep, "%d", dev_config->network_option.dhcp_use);	// 0:STATIC, 1:DHCP (PPPoE X)
                    break;
                case SEGCP_OP: sprintf(trep, "%d", dev_config->network_connection[0].working_mode); // opmode
                    break;
                case SEGCP_AO: sprintf(trep, "%d", dev_config->network_connection[1].working_mode); // opmode
                    break;
                case SEGCP_CP: sprintf(trep, "%d", dev_config->tcp_option[0].pw_connect_en);
                    break;
                case SEGCP_DG: sprintf(trep, "%d", dev_config->serial_common.serial_debug_en);
                    break;
                case SEGCP_KA: sprintf(trep, "%d", dev_config->tcp_option[0].keepalive_en);
                    break;
                case SEGCP_RA: sprintf(trep, "%d", dev_config->tcp_option[1].keepalive_en);
                    break;
                case SEGCP_KI: sprintf(trep, "%d", dev_config->tcp_option[0].keepalive_wait_time);
                    break;
                case SEGCP_RS: sprintf(trep, "%d", dev_config->tcp_option[1].keepalive_wait_time);
                    break;
                case SEGCP_KE: sprintf(trep, "%d", dev_config->tcp_option[0].keepalive_retry_time);
                    break;
                case SEGCP_RE: sprintf(trep, "%d", dev_config->tcp_option[1].keepalive_retry_time);
                    break;
                case SEGCP_RI: sprintf(trep, "%d", dev_config->tcp_option[0].reconnection);
                    break;
                case SEGCP_RR: sprintf(trep, "%d", dev_config->tcp_option[1].reconnection);
                    break;
                case SEGCP_LI:
                    if (dev_config->network_option.dhcp_use && !flag_process_dhcp_success) {  //if dhcp doesn't be finished, send all 0
                        sprintf(trep, "0.0.0.0");
                    } else {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_common.local_ip[0], dev_config->network_common.local_ip[1],
                                dev_config->network_common.local_ip[2], dev_config->network_common.local_ip[3]);
                    }
                    break;
                case SEGCP_SM:
                    if (dev_config->network_option.dhcp_use && !flag_process_dhcp_success) {  //if dhcp doesn't be finished, send all 0
                        sprintf(trep, "0.0.0.0");
                    } else {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_common.subnet[0], dev_config->network_common.subnet[1],
                                dev_config->network_common.subnet[2], dev_config->network_common.subnet[3]);
                    }
                    break;
                case SEGCP_GW:
                    if (dev_config->network_option.dhcp_use && !flag_process_dhcp_success) {  //if dhcp doesn't be finished, send all 0
                        sprintf(trep, "0.0.0.0");
                    } else {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_common.gateway[0], dev_config->network_common.gateway[1],
                                dev_config->network_common.gateway[2], dev_config->network_common.gateway[3]);
                    }
                    break;
                case SEGCP_DS:
                    if (dev_config->network_option.dhcp_use && !flag_process_dhcp_success) {  //if dhcp doesn't be finished, send all 0
                        sprintf(trep, "0.0.0.0");
                    } else {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_option.dns_server_ip[0], dev_config->network_option.dns_server_ip[1],
                                dev_config->network_option.dns_server_ip[2], dev_config->network_option.dns_server_ip[3]);
                    }
                    break;

                case SEGCP_DH:
                    if (dev_config->device_common.device_name[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else
                        //sprintf(trep, "%s-%02X%02X%02X", dev_config->module_name, dev_config->network_info_common.mac[3], dev_config->network_info_common.mac[4], dev_config->network_info_common.mac[5]);
                        sprintf(trep, "%s-%02X%02X%02X%02X%02X%02X", dev_config->device_common.device_name,
                                dev_config->network_common.mac[0],
                                dev_config->network_common.mac[1],
                                dev_config->network_common.mac[2],
                                dev_config->network_common.mac[3],
                                dev_config->network_common.mac[4],
                                dev_config->network_common.mac[5]);
                    break;
                case SEGCP_LP: sprintf(trep, "%d", dev_config->network_connection[0].local_port);
                    break;
                case SEGCP_QL: sprintf(trep, "%d", dev_config->network_connection[1].local_port);
                    break;
                case SEGCP_RP: sprintf(trep, "%d", dev_config->network_connection[0].remote_port);
                    break;
                case SEGCP_AP: sprintf(trep, "%d", dev_config->network_connection[1].remote_port);
                    break;
                case SEGCP_RH:
                    if (dev_config->network_connection[0].dns_use == SEGCP_DISABLE) {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_connection[0].remote_ip[0],
                                dev_config->network_connection[0].remote_ip[1],
                                dev_config->network_connection[0].remote_ip[2],
                                dev_config->network_connection[0].remote_ip[3]);
                    } else {
                        if (dev_config->network_connection[0].dns_domain_name[0] == 0) {
                            sprintf(trep, "%c", SEGCP_NULL);
                        } else {
                            sprintf(trep, "%s", dev_config->network_connection[0].dns_domain_name);
                        }
                    }
                    break;
                case SEGCP_QH:
                    if (dev_config->network_connection[1].dns_use == SEGCP_DISABLE) {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_connection[1].remote_ip[0],
                                dev_config->network_connection[1].remote_ip[1],
                                dev_config->network_connection[1].remote_ip[2],
                                dev_config->network_connection[1].remote_ip[3]);
                    } else {
                        if (dev_config->network_connection[1].dns_domain_name[0] == 0) {
                            sprintf(trep, "%c", SEGCP_NULL);
                        } else {
                            sprintf(trep, "%s", dev_config->network_connection[1].dns_domain_name);
                        }
                    }
                    break;
                case SEGCP_BR: sprintf(trep, "%d", dev_config->serial_option[0].baud_rate);
                    break;
                case SEGCP_EB: sprintf(trep, "%d", dev_config->serial_option[1].baud_rate);
                    break;
                case SEGCP_DB: sprintf(trep, "%d", dev_config->serial_option[0].data_bits);
                    break;
                case SEGCP_ED: sprintf(trep, "%d", dev_config->serial_option[1].data_bits);
                    break;
                case SEGCP_PR: sprintf(trep, "%d", dev_config->serial_option[0].parity);
                    break;
                case SEGCP_EP: sprintf(trep, "%d", dev_config->serial_option[1].parity);
                    break;
                case SEGCP_SB: sprintf(trep, "%d", dev_config->serial_option[0].stop_bits);
                    break;
                case SEGCP_ES: sprintf(trep, "%d", dev_config->serial_option[1].stop_bits);
                    break;
                case SEGCP_FL: sprintf(trep, "%d", dev_config->serial_option[0].flow_control);
                    break;
                case SEGCP_EF: sprintf(trep, "%d", dev_config->serial_option[1].flow_control);
                    break;
                case SEGCP_PO: sprintf(trep, "%d", dev_config->serial_option[0].protocol);
                    break;
                case SEGCP_EO: sprintf(trep, "%d", dev_config->serial_option[1].protocol);
                    break;
                case SEGCP_IT: sprintf(trep, "%d", dev_config->tcp_option[0].inactivity);
                    break;
                case SEGCP_RV: sprintf(trep, "%d", dev_config->tcp_option[1].inactivity);
                    break;
                case SEGCP_PT: sprintf(trep, "%d", dev_config->serial_data_packing[0].packing_time);
                    break;
                case SEGCP_AT: sprintf(trep, "%d", dev_config->serial_data_packing[1].packing_time);
                    break;
                case SEGCP_PS: sprintf(trep, "%d", dev_config->serial_data_packing[0].packing_size);
                    break;
                case SEGCP_NS: sprintf(trep, "%d", dev_config->serial_data_packing[1].packing_size);
                    break;
                case SEGCP_PD: sprintf(trep, "%02X", dev_config->serial_data_packing[0].packing_delimiter[0]);
                    break;
                case SEGCP_ND: sprintf(trep, "%02X", dev_config->serial_data_packing[1].packing_delimiter[0]);
                    break;
                case SEGCP_TE: sprintf(trep, "%d", dev_config->serial_command.serial_command);
                    break;
                case SEGCP_SS: sprintf(trep, "%02X%02X%02X", dev_config->serial_command.serial_trigger[0],
                                           dev_config->serial_command.serial_trigger[1],
                                           dev_config->serial_command.serial_trigger[2]);
                    break;
                case SEGCP_NP:
                    if (dev_config->tcp_option[0].pw_connect[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->tcp_option[0].pw_connect);
                    }
                    break;
                case SEGCP_SP:
                    if (dev_config->config_common.pw_search[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->config_common.pw_search);
                    }
                    break;
                case SEGCP_MA:
                case SEGCP_PW: ret |= SEGCP_RET_ERR_NOTAVAIL;
                    break;
#ifdef __USE_USERS_GPIO__
                // GET GPIOs Status / Value
                case SEGCP_GA:
                case SEGCP_GB:
                    io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_GA;
                    if (get_user_io_val(USER_IO_SEL[io_num], &tmp_int) != 0) {
                        sprintf(trep, "%d", tmp_int);
                    } else {
                        ret |= SEGCP_RET_ERR_NOTAVAIL;
                    }
                    break;

                // GET GPIOs settings; Type and Direction
                case SEGCP_CA:
                case SEGCP_CB:
                    io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_CA;
                    io_type = get_user_io_type(USER_IO_SEL[io_num]);
                    io_dir = get_user_io_direction(USER_IO_SEL[io_num]);
                    sprintf(trep, "%d", (((io_type & 0x01) << 1) | io_dir));
                    break;
#endif

                // GET Status pin's setting and status
                case SEGCP_SC: // mode select
                    sprintf(trep, "%d%d", dev_config->serial_option[0].dtr_en, dev_config->serial_option[0].dsr_en);
                    break;
                case SEGCP_S0:
                    sprintf(trep, "%d", get_connection_status_io(STATUS_PHYLINK_PIN)); // STATUS_PHYLINK_PIN (in) == DTR_PIN (out)
                    break;
                case SEGCP_S1:
#if (DEVICE_BOARD_NAME == PLATYPUS_S2E)
                    sprintf(trep, "%d", !get_connection_status_io(DATA0_STATUS_TCPCONNECT_PIN)); // STATUS_TCPCONNECT_PIN (in) == DSR_PIN (in)
#else
                    sprintf(trep, "%d", get_connection_status_io(DATA0_STATUS_TCPCONNECT_PIN)); // STATUS_TCPCONNECT_PIN (in) == DSR_PIN (in)
#endif

                    break;
                case SEGCP_RX:
                    data_buffer_flush(SEG_DATA0_CH);
                    sprintf(trep, "%s", "FLUSH");
                    break;
                case SEGCP_SV:
                    if (segcp_privilege & (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE)) {
                        ret |= SEGCP_RET_SAVE;
                    } else {
                        ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                    }
                    break;
                case SEGCP_EX:
                    if (segcp_privilege & (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE)) {
                        ret |= SEGCP_RET_SWITCH;
                    } else {
                        ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                    }
                    break;
                case SEGCP_RT:
                    if (segcp_privilege & (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE)) {
                        ret |= SEGCP_RET_REBOOT;
                    } else {
                        ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                    }
                    break;
                case SEGCP_UN:
                    sprintf(trep, "%s", uart_if_table[dev_config->serial_option[0].uart_interface]);
                    break;
                case SEGCP_EN:
                    sprintf(trep, "%s", uart_if_table[dev_config->serial_option[1].uart_interface]);
                    break;
                case SEGCP_UI:
                    sprintf(trep, "%d", dev_config->serial_option[0].uart_interface);
                    break;
                case SEGCP_EI:
                    sprintf(trep, "%d", dev_config->serial_option[1].uart_interface);
                    break;
                case SEGCP_ST:
                    sprintf(trep, "%s", strDEVSTATUS[dev_config->network_connection[0].working_state]);
                    break;
                case SEGCP_QS:
                    sprintf(trep, "%s", strDEVSTATUS[dev_config->network_connection[1].working_state]);
                    break;
                case SEGCP_FR:
                    if (segcp_privilege & (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE)) {
                        // #20161110 Hidden option, Local port number [1] + FR cmd => K! (EEPROM Erase)
                        if (dev_config->network_connection[0].local_port == 1) {
                            ret |= SEGCP_RET_ERASE_EEPROM | SEGCP_RET_REBOOT;    // EEPROM Erase
                        } else {
                            ret |= SEGCP_RET_FACTORY | SEGCP_RET_REBOOT;    // Factory Reset
                        }
                    } else {
                        ret |= SEGCP_RET_ERR_NOPRIVILEGE;
                    }
                    break;
                case SEGCP_EC:
                    sprintf(trep, "%d", dev_config->serial_command.serial_command_echo);
                    break;
                case SEGCP_TR:
                    sprintf(trep, "%d", dev_config->network_option.tcp_rcr_val);
                    break;
                case SEGCP_RC: // root ca option
                    sprintf(trep, "%d", dev_config->ssl_option[0].root_ca_option);
                    break;

                case SEGCP_CE: // client cert en/dis
                    sprintf(trep, "%d", dev_config->ssl_option[0].client_cert_enable);
                    break;

                case SEGCP_SO: // SSL Recv Timeout
                    sprintf(trep, "%d", dev_config->ssl_option[0].recv_timeout);
                    break;

                case SEGCP_RO: // SSL Recv Timeout
                    sprintf(trep, "%d", dev_config->ssl_option[1].recv_timeout);
                    break;

                case SEGCP_QU: // mqtt username
                    if (dev_config->mqtt_option[0].user_name[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->mqtt_option[0].user_name);
                    }
                    break;

                case SEGCP_QP: // mqtt password
                    if (dev_config->mqtt_option[0].password[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->mqtt_option[0].password);
                    }
                    break;

                case SEGCP_QC: // mqtt client id
                    if (dev_config->mqtt_option[0].client_id[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->mqtt_option[0].client_id);
                    }
                    break;

                case SEGCP_QK: // mqtt keepalive
                    sprintf(trep, "%d", dev_config->mqtt_option[0].keepalive);
                    break;

                case SEGCP_PU: // mqtt publish topic
                    if (dev_config->mqtt_option[0].pub_topic[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->mqtt_option[0].pub_topic);
                    }
                    break;

                case SEGCP_U0: // mqtt subscribe topic
                    if (dev_config->mqtt_option[0].sub_topic_0[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->mqtt_option[0].sub_topic_0);
                    }
                    //sprintf(trep, "%s", dev_config->mqtt_option[0].sub_topic);
                    break;

                case SEGCP_U1: // mqtt subscribe topic
                    if (dev_config->mqtt_option[0].sub_topic_1[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->mqtt_option[0].sub_topic_1);
                    }
                    //sprintf(trep, "%s", dev_config->mqtt_option[0].sub_topic);
                    break;

                case SEGCP_U2: // mqtt subscribe topic
                    if (dev_config->mqtt_option[0].sub_topic_2[0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->mqtt_option[0].sub_topic_2);
                    }
                    //sprintf(trep, "%s", dev_config->mqtt_option[0].sub_topic);
                    break;

                case SEGCP_QO: // mqtt qos level
                    sprintf(trep, "%d", dev_config->mqtt_option[0].qos);
                    break;

                case SEGCP_UF: // fw bank copy flag
                    sprintf(trep, "%d", dev_config->firmware_update.fwup_copy_flag);
                    break;

                case SEGCP_SD: // device connect data
                    if (dev_config->device_option.device_serial_connect_data[0][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_connect_data[0]);
                    }
                    break;

                case SEGCP_DD: // device disconnect data
                    if (dev_config->device_option.device_serial_disconnect_data[0][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_disconnect_data[0]);
                    }
                    break;

                case SEGCP_RD: // device connect data
                    if (dev_config->device_option.device_serial_connect_data[1][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_connect_data[1]);
                    }
                    break;

                case SEGCP_RF: // device disconnect data
                    if (dev_config->device_option.device_serial_disconnect_data[1][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_disconnect_data[1]);
                    }
                    break;

                case SEGCP_SE:
                    if (dev_config->device_option.device_eth_connect_data[0][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_eth_connect_data[0]);
                    }
                    break;

                case SEGCP_EE:
                    if (dev_config->device_option.device_eth_connect_data[1][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_eth_connect_data[1]);
                    }
                    break;

#if (DEVICE_UART_CNT > 2)
                // ---- ch2 (index [2]) GET ----
                case SEGCP_GS: sprintf(trep, "%s", strDEVSTATUS[dev_config->network_connection[2].working_state]); break; // status(R)
                case SEGCP_WN: sprintf(trep, "%s", uart_if_table[dev_config->serial_option[2].uart_interface]); break;    // IF str(R)
                case SEGCP_WI: sprintf(trep, "%d", dev_config->serial_option[2].uart_interface); break;                   // IF num(R)
                case SEGCP_TO: sprintf(trep, "%d", dev_config->network_connection[2].working_mode); break;                // opmode
                case SEGCP_GL: sprintf(trep, "%d", dev_config->network_connection[2].local_port); break;
                case SEGCP_GH: // remote host
                    if (dev_config->network_connection[2].dns_use == SEGCP_DISABLE) {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_connection[2].remote_ip[0],
                                dev_config->network_connection[2].remote_ip[1],
                                dev_config->network_connection[2].remote_ip[2],
                                dev_config->network_connection[2].remote_ip[3]);
                    } else {
                        if (dev_config->network_connection[2].dns_domain_name[0] == 0) {
                            sprintf(trep, "%c", SEGCP_NULL);
                        } else {
                            sprintf(trep, "%s", dev_config->network_connection[2].dns_domain_name);
                        }
                    }
                    break;
                case SEGCP_TP: sprintf(trep, "%d", dev_config->network_connection[2].remote_port); break;
                case SEGCP_WB: sprintf(trep, "%d", dev_config->serial_option[2].baud_rate); break;
                case SEGCP_WD: sprintf(trep, "%d", dev_config->serial_option[2].data_bits); break;
                case SEGCP_WP: sprintf(trep, "%d", dev_config->serial_option[2].parity); break;
                case SEGCP_WS: sprintf(trep, "%d", dev_config->serial_option[2].stop_bits); break;
                case SEGCP_WF: sprintf(trep, "%d", dev_config->serial_option[2].flow_control); break;
                case SEGCP_HD: sprintf(trep, "%02X", dev_config->serial_data_packing[2].packing_delimiter[0]); break;
                case SEGCP_HS: sprintf(trep, "%d", dev_config->serial_data_packing[2].packing_size); break;
                case SEGCP_TT: sprintf(trep, "%d", dev_config->serial_data_packing[2].packing_time); break;
                case SEGCP_XV: sprintf(trep, "%d", dev_config->tcp_option[2].inactivity); break;
                case SEGCP_XR: sprintf(trep, "%d", dev_config->tcp_option[2].reconnection); break;
                case SEGCP_XA: sprintf(trep, "%d", dev_config->tcp_option[2].keepalive_en); break;
                case SEGCP_XS: sprintf(trep, "%d", dev_config->tcp_option[2].keepalive_wait_time); break;
                case SEGCP_XE: sprintf(trep, "%d", dev_config->tcp_option[2].keepalive_retry_time); break;
                case SEGCP_XO: sprintf(trep, "%d", dev_config->ssl_option[2].recv_timeout); break;
                case SEGCP_WO: sprintf(trep, "%d", dev_config->serial_option[2].protocol); break;
                case SEGCP_XD:
                    if (dev_config->device_option.device_serial_connect_data[2][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_connect_data[2]);
                    }
                    break;
                case SEGCP_XF:
                    if (dev_config->device_option.device_serial_disconnect_data[2][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_disconnect_data[2]);
                    }
                    break;
                case SEGCP_WE:
                    if (dev_config->device_option.device_eth_connect_data[2][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_eth_connect_data[2]);
                    }
                    break;
#endif
#if (DEVICE_UART_CNT > 3)
                // ---- ch3 (index [3]) GET ----
                case SEGCP_CS: sprintf(trep, "%s", strDEVSTATUS[dev_config->network_connection[3].working_state]); break; // status(R)
                case SEGCP_YN: sprintf(trep, "%s", uart_if_table[dev_config->serial_option[3].uart_interface]); break;    // IF str(R)
                case SEGCP_YI: sprintf(trep, "%d", dev_config->serial_option[3].uart_interface); break;                   // IF num(R)
                case SEGCP_JO: sprintf(trep, "%d", dev_config->network_connection[3].working_mode); break;                // opmode
                case SEGCP_CL: sprintf(trep, "%d", dev_config->network_connection[3].local_port); break;
                case SEGCP_CH: // remote host
                    if (dev_config->network_connection[3].dns_use == SEGCP_DISABLE) {
                        sprintf(trep, "%d.%d.%d.%d", dev_config->network_connection[3].remote_ip[0],
                                dev_config->network_connection[3].remote_ip[1],
                                dev_config->network_connection[3].remote_ip[2],
                                dev_config->network_connection[3].remote_ip[3]);
                    } else {
                        if (dev_config->network_connection[3].dns_domain_name[0] == 0) {
                            sprintf(trep, "%c", SEGCP_NULL);
                        } else {
                            sprintf(trep, "%s", dev_config->network_connection[3].dns_domain_name);
                        }
                    }
                    break;
                case SEGCP_JP: sprintf(trep, "%d", dev_config->network_connection[3].remote_port); break;
                case SEGCP_YB: sprintf(trep, "%d", dev_config->serial_option[3].baud_rate); break;
                case SEGCP_YD: sprintf(trep, "%d", dev_config->serial_option[3].data_bits); break;
                case SEGCP_YP: sprintf(trep, "%d", dev_config->serial_option[3].parity); break;
                case SEGCP_YS: sprintf(trep, "%d", dev_config->serial_option[3].stop_bits); break;
                case SEGCP_YF: sprintf(trep, "%d", dev_config->serial_option[3].flow_control); break;
                case SEGCP_UD: sprintf(trep, "%02X", dev_config->serial_data_packing[3].packing_delimiter[0]); break;
                case SEGCP_US: sprintf(trep, "%d", dev_config->serial_data_packing[3].packing_size); break;
                case SEGCP_JT: sprintf(trep, "%d", dev_config->serial_data_packing[3].packing_time); break;
                case SEGCP_ZV: sprintf(trep, "%d", dev_config->tcp_option[3].inactivity); break;
                case SEGCP_ZR: sprintf(trep, "%d", dev_config->tcp_option[3].reconnection); break;
                case SEGCP_ZA: sprintf(trep, "%d", dev_config->tcp_option[3].keepalive_en); break;
                case SEGCP_ZS: sprintf(trep, "%d", dev_config->tcp_option[3].keepalive_wait_time); break;
                case SEGCP_ZE: sprintf(trep, "%d", dev_config->tcp_option[3].keepalive_retry_time); break;
                case SEGCP_ZO: sprintf(trep, "%d", dev_config->ssl_option[3].recv_timeout); break;
                case SEGCP_YO: sprintf(trep, "%d", dev_config->serial_option[3].protocol); break;
                case SEGCP_ZD:
                    if (dev_config->device_option.device_serial_connect_data[3][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_connect_data[3]);
                    }
                    break;
                case SEGCP_ZF:
                    if (dev_config->device_option.device_serial_disconnect_data[3][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_serial_disconnect_data[3]);
                    }
                    break;
                case SEGCP_YE:
                    if (dev_config->device_option.device_eth_connect_data[3][0] == 0) {
                        sprintf(trep, "%c", SEGCP_NULL);
                    } else {
                        sprintf(trep, "%s", dev_config->device_option.device_eth_connect_data[3]);
                    }
                    break;
#endif
                default:
                    //ret |= SEGCP_RET_ERR_NOCOMMAND;
                    //sprintf(trep,"%s", strDEVSTATUS[dev_config->network_connection[0].working_state]);
                    sprintf(trep, "%c", SEGCP_NULL);
                    break;
                }

                if (ret & (SEGCP_RET_ERR | SEGCP_RET_REBOOT | SEGCP_RET_SWITCH | SEGCP_RET_SAVE))
                    //if(ret & (SEGCP_RET_REBOOT | SEGCP_RET_SWITCH | SEGCP_RET_SAVE))
                {
                    trep -= SEGCP_CMD_MAX;
                    *trep = 0;
                } else {
                    strcat(trep, SEGCP_DELIMETER);
                    trep += strlen(trep);
                }
            } else if (segcp_privilege & (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE)) {
                switch ((teSEGCPCMDNUM)cmdnum) {
                case SEGCP_MC:
                    if ((dev_config->network_common.mac[0] == MAC_OUI0) && (dev_config->network_common.mac[1] == MAC_OUI1) && (dev_config->network_common.mac[2] == MAC_OUI2)) {
                        ret |= SEGCP_RET_ERR_IGNORED;
                    } else if (!is_macaddr(param, ".:-", dev_config->network_common.mac)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
                    break;
                case SEGCP_VR:
                case SEGCP_MN:
                    ret |= SEGCP_RET_ERR_IGNORED;
                    break;
                case SEGCP_IM:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_DHCP) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_option.dhcp_use = tmp_byte;
                    }
                    break;
                case SEGCP_OP:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UDP_MODE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA0_CH, TRUE);
                        dev_config->network_connection[0].working_mode = tmp_byte;
                    }
                    break;

                case SEGCP_AO:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UDP_MODE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        process_socket_termination(SEG_DATA1_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA1_CH, TRUE);
                        dev_config->network_connection[1].working_mode = tmp_byte;
                    }
                    break;

                case SEGCP_CP:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_ENABLE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[0].pw_connect_en = tmp_byte;
                    }
                    break;
                case SEGCP_DG:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEG_DEBUG_ALL) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_common.serial_debug_en = tmp_byte;
                    }
                    break;
                case SEGCP_KA:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_ENABLE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[0].keepalive_en = tmp_byte;
                    }
                    break;
                case SEGCP_RA:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_ENABLE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[1].keepalive_en = tmp_byte;
                    }
                    break;
                case SEGCP_KI:
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[0].keepalive_wait_time = (uint16_t) tmp_long;
                    }
                    break;
                case SEGCP_RS:
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[1].keepalive_wait_time = (uint16_t) tmp_long;
                    }
                    break;
                case SEGCP_KE:
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[0].keepalive_retry_time = (uint16_t) tmp_long;
                    }
                    break;
                case SEGCP_RE:
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[1].keepalive_retry_time = (uint16_t) tmp_long;
                    }
                    break;
                case SEGCP_RI:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[0].reconnection = (uint16_t) tmp_long;
                    }
                    break;
                case SEGCP_RR:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[1].reconnection = (uint16_t) tmp_long;
                    }
                    break;
                case SEGCP_LI:
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_common.local_ip[0] = tmp_ip[0];
                        dev_config->network_common.local_ip[1] = tmp_ip[1];
                        dev_config->network_common.local_ip[2] = tmp_ip[2];
                        dev_config->network_common.local_ip[3] = tmp_ip[3];
                    } else {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
                    break;
                case SEGCP_SM:
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_common.subnet[0] = tmp_ip[0];
                        dev_config->network_common.subnet[1] = tmp_ip[1];
                        dev_config->network_common.subnet[2] = tmp_ip[2];
                        dev_config->network_common.subnet[3] = tmp_ip[3];
                    } else {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
                    break;
                case SEGCP_GW:
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_common.gateway[0] = tmp_ip[0];
                        dev_config->network_common.gateway[1] = tmp_ip[1];
                        dev_config->network_common.gateway[2] = tmp_ip[2];
                        dev_config->network_common.gateway[3] = tmp_ip[3];
                    } else {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
                    break;
                case SEGCP_DS:
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_option.dns_server_ip[0] = tmp_ip[0];
                        dev_config->network_option.dns_server_ip[1] = tmp_ip[1];
                        dev_config->network_option.dns_server_ip[2] = tmp_ip[2];
                        dev_config->network_option.dns_server_ip[3] = tmp_ip[3];
                    } else {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
                    break;

                case SEGCP_DH:
                    if (param_len > sizeof(dev_config->device_common.device_name) -1) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if (param[0] == SEGCP_NULL) {
                            dev_config->device_common.device_name[0] = 0;
                        } else {
                            sprintf(dev_config->device_common.device_name, "%s", param);
                        }
                    }
                    break;
                case SEGCP_LP:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[0].local_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_QL:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[1].local_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_RP:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[0].remote_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_AP:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[1].remote_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_RH:
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_connection[0].dns_use = SEGCP_DISABLE;
                        dev_config->network_connection[0].remote_ip[0] = tmp_ip[0];
                        dev_config->network_connection[0].remote_ip[1] = tmp_ip[1];
                        dev_config->network_connection[0].remote_ip[2] = tmp_ip[2];
                        dev_config->network_connection[0].remote_ip[3] = tmp_ip[3];
                        segcp_store_string(dev_config->network_connection[0].dns_domain_name, sizeof(dev_config->network_connection[0].dns_domain_name), param, &ret);
                    } else {
                        dev_config->network_connection[0].dns_use = SEGCP_ENABLE;
                        if (param[0] == SEGCP_NULL) {
                            dev_config->network_connection[0].dns_domain_name[0] = 0;
                        } else {
                            segcp_store_string(dev_config->network_connection[0].dns_domain_name, sizeof(dev_config->network_connection[0].dns_domain_name), param, &ret);
                        }
                    }
                    break;
                case SEGCP_QH:
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_connection[1].dns_use = SEGCP_DISABLE;
                        dev_config->network_connection[1].remote_ip[0] = tmp_ip[0];
                        dev_config->network_connection[1].remote_ip[1] = tmp_ip[1];
                        dev_config->network_connection[1].remote_ip[2] = tmp_ip[2];
                        dev_config->network_connection[1].remote_ip[3] = tmp_ip[3];
                        segcp_store_string(dev_config->network_connection[1].dns_domain_name, sizeof(dev_config->network_connection[1].dns_domain_name), param, &ret);
                    } else {
                        dev_config->network_connection[1].dns_use = SEGCP_ENABLE;
                        if (param[0] == SEGCP_NULL) {
                            dev_config->network_connection[1].dns_domain_name[0] = 0;
                        } else {
                            segcp_store_string(dev_config->network_connection[1].dns_domain_name, sizeof(dev_config->network_connection[1].dns_domain_name), param, &ret);
                        }
                    }
                    break;
                case SEGCP_BR:
                    tmp_int = atoi(param);
#if (DEVICE_BOARD_NAME == W232N)
                    if (param_len > 2 || tmp_int > baud_230400) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#else
                    if (param_len > 2 || tmp_int >= baud_max) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#endif
                    else {
                        dev_config->serial_option[0].baud_rate = (uint8_t)tmp_int;
                    }
                    break;
                case SEGCP_EB:
                    tmp_int = atoi(param);
#if (DEVICE_BOARD_NAME == W232N)
                    if (param_len > 2 || tmp_int > baud_230400) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#else
                    if (param_len > 2 || tmp_int >= baud_max) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#endif
                    else {
                        dev_config->serial_option[1].baud_rate = (uint8_t)tmp_int;
                    }
                    break;
                case SEGCP_DB:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > word_len8) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[0].data_bits = tmp_byte;
                    }
                    break;
                case SEGCP_ED:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > word_len8) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[1].data_bits = tmp_byte;
                    }
                    break;
                case SEGCP_PR:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > parity_even) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[0].parity = tmp_byte;
                    }
                    break;
                case SEGCP_EP:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > parity_even) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[1].parity = tmp_byte;
                    }
                    break;
                case SEGCP_SB:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > stop_bit2) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[0].stop_bits = tmp_byte;
                    }
                    break;
                case SEGCP_ES:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > stop_bit2) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[1].stop_bits = tmp_byte;
                    }
                    break;
                case SEGCP_FL:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > flow_dtr_dsr) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if (dev_config->serial_option[0].uart_interface != UART_IF_RS232_TTL) {
                            if ((tmp_byte != flow_rtsonly) && (tmp_byte != flow_reverserts)) {
                                dev_config->serial_option[0].flow_control = flow_none;
                            } else {
                                dev_config->serial_option[0].flow_control = tmp_byte;
                            }
                        } else {
                            dev_config->serial_option[0].flow_control = tmp_byte;
                        }
                    }
                    break;
                case SEGCP_EF:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > flow_dtr_dsr) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if (dev_config->serial_option[1].uart_interface != UART_IF_RS232_TTL) {
                            if ((tmp_byte != flow_rtsonly) && (tmp_byte != flow_reverserts)) {
                                dev_config->serial_option[1].flow_control = flow_none;
                            } else {
                                dev_config->serial_option[1].flow_control = tmp_byte;
                            }
                        } else {
                            dev_config->serial_option[1].flow_control = tmp_byte;
                        }
                    }
                    break;
                case SEGCP_PO:
                    tmp_int = atoi(param);
                    if (param_len > 2 || tmp_int > modbus_ascii) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[0].protocol = tmp_int;
                    }
                    break;
                case SEGCP_EO:
                    tmp_int = atoi(param);
                    if (param_len > 2 || tmp_int > modbus_ascii) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[1].protocol = tmp_int;
                    }
                    break;
                case SEGCP_IT:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[0].inactivity = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_RV:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[1].inactivity = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_PT:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[0].packing_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_AT:
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[1].packing_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_PS:
                    tmp_int = atoi(param);
                    if (param_len > 4 || tmp_int > (SEG_DATA_BUF_SIZE / 2)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[0].packing_size = (uint16_t)tmp_int;
                    }
                    break;
                case SEGCP_NS:
                    tmp_int = atoi(param);
                    if (param_len > 4 || tmp_int > (SEG_DATA_BUF_SIZE / 2)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[1].packing_size = (uint16_t)tmp_int;
                    }
                    break;
                case SEGCP_PD:
                    if (param_len != 2 || !is_hexstr(param)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        str_to_hex(param, &tmp_byte);
                        dev_config->serial_data_packing[0].packing_delimiter[0] = tmp_byte;

                        if (dev_config->serial_data_packing[0].packing_delimiter[0] == 0x00) {
                            dev_config->serial_data_packing[0].packing_delimiter_length = 0;
                        } else {
                            dev_config->serial_data_packing[0].packing_delimiter_length = 1;
                        }
                    }
                    break;
                case SEGCP_ND:
                    if (param_len != 2 || !is_hexstr(param)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        str_to_hex(param, &tmp_byte);
                        dev_config->serial_data_packing[1].packing_delimiter[0] = tmp_byte;

                        if (dev_config->serial_data_packing[1].packing_delimiter[0] == 0x00) {
                            dev_config->serial_data_packing[1].packing_delimiter_length = 0;
                        } else {
                            dev_config->serial_data_packing[1].packing_delimiter_length = 1;
                        }
                    }
                    break;
                case SEGCP_TE:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_ENABLE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_command.serial_command = tmp_byte;
                    }
                    break;
                case SEGCP_SS:
                    if (param_len != 6 || !is_hexstr(param) || !str_to_hex(param, dev_config->serial_command.serial_trigger)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
                    break;
                case SEGCP_NP:
                    if (param_len > sizeof(dev_config->tcp_option[0].pw_connect) -1) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if (param[0] == SEGCP_NULL) {
                            dev_config->tcp_option[0].pw_connect[0] = 0;
                        } else {
                            sprintf(dev_config->tcp_option[0].pw_connect, "%s", param);
                        }
                    }
                    break;
                case SEGCP_SP:
                    if (param_len > sizeof(dev_config->config_common.pw_search) -1) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if (param[0] == SEGCP_NULL) {
                            dev_config->config_common.pw_search[0] = 0;
                        } else {
                            sprintf(dev_config->config_common.pw_search, "%s", param);
                        }
                    }
                    break;

                // SET status pin mode selector
                case SEGCP_SC:
#if (DEVICE_BOARD_NAME == W55RP20_S2E)
                    // This board has no dedicated DTR/DSR pins; the signals share the
                    // RTS/CTS pins and flow_control alone picks which function they serve.
                    // Accepting the selector would store a value nothing reads.
                    ret |= SEGCP_RET_ERR_NOTAVAIL;
#else
                    str_to_hex(param, &tmp_byte);

                    tmp_int = (tmp_byte & 0xF0) >> 4;   // [0] PHY link / [1] DTR
                    tmp_byte = (tmp_byte & 0x0F);       // [0] TCP connection / [1] DSR

                    if ((param_len > 2) || (tmp_byte > IO_HIGH) || (tmp_int > IO_HIGH)) { // Invalid parameters
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[0].dtr_en = (uint8_t)tmp_int;
                        dev_config->serial_option[0].dsr_en = tmp_byte;

                        // Set the DTR pin to high when the DTR signal enabled (== PHY link status disabled)
                        if (dev_config->serial_option[0].dtr_en == SEGCP_ENABLE) {
                            set_flowcontrol_dtr_pin(ON, SEG_DATA0_CH);
                        }
                    }
#endif
                    break;
                case SEGCP_S0:
                case SEGCP_S1:
                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    break;

                case SEGCP_RX:
                    ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    break;

                case SEGCP_EC:
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_ENABLE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_command.serial_command_echo = tmp_byte;
                    }
                    break;

                case SEGCP_TR: // TCP Retransmission retry count
                    tmp_int = atoi(param);
                    if ((param_len > 3) || (tmp_int < 1) || (tmp_int > 0xFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_option.tcp_rcr_val = (uint8_t)tmp_int;
                    }
                    break;

                case SEGCP_RC: // root ca option
                    tmp_byte = atoi(param);
                    if (tmp_byte > 2) { //0: Verify_none / 1: Verify_option / 2: Verify_require
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    }
                    dev_config->ssl_option[0].root_ca_option = tmp_byte;
                    break;

                case SEGCP_CE: // client cert en/dis
                    tmp_byte = atoi(param);
                    if (tmp_byte > 1) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    }
                    dev_config->ssl_option[0].client_cert_enable = tmp_byte;
                    break;

                case SEGCP_SO: // SSL Recv Timeout
                    tmp_int = atoi(param);
                    if (tmp_int > SSL_RECV_MAX_TIMEOUT) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->ssl_option[0].recv_timeout = tmp_int;
                    }
                    break;

                case SEGCP_RO: // SSL Recv Timeout
                    tmp_int = atoi(param);
                    if (tmp_int > SSL_RECV_MAX_TIMEOUT) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->ssl_option[1].recv_timeout = tmp_int;
                    }
                    break;

                case SEGCP_QU: // mqtt username
                    if (param[0] == SEGCP_NULL) {
                        dev_config->mqtt_option[0].user_name[0] = 0;
                    } else {
                        segcp_store_string(dev_config->mqtt_option[0].user_name, sizeof(dev_config->mqtt_option[0].user_name), param, &ret);
                    }
                    break;

                case SEGCP_QP: // mqtt password
                    if (param[0] == SEGCP_NULL) {
                        dev_config->mqtt_option[0].password[0] = 0;
                    } else {
                        segcp_store_string(dev_config->mqtt_option[0].password, sizeof(dev_config->mqtt_option[0].password), param, &ret);
                    }
                    break;


                case SEGCP_QC: // mqtt client id
                    if (param[0] == SEGCP_NULL) {
                        dev_config->mqtt_option[0].client_id[0] = 0;
                    } else {
                        segcp_store_string(dev_config->mqtt_option[0].client_id, sizeof(dev_config->mqtt_option[0].client_id), param, &ret);
                    }
                    break;

                case SEGCP_QK: // mqtt keepalive
                    dev_config->mqtt_option[0].keepalive = atoi(param);
                    break;

                case SEGCP_PU: // mqtt publish topic
                    if (param[0] == SEGCP_NULL) {
                        dev_config->mqtt_option[0].pub_topic[0] = 0;
                    } else {
                        segcp_store_string(dev_config->mqtt_option[0].pub_topic, sizeof(dev_config->mqtt_option[0].pub_topic), param, &ret);
                    }
                    break;

                case SEGCP_U0: // mqtt subscribe topic
                    if (param[0] == SEGCP_NULL) {
                        dev_config->mqtt_option[0].sub_topic_0[0] = 0;
                    } else {
                        segcp_store_string(dev_config->mqtt_option[0].sub_topic_0, sizeof(dev_config->mqtt_option[0].sub_topic_0), param, &ret);
                    }
                    break;

                case SEGCP_U1: // mqtt subscribe topic
                    if (param[0] == SEGCP_NULL) {
                        dev_config->mqtt_option[0].sub_topic_1[0] = 0;
                    } else {
                        segcp_store_string(dev_config->mqtt_option[0].sub_topic_1, sizeof(dev_config->mqtt_option[0].sub_topic_1), param, &ret);
                    }
                    break;

                case SEGCP_U2: // mqtt subscribe topic
                    if (param[0] == SEGCP_NULL) {
                        dev_config->mqtt_option[0].sub_topic_2[0] = 0;
                    } else {
                        segcp_store_string(dev_config->mqtt_option[0].sub_topic_2, sizeof(dev_config->mqtt_option[0].sub_topic_2), param, &ret);
                    }
                    break;

                case SEGCP_QO: // mqtt qos level
                    tmp_byte = atoi(param);
                    if (tmp_byte > 2) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    }
                    dev_config->mqtt_option[0].qos = tmp_byte;
                    break;
                case SEGCP_UF: // Current Bank
                    tmp_byte = atoi(param);
                    if (tmp_byte > 1) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    }
                    if (0 == device_bank_check(tmp_byte)) {
                        dev_config->firmware_update.fwup_copy_flag = tmp_byte;
                    }
                    break;

#ifdef __USE_S2E_OVER_TLS__
                case SEGCP_OC: { // rootca
                    int32_t received_len;
                    temp_buf = pvPortMalloc(ROOTCA_BUF_SIZE);
                    if (temp_buf == NULL) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    }
                    tmp_ptr = temp_buf;
                    memset(tmp_ptr, 0, ROOTCA_BUF_SIZE);
                    sprintf(tmp_ptr, "%s", treq + SEGCP_CMD_MAX);
                    tmp_ptr += strlen(tmp_ptr);

                    while ((len = segcp_socket_rx_available(SEGCP_UDP_SOCK)) > 0) {
                        size_t used = (size_t)(tmp_ptr - temp_buf);
                        size_t remaining;

                        if (used >= (ROOTCA_BUF_SIZE - 1U)) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        remaining = (ROOTCA_BUF_SIZE - 1U) - used;
                        if (len > remaining) {
                            len = remaining;
                        }
                        received_len = segcp_socket_recvfrom(SEGCP_UDP_SOCK,
                                                             tmp_ptr, (uint16_t)len, tmp_ip, &tmp_port);
                        if (received_len <= 0) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        tmp_ptr += received_len;
                    }
                    if (!(ret & SEGCP_RET_ERR)) {
                        tmp_ptr = strstr(temp_buf, END_CERT);
                        if (tmp_ptr == NULL) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        tmp_ptr += strlen(END_CERT);

                        if (*tmp_ptr == '\n') {
                            tmp_ptr++;
                        } else if (*tmp_ptr == '\r') {
                            tmp_ptr += 2;
                        } else {
                            *(tmp_ptr) = '\r';
                            *(tmp_ptr + 1) = '\n';
                            tmp_ptr += 2;
                        }

                        dev_config->ssl_option[0].rootca_len = tmp_ptr - temp_buf;
                        temp_buf[dev_config->ssl_option[0].rootca_len] = 0;

                        //PRT_SEGCP("rootca_data = \r\n%s\r\n", temp_buf);

                        ret_2 = check_ca(temp_buf, dev_config->ssl_option[0].rootca_len);
                        if (ret_2 < 0) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        } else {
                            save_DevConfig_to_storage();
                            erase_storage(STORAGE_ROOTCA0);
                            write_storage(STORAGE_ROOTCA0, 0, (uint8_t *)temp_buf, dev_config->ssl_option[0].rootca_len + 1);

                            memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                            trep += SEGCP_CMD_MAX;
                            strcat(trep, SEGCP_DELIMETER);
                        }
                    }
                    vPortFree(temp_buf);
                    return ret;
                }
                break;

                case SEGCP_LC: { // client_cert
                    int32_t received_len;
                    temp_buf = pvPortMalloc(CLICA_BUF_SIZE);
                    if (temp_buf == NULL) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    }
                    tmp_ptr = temp_buf;
                    memset(tmp_ptr, 0, CLICA_BUF_SIZE);
                    sprintf(tmp_ptr, "%s", treq + SEGCP_CMD_MAX);

                    tmp_ptr += strlen(tmp_ptr);
                    while ((len = segcp_socket_rx_available(SEGCP_UDP_SOCK)) > 0) {
                        size_t used = (size_t)(tmp_ptr - temp_buf);
                        size_t remaining;

                        if (used >= (CLICA_BUF_SIZE - 1U)) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        remaining = (CLICA_BUF_SIZE - 1U) - used;
                        if (len > remaining) {
                            len = remaining;
                        }
                        received_len = segcp_socket_recvfrom(SEGCP_UDP_SOCK,
                                                             tmp_ptr, (uint16_t)len, tmp_ip, &tmp_port);
                        if (received_len <= 0) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        tmp_ptr += received_len;
                    }
                    if (!(ret & SEGCP_RET_ERR)) {
                        tmp_ptr = strstr(temp_buf, END_CERT);
                        if (tmp_ptr == NULL) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        tmp_ptr += strlen(END_CERT);

                        if (*tmp_ptr == '\n') {
                            tmp_ptr++;
                        } else if (*tmp_ptr == '\r') {
                            tmp_ptr += 2;
                        } else {
                            *(tmp_ptr) = '\r';
                            *(tmp_ptr + 1) = '\n';
                            tmp_ptr += 2;
                        }

                        dev_config->ssl_option[0].clica_len = tmp_ptr - temp_buf;
                        temp_buf[dev_config->ssl_option[0].clica_len] = 0;
                        ret_2 = check_ca(temp_buf, dev_config->ssl_option[0].clica_len);
                        if (ret_2 < 0) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        } else {
                            save_DevConfig_to_storage();
                            erase_storage(STORAGE_CLICA0);
                            write_storage(STORAGE_CLICA0, 0, (uint8_t *)temp_buf, dev_config->ssl_option[0].clica_len + 1);

                            memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                            trep += SEGCP_CMD_MAX;
                            strcat(trep, SEGCP_DELIMETER);
                        }
                    }
                    vPortFree(temp_buf);
                    return ret;
                }
                break;

                case SEGCP_PK: { // pkey
                    int32_t received_len;
                    temp_buf = pvPortMalloc(PKEY_BUF_SIZE);
                    if (temp_buf == NULL) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        break;
                    }
                    tmp_ptr = temp_buf;
                    memset(tmp_ptr, 0, PKEY_BUF_SIZE);
                    sprintf(tmp_ptr, "%s", treq + SEGCP_CMD_MAX);

                    tmp_ptr += strlen(tmp_ptr);
                    while ((len = segcp_socket_rx_available(SEGCP_UDP_SOCK)) > 0) {
                        size_t used = (size_t)(tmp_ptr - temp_buf);
                        size_t remaining;

                        if (used >= (PKEY_BUF_SIZE - 1U)) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        remaining = (PKEY_BUF_SIZE - 1U) - used;
                        if (len > remaining) {
                            len = remaining;
                        }
                        received_len = segcp_socket_recvfrom(SEGCP_UDP_SOCK,
                                                             tmp_ptr, (uint16_t)len, tmp_ip, &tmp_port);
                        if (received_len <= 0) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        tmp_ptr += received_len;
                    }
                    if (!(ret & SEGCP_RET_ERR)) {
                        tmp_ptr = strstr(temp_buf, END_PKEY);
                        if (tmp_ptr == NULL) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                            break;
                        }
                        tmp_ptr += strlen(END_PKEY);

                        if (*tmp_ptr == '\n') {
                            tmp_ptr++;
                        } else if (*tmp_ptr == '\r') {
                            tmp_ptr += 2;
                        } else {
                            *(tmp_ptr) = '\r';
                            *(tmp_ptr + 1) = '\n';
                            tmp_ptr += 2;
                        }

                        dev_config->ssl_option[0].pkey_len = tmp_ptr - temp_buf;
                        temp_buf[dev_config->ssl_option[0].pkey_len] = 0;
                        ret_2 = check_pkey(&s2e_tlsContext[SEG_DATA0_CH], temp_buf, dev_config->ssl_option[0].pkey_len);
                        if (ret_2 < 0) {
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        } else {
                            save_DevConfig_to_storage();
                            erase_storage(STORAGE_PKEY0);
                            write_storage(STORAGE_PKEY0, 0, (uint8_t *)temp_buf, dev_config->ssl_option[0].pkey_len + 1);

                            memcpy(trep, tbSEGCPCMD[cmdnum], SEGCP_CMD_MAX);
                            trep += SEGCP_CMD_MAX;
                            strcat(trep, SEGCP_DELIMETER);
                        }
                    }
                    vPortFree(temp_buf);
                    return ret;
                }
                break;
#endif // __USE_S2E_OVER_TLS__

                case SEGCP_FW: // f/w update
                    tmp_long = atol(param);

                    if (tmp_long > (uint32_t)FLASH_APP_BANK_SIZE) {
                        dev_config->firmware_update.fwup_size = 0;
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        PRT_SEGCP("SEGCP_FW:ERROR:TOOBIG\r\n");
                    } else {
                        dev_config->firmware_update.fwup_size = tmp_long;
                        dev_config->firmware_update.fwup_flag = SEGCP_ENABLE;
                        ret |= SEGCP_RET_FWUP_BANK;

                        sprintf(trep, "FW%d.%d.%d.%d:%d\r\n", dev_config->network_common.local_ip[0],
                                dev_config->network_common.local_ip[1],
                                dev_config->network_common.local_ip[2],
                                dev_config->network_common.local_ip[3],
                                (uint16_t)DEVICE_FWUP_PORT);

                        for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
                            process_socket_termination(seg_data_sock[ch],
                                                       SOCK_TERMINATION_DELAY, ch, TRUE);
                        }
                        PRT_SEGCP("SEGCP_FW:OK\r\n");
                    }
                    break;
                case SEGCP_SD: // device connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_connect_data[0][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_connect_data[0], sizeof(dev_config->device_option.device_serial_connect_data[0]), param, &ret);
                    }
                    break;

                case SEGCP_DD: // device disconnect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_disconnect_data[0][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_disconnect_data[0], sizeof(dev_config->device_option.device_serial_disconnect_data[0]), param, &ret);
                    }
                    break;

                case SEGCP_RD: // device connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_connect_data[1][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_connect_data[1], sizeof(dev_config->device_option.device_serial_connect_data[1]), param, &ret);
                    }
                    break;

                case SEGCP_RF: // device disconnect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_disconnect_data[1][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_disconnect_data[1], sizeof(dev_config->device_option.device_serial_disconnect_data[1]), param, &ret);
                    }
                    break;

                case SEGCP_SE: // device eth connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_eth_connect_data[0][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_eth_connect_data[0], sizeof(dev_config->device_option.device_eth_connect_data[0]), param, &ret);
                    }
                    break;

                case SEGCP_EE: // device eth connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_eth_connect_data[1][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_eth_connect_data[1], sizeof(dev_config->device_option.device_eth_connect_data[1]), param, &ret);
                    }
                    break;

#ifdef __USE_USERS_GPIO__
                // SET GPIOs Status / Value (Digital output only)
                case SEGCP_GA:
                case SEGCP_GB:
                    io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_GA;
                    tmp_int = is_hex(*param);
                    if (param_len != 1 || tmp_int > IO_HIGH) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else if (set_user_io_val(USER_IO_SEL[io_num], &tmp_int) == 0) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
                    break;

                // SET GPIOs settings; Type and Direction ('Analog output' mode is not allowed)
                case SEGCP_CA:
                case SEGCP_CB:
                    io_num = (teSEGCPCMDNUM)cmdnum - SEGCP_CA;
                    tmp_int = atoi(param);

                    io_type = (uint8_t)(tmp_int >> 1);
                    io_dir = (uint8_t)(tmp_int & 0x01);

                    if ((param_len > 2) || (io_type > IO_ANALOG_IN) || (io_dir > IO_OUTPUT)) { // Invalid parameters
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if ((io_type == IO_ANALOG_IN) && (io_dir == IO_OUTPUT)) { // This case not allowed. (Analog output)
                            ret |= SEGCP_RET_ERR_INVALIDPARAM;
                        } else {
                            // IO type and Direction settings
                            set_user_io_type(USER_IO_SEL[io_num], io_type);
                            set_user_io_direction(USER_IO_SEL[io_num], io_dir);
                            init_user_io(USER_IO_SEL[io_num]);
                        }
                    }
                    break;
#endif
                case SEGCP_UI: // ch0 serial IF (num) — R/W
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UART_IF_RS485_REVERSE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[0].uart_interface = tmp_byte;
                    }
                    break;
                case SEGCP_EI: // ch1 serial IF (num) — R/W
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UART_IF_RS485_REVERSE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[1].uart_interface = tmp_byte;
                    }
                    break;
#if (DEVICE_UART_CNT > 2)
                // ---- ch2 (index [2]) SET ----
                case SEGCP_TO: // opmode
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UDP_MODE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        process_socket_termination(SEG_DATA2_SOCK,
                                                   SOCK_TERMINATION_DELAY, SEG_DATA2_CH, TRUE);
                        dev_config->network_connection[2].working_mode = tmp_byte;
                    }
                    break;
                case SEGCP_GL: // local port
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[2].local_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_GH: // remote host
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_connection[2].dns_use = SEGCP_DISABLE;
                        dev_config->network_connection[2].remote_ip[0] = tmp_ip[0];
                        dev_config->network_connection[2].remote_ip[1] = tmp_ip[1];
                        dev_config->network_connection[2].remote_ip[2] = tmp_ip[2];
                        dev_config->network_connection[2].remote_ip[3] = tmp_ip[3];
                        segcp_store_string(dev_config->network_connection[2].dns_domain_name, sizeof(dev_config->network_connection[2].dns_domain_name), param, &ret);
                    } else {
                        dev_config->network_connection[2].dns_use = SEGCP_ENABLE;
                        if (param[0] == SEGCP_NULL) {
                            dev_config->network_connection[2].dns_domain_name[0] = 0;
                        } else {
                            segcp_store_string(dev_config->network_connection[2].dns_domain_name, sizeof(dev_config->network_connection[2].dns_domain_name), param, &ret);
                        }
                    }
                    break;
                case SEGCP_TP: // remote port
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[2].remote_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_WB: // baud
                    tmp_int = atoi(param);
#if (DEVICE_BOARD_NAME == W232N)
                    if (param_len > 2 || tmp_int > baud_230400) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#else
                    if (param_len > 2 || tmp_int >= baud_max) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#endif
                    else {
                        dev_config->serial_option[2].baud_rate = (uint8_t)tmp_int;
                    }
                    break;
                case SEGCP_WD: // data bits
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > word_len8) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[2].data_bits = tmp_byte;
                    }
                    break;
                case SEGCP_WP: // parity
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > parity_even) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[2].parity = tmp_byte;
                    }
                    break;
                case SEGCP_WS: // stop bits
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > stop_bit2) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[2].stop_bits = tmp_byte;
                    }
                    break;
                case SEGCP_WF: // flow control (RS422/485 특례)
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > flow_dtr_dsr) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if (dev_config->serial_option[2].uart_interface != UART_IF_RS232_TTL) {
                            if ((tmp_byte != flow_rtsonly) && (tmp_byte != flow_reverserts)) {
                                dev_config->serial_option[2].flow_control = flow_none;
                            } else {
                                dev_config->serial_option[2].flow_control = tmp_byte;
                            }
                        } else {
                            dev_config->serial_option[2].flow_control = tmp_byte;
                        }
                    }
                    break;
                case SEGCP_HD: // pack delimiter
                    if (param_len != 2 || !is_hexstr(param)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        str_to_hex(param, &tmp_byte);
                        dev_config->serial_data_packing[2].packing_delimiter[0] = tmp_byte;
                        if (dev_config->serial_data_packing[2].packing_delimiter[0] == 0x00) {
                            dev_config->serial_data_packing[2].packing_delimiter_length = 0;
                        } else {
                            dev_config->serial_data_packing[2].packing_delimiter_length = 1;
                        }
                    }
                    break;
                case SEGCP_HS: // pack size
                    tmp_int = atoi(param);
                    if (param_len > 4 || tmp_int > (SEG_DATA_BUF_SIZE / 2)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[2].packing_size = (uint16_t)tmp_int;
                    }
                    break;
                case SEGCP_TT: // pack time
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[2].packing_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_XV: // inactivity
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[2].inactivity = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_XR: // reconnection
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[2].reconnection = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_XA: // keepalive en
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_ENABLE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[2].keepalive_en = tmp_byte;
                    }
                    break;
                case SEGCP_XS: // keepalive wait
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[2].keepalive_wait_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_XE: // keepalive retry
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[2].keepalive_retry_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_XO: // ssl recv timeout
                    tmp_int = atoi(param);
                    if (tmp_int > SSL_RECV_MAX_TIMEOUT) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->ssl_option[2].recv_timeout = tmp_int;
                    }
                    break;
                case SEGCP_WO: // protocol
                    tmp_int = atoi(param);
                    if (param_len > 2 || tmp_int > modbus_ascii) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[2].protocol = tmp_int;
                    }
                    break;
                case SEGCP_XD: // serial connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_connect_data[2][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_connect_data[2], sizeof(dev_config->device_option.device_serial_connect_data[2]), param, &ret);
                    }
                    break;
                case SEGCP_XF: // serial disconnect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_disconnect_data[2][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_disconnect_data[2], sizeof(dev_config->device_option.device_serial_disconnect_data[2]), param, &ret);
                    }
                    break;
                case SEGCP_WE: // eth connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_eth_connect_data[2][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_eth_connect_data[2], sizeof(dev_config->device_option.device_eth_connect_data[2]), param, &ret);
                    }
                    break;
                case SEGCP_WI: // ch2 serial IF (num) — R/W
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UART_IF_RS485_REVERSE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[2].uart_interface = tmp_byte;
                    }
                    break;
#endif
#if (DEVICE_UART_CNT > 3)
                // ---- ch3 (index [3]) SET ----
                case SEGCP_JO: // opmode
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UDP_MODE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        process_socket_termination(SEG_DATA3_SOCK,
                                                   SOCK_TERMINATION_DELAY, SEG_DATA3_CH, TRUE);
                        dev_config->network_connection[3].working_mode = tmp_byte;
                    }
                    break;
                case SEGCP_CL: // local port
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[3].local_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_CH: // remote host
                    if (is_ipaddr(param, tmp_ip)) {
                        dev_config->network_connection[3].dns_use = SEGCP_DISABLE;
                        dev_config->network_connection[3].remote_ip[0] = tmp_ip[0];
                        dev_config->network_connection[3].remote_ip[1] = tmp_ip[1];
                        dev_config->network_connection[3].remote_ip[2] = tmp_ip[2];
                        dev_config->network_connection[3].remote_ip[3] = tmp_ip[3];
                        segcp_store_string(dev_config->network_connection[3].dns_domain_name, sizeof(dev_config->network_connection[3].dns_domain_name), param, &ret);
                    } else {
                        dev_config->network_connection[3].dns_use = SEGCP_ENABLE;
                        if (param[0] == SEGCP_NULL) {
                            dev_config->network_connection[3].dns_domain_name[0] = 0;
                        } else {
                            segcp_store_string(dev_config->network_connection[3].dns_domain_name, sizeof(dev_config->network_connection[3].dns_domain_name), param, &ret);
                        }
                    }
                    break;
                case SEGCP_JP: // remote port
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->network_connection[3].remote_port = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_YB: // baud
                    tmp_int = atoi(param);
#if (DEVICE_BOARD_NAME == W232N)
                    if (param_len > 2 || tmp_int > baud_230400) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#else
                    if (param_len > 2 || tmp_int >= baud_max) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    }
#endif
                    else {
                        dev_config->serial_option[3].baud_rate = (uint8_t)tmp_int;
                    }
                    break;
                case SEGCP_YD: // data bits
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > word_len8) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[3].data_bits = tmp_byte;
                    }
                    break;
                case SEGCP_YP: // parity
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > parity_even) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[3].parity = tmp_byte;
                    }
                    break;
                case SEGCP_YS: // stop bits
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > stop_bit2) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[3].stop_bits = tmp_byte;
                    }
                    break;
                case SEGCP_YF: // flow control (RS422/485 특례)
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > flow_dtr_dsr) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        if (dev_config->serial_option[3].uart_interface != UART_IF_RS232_TTL) {
                            if ((tmp_byte != flow_rtsonly) && (tmp_byte != flow_reverserts)) {
                                dev_config->serial_option[3].flow_control = flow_none;
                            } else {
                                dev_config->serial_option[3].flow_control = tmp_byte;
                            }
                        } else {
                            dev_config->serial_option[3].flow_control = tmp_byte;
                        }
                    }
                    break;
                case SEGCP_UD: // pack delimiter
                    if (param_len != 2 || !is_hexstr(param)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        str_to_hex(param, &tmp_byte);
                        dev_config->serial_data_packing[3].packing_delimiter[0] = tmp_byte;
                        if (dev_config->serial_data_packing[3].packing_delimiter[0] == 0x00) {
                            dev_config->serial_data_packing[3].packing_delimiter_length = 0;
                        } else {
                            dev_config->serial_data_packing[3].packing_delimiter_length = 1;
                        }
                    }
                    break;
                case SEGCP_US: // pack size
                    tmp_int = atoi(param);
                    if (param_len > 4 || tmp_int > (SEG_DATA_BUF_SIZE / 2)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[3].packing_size = (uint16_t)tmp_int;
                    }
                    break;
                case SEGCP_JT: // pack time
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_data_packing[3].packing_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_ZV: // inactivity
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[3].inactivity = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_ZR: // reconnection
                    tmp_long = atol(param);
                    if (tmp_long > 0xFFFF) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[3].reconnection = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_ZA: // keepalive en
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > SEGCP_ENABLE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[3].keepalive_en = tmp_byte;
                    }
                    break;
                case SEGCP_ZS: // keepalive wait
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[3].keepalive_wait_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_ZE: // keepalive retry
                    tmp_long = atol(param);
                    if ((tmp_long < SEG_KEEPALIVE_MIN_INTERVAL_MS) ||
                            (tmp_long > 0xFFFF)) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->tcp_option[3].keepalive_retry_time = (uint16_t)tmp_long;
                    }
                    break;
                case SEGCP_ZO: // ssl recv timeout
                    tmp_int = atoi(param);
                    if (tmp_int > SSL_RECV_MAX_TIMEOUT) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->ssl_option[3].recv_timeout = tmp_int;
                    }
                    break;
                case SEGCP_YO: // protocol
                    tmp_int = atoi(param);
                    if (param_len > 2 || tmp_int > modbus_ascii) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[3].protocol = tmp_int;
                    }
                    break;
                case SEGCP_ZD: // serial connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_connect_data[3][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_connect_data[3], sizeof(dev_config->device_option.device_serial_connect_data[3]), param, &ret);
                    }
                    break;
                case SEGCP_ZF: // serial disconnect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_serial_disconnect_data[3][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_serial_disconnect_data[3], sizeof(dev_config->device_option.device_serial_disconnect_data[3]), param, &ret);
                    }
                    break;
                case SEGCP_YE: // eth connect data
                    if (param[0] == SEGCP_NULL) {
                        dev_config->device_option.device_eth_connect_data[3][0] = 0;
                    } else {
                        segcp_store_string(dev_config->device_option.device_eth_connect_data[3], sizeof(dev_config->device_option.device_eth_connect_data[3]), param, &ret);
                    }
                    break;
                case SEGCP_YI: // ch3 serial IF (num) — R/W
                    tmp_byte = is_hex(*param);
                    if (param_len != 1 || tmp_byte > UART_IF_RS485_REVERSE) {
                        ret |= SEGCP_RET_ERR_INVALIDPARAM;
                    } else {
                        dev_config->serial_option[3].uart_interface = tmp_byte;
                    }
                    break;
#endif
                case SEGCP_EN:
                case SEGCP_ST:
                case SEGCP_QS:
                case SEGCP_MA:
                case SEGCP_EX:
                case SEGCP_SV:
                case SEGCP_RT:
                case SEGCP_FR:
                case SEGCP_PW:
#if (DEVICE_UART_CNT > 2)
                case SEGCP_GS: // status(R/O twin)
                case SEGCP_WN: // IF str(R/O twin)
#endif
#if (DEVICE_UART_CNT > 3)
                case SEGCP_CS:
                case SEGCP_YN:
#endif
                    ret |= SEGCP_RET_ERR_NOTAVAIL;
                    break;
                default:
                    //ret |= SEGCP_RET_ERR_NOCOMMAND;
                    break;
                }
            } else {
                ret |= SEGCP_RET_ERR_NOPRIVILEGE;
            }
        } else {
            ret |= SEGCP_RET_ERR_NOCOMMAND;
        }

        // Process the serial command mode
        if (opmode == DEVICE_AT_MODE) {
            if (ret & SEGCP_RET_ERR) {
                treq[2] = 0;
                sprintf(trep, "%s:%s\r\n", tbSEGCPERR[((ret - SEGCP_RET_ERR) >> 8)], (cmdnum != SEGCP_UNKNOWN) ? tbSEGCPCMD[cmdnum] : treq);
#ifdef DBG_LEVEL_SEGCP
                PRT_SEGCP("ERROR : %s\r\n", trep);
#endif
                data_buffer_flush(SEG_DATA0_CH);
                return ret;
            }
        }
        treq = strtok(NULL, SEGCP_DELIMETER);
    }

#ifdef DBG_LEVEL_SEGCP
    PRT_SEGCP("END of [proc_SEGCP] function - RET[0x%.4x]\r\n", ret);
#endif

    return ret;
}

uint16_t proc_SEGCP_udp(uint8_t* segcp_req, uint8_t* segcp_rep) {
    DevConfig *dev_config = get_DevConfig_pointer();

    uint16_t ret = 0;
    uint16_t len = 0;
    int32_t received;

    uint8_t destip[4];
    uint16_t destport;

    uint8_t* treq;
    uint8_t* trep;
    uint8_t segcp_privilege = SEGCP_PRIVILEGE_CLR;

    switch (segcp_socket_status(SEGCP_UDP_SOCK)) {
    case SOCK_UDP:
        len = segcp_socket_rx_available(SEGCP_UDP_SOCK);
        if (len > 0) {
            treq = segcp_req;
            trep = segcp_rep;
            if (len >= CONFIG_BUF_SIZE) {
                len = CONFIG_BUF_SIZE - 1U;
            }
            received = segcp_socket_recvfrom(SEGCP_UDP_SOCK, treq, len,
                                             destip, &destport);
            if (received <= 0) {
                break;
            }
            len = (uint16_t)received;
            //reg_val = (SIK_RECEIVED) & 0x00FF;
            //ctlsocket(SEGCP_UDP_SOCK, CS_CLR_INTERRUPT, (void *)&reg_val);

            treq[len] = 0;

            if (SEGCP_MA == parse_SEGCP(treq, tpar)) {
                if (!memcmp(tpar, "\xFF\xFF\xFF\xFF\xFF\xFF", 6)) {
                    segcp_privilege |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_READ);
                } else if (!memcmp(tpar, dev_config->network_common.mac, sizeof(dev_config->network_common.mac))) {
                    segcp_privilege |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE);
                } else {
                    break;
                }

                if (segcp_privilege & SEGCP_PRIVILEGE_SET) {
                    sprintf(trep, "%s%c%c%c%c%c%c\r\n", tbSEGCPCMD[SEGCP_MA],
                            dev_config->network_common.mac[0],
                            dev_config->network_common.mac[1],
                            dev_config->network_common.mac[2],
                            dev_config->network_common.mac[3],
                            dev_config->network_common.mac[4],
                            dev_config->network_common.mac[5]);

                    treq += 10;
                    trep += 10;

                    if (SEGCP_PW == parse_SEGCP(treq, tpar)) {
                        if ((tpar[0] == SEGCP_NULL && dev_config->config_common.pw_search[0] == 0) || !strcmp(tpar, dev_config->config_common.pw_search)) {
                            memcpy(trep, treq, strlen(tpar) +4); // "PWxxxx\r\n"
                            treq += (strlen(tpar) + 4);
                            trep += (strlen(tpar) + 4);
                            ret = proc_SEGCP(treq, trep, segcp_privilege);

                            segcp_udp_reply_sent(
                                segcp_socket_sendto(SEGCP_UDP_SOCK, segcp_rep,
                                                    14 + strlen(tpar) + strlen(trep),
                                                    (uint8_t *)"\xFF\xFF\xFF\xFF",
                                                    destport));

                        }
                    }
                } else {
                    return 0;
                }
            } else {
                return 0;
            }
        }
        //            reg_val = (SIK_RECEIVED) & 0x00FF;
        //            ctlsocket(SEGCP_UDP_SOCK, CS_CLR_INTERRUPT, (void *)&reg_val);
        break;
    case SOCK_CLOSED:
        // Non-blocking like every other socket here.  ioLibrary's recvfrom()
        // offers its SOCK_BUSY exit only to a socket opened this way; without
        // it, a read that finds the receive size at zero spins forever holding
        // seg_socket_sem, which stops the data channels as well as this one.
        // The size is read and the datagram taken under two separate
        // acquisitions, so another task draining the socket in between is all
        // it takes to arrive with nothing there.
        segcp_socket_open(SEGCP_UDP_SOCK, Sn_MR_UDP, DEVICE_SEGCP_PORT,
                          SF_IO_NONBLOCK);
        break;

    default:
        // A UDP socket only reports these two states, and the cases above carry
        // it between them.  Any other reading has no way back - the socket is
        // never reopened, so the device answers no search again until it is
        // power cycled, while its data channels carry on.  Close it and let the
        // next pass reopen it.
        segcp_socket_close(SEGCP_UDP_SOCK);
        break;
    }
    return ret;
}

uint16_t proc_SEGCP_tcp(uint8_t* segcp_req, uint8_t* segcp_rep) {
    DevConfig *dev_config = get_DevConfig_pointer();

    uint16_t ret = 0;
    uint16_t len = 0;
    int32_t received;

    uint8_t * treq;
    uint8_t * trep;
    uint16_t reg_val;
    uint8_t segcp_privilege = SEGCP_PRIVILEGE_CLR;

    switch (segcp_socket_status(SEGCP_TCP_SOCK)) {
    case SOCK_INIT:
        break;

    case SOCK_LISTEN:
        //reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT) & 0x00FF; // except SIK_SENT(send OK) interrupt
        //reg_val = (SIK_CONNECTED) & 0x00FF; // except SIK_SENT(send OK) interrupt
        //ctlsocket(SEGCP_TCP_SOCK, CS_CLR_INTERRUPT, (void *)&reg_val);
        break;

    case SOCK_ESTABLISHED:
        if (segcp_socket_interrupt(SEGCP_TCP_SOCK) & Sn_IR_CON) {
            // TCP unicast search: Keep-alive timer enable
            enable_configtool_keepalive_timer = ENABLE;
            configtool_keepalive_time = 0;
            reg_val = SIK_CONNECTED & 0x00FF; // except SIK_SENT(send OK) interrupt
            segcp_socket_clear_interrupt(SEGCP_TCP_SOCK, reg_val);
        }

        if (flag_send_configtool_keepalive == SEGCP_ENABLE) { // default: 15sec
            flag_send_configtool_keepalive = SEGCP_DISABLE;    // flag clear
        }

        len = segcp_socket_rx_available(SEGCP_TCP_SOCK);
        if (len > 0) {
            treq = segcp_req;
            trep = segcp_rep;
            if (len >= CONFIG_BUF_SIZE) {
                len = CONFIG_BUF_SIZE - 1U;
            }
            received = segcp_socket_recv(SEGCP_TCP_SOCK, treq, len);
            if (received <= 0) {
                break;
            }
            len = (uint16_t)received;
            treq[len] = 0x00;
            while ((len > 0) &&
                    ((treq[len - 1] == '\r') || (treq[len - 1] == '\n'))) {
                treq[--len] = 0x00;
            }
            if (len == 0) {
                break;
            }

            if (SEGCP_MA == parse_SEGCP(treq, tpar)) {
                if (!memcmp(tpar, "\xFF\xFF\xFF\xFF\xFF\xFF", 6)) {
                    segcp_privilege |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_READ);
                } else if (!memcmp(tpar, dev_config->network_common.mac, sizeof(dev_config->network_common.mac))) {
                    segcp_privilege |= (SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE);
                } else {
                    break;
                }

                if (segcp_privilege & SEGCP_PRIVILEGE_SET) {
                    sprintf(trep, "%s%c%c%c%c%c%c\r\n", tbSEGCPCMD[SEGCP_MA],
                            dev_config->network_common.mac[0],
                            dev_config->network_common.mac[1],
                            dev_config->network_common.mac[2],
                            dev_config->network_common.mac[3],
                            dev_config->network_common.mac[4],
                            dev_config->network_common.mac[5]);

                    treq += 10;
                    trep += 10;

                    if (SEGCP_PW == parse_SEGCP(treq, tpar)) {
                        if ((tpar[0] == SEGCP_NULL && dev_config->config_common.pw_search[0] == 0) || !strcmp(tpar, dev_config->config_common.pw_search)) {
                            memcpy(trep, treq, strlen(tpar) +4); // "PWxxxx\r\n"
                            treq += (strlen(tpar) + 4);
                            trep += (strlen(tpar) + 4);
                            ret = proc_SEGCP(treq, trep, segcp_privilege);
                            segcp_socket_send(SEGCP_TCP_SOCK, segcp_rep,
                                              14 + strlen(tpar) + strlen(trep));
                        }
                    }
                } else {
                    return 0;
                }
            } else {
                return 0;
            }
        }
        break;

    case SOCK_CLOSE_WAIT:
        segcp_socket_disconnect(SEGCP_TCP_SOCK);

    case SOCK_CLOSED:
    case SOCK_FIN_WAIT:
        segcp_socket_close(SEGCP_TCP_SOCK);

        int8_t socket_rc = segcp_socket_open(SEGCP_TCP_SOCK, Sn_MR_TCP,
                                             DEVICE_SEGCP_PORT,
                                             SF_TCP_NODELAY | SF_IO_NONBLOCK);
        if (socket_rc == SEGCP_TCP_SOCK) {
            //Keep-alive timer keep disabled until TCP connection established.
            enable_configtool_keepalive_timer = DISABLE;
            segcp_socket_listen(SEGCP_TCP_SOCK);
        }
        break;
    }
    return ret;
}

uint16_t proc_SEGCP_serial(uint8_t * segcp_req, uint8_t * segcp_rep) {
    DevConfig *dev_config = get_DevConfig_pointer();

    uint16_t len = 0;
    uint16_t ret = 0;
    uint8_t segcp_privilege;

    if (get_data_buffer_usedsize(SEG_DATA0_CH)) {
        len = uart_get_commandline(segcp_req, CONFIG_BUF_SIZE);
        if (len != 0) {
            segcp_privilege = SEGCP_PRIVILEGE_SET | SEGCP_PRIVILEGE_WRITE;
            ret = proc_SEGCP(segcp_req, segcp_rep, segcp_privilege);
            if (segcp_rep[0]) {
                if (dev_config->serial_common.serial_debug_en) {
                    printf("%s", segcp_rep);
                }
                platform_uart_puts(segcp_rep, strlen((char *)segcp_rep), SEG_DATA0_CH);
            }
        }
    }

    return ret;
}

uint16_t uart_get_commandline(uint8_t* buf, uint16_t maxSize) {
    DevConfig *dev_config = get_DevConfig_pointer();

    uint16_t i;
    uint16_t len = get_data_buffer_usedsize(SEG_DATA0_CH);

    if (len >= 4) { // Minimum of command: 4-bytes, e.g., MC\r\n (MC$0d$0a)
        memset(buf, 0, CONFIG_BUF_SIZE);
        for (i = 0; i < maxSize; i++) {
            buf[i] = data_buffer_getc(SEG_DATA0_CH);
            if (buf[i] == 0x0a) {
                break;    // [0x0a]: end of command (Line feed)
            }
        }

        if ((!(memcmp(buf, "OC", SEGCP_CMD_MAX))) || (!(memcmp(buf, "LC", SEGCP_CMD_MAX)))) {
            for (i++; i < maxSize; i++) {
                buf[i] = data_buffer_getc(SEG_DATA0_CH);
                if (strstr(buf, END_CERT)) {
                    vTaskDelay(10);
                    data_buffer_flush(SEG_DATA0_CH);
                    break;
                }
            }
        } else if (!(memcmp(buf, "PK", SEGCP_CMD_MAX))) {
            for (i++; i < maxSize; i++) {
                buf[i] = data_buffer_getc(SEG_DATA0_CH);
                if (strstr(buf, END_PKEY)) {
                    delay_ms(10);
                    data_buffer_flush(SEG_DATA0_CH);
                    break;
                }
            }
        }
        buf[i + 1] = 0x00; // end of string
        if (dev_config->serial_command.serial_command_echo == SEGCP_ENABLE) {
            platform_uart_puts(buf, i, SEG_DATA0_CH);
        }
    } else {
        return 0;
    }
    return (uint16_t)strlen(buf);
}

// Function for Timer
void segcp_timer_msec(void) {
    if (enable_configtool_keepalive_timer) {
        if (configtool_keepalive_time < 0xFFFF) {
            configtool_keepalive_time++;
        } else {
            configtool_keepalive_time = 0;
        }

        if (configtool_keepalive_time >= CONFIGTOOL_KEEPALIVE_TIME_MS) {
            flag_send_configtool_keepalive = SEGCP_ENABLE;
            configtool_keepalive_time = 0;
        }
    }
}

void segcp_udp_task(void *argument) {

    while (1) {
        if (get_net_status() == NET_LINK_DISCONNECTED) {
            xSemaphoreTake(net_segcp_udp_sem, portMAX_DELAY);
        }
        do_segcp_udp();
        vTaskDelay(200);
    }
}

void segcp_tcp_task(void *argument) {

    while (1) {
        if (get_net_status() == NET_LINK_DISCONNECTED) {
            xSemaphoreTake(net_segcp_tcp_sem, portMAX_DELAY);
        }
        do_segcp_tcp();
        vTaskDelay(200);
    }
}

void segcp_serial_task(void *argument) {

    while (1) {
        xSemaphoreTake(segcp_uart_sem, portMAX_DELAY);

        // Serial AT command mode enabled, initial settings
        if ((opmode == DEVICE_GW_MODE) && (sw_modeswitch_at_mode_on == SEG_ENABLE)) {
            // Mode switch
            init_trigger_modeswitch(DEVICE_AT_MODE);

            // AT mode is device-wide.  Close all four data sockets so DATA2/3
            // cannot remain established while their SEG tasks are suspended.
            for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
                process_socket_termination(seg_data_sock[ch],
                                           SOCK_TERMINATION_DELAY, ch, TRUE);
            }

            // Mode switch flag disabled
            sw_modeswitch_at_mode_on = SEG_DISABLE;
        } else {
            do_segcp_serial();
        }
        //vTaskDelay(10);
    }
}
