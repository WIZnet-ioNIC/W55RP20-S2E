
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "ConfigData.h"
#include "storageHandler.h"
#include "deviceHandler.h"
#include "segcp.h"
#include "uartHandler.h"
#include "netHandler.h"
#include "flashHandler.h"

#include "httpParser.h"
#include "httpHandler.h"
#include "Web_page.h"
#include "socket.h"
#include "httpServer.h"
#include "util.h"
#include "SSLInterface.h"

extern wiz_tls_context s2e_tlsContext;

extern xSemaphoreHandle net_http_webserver_sem;
extern TimerHandle_t reset_timer;

extern uint8_t gSEGCPREQ[];
extern uint8_t gSEGCPREP[];

extern uint8_t *pHTTP_RX;
extern uint8_t *pHTTP_TX;

void make_json_devinfo(uint8_t * buf, uint16_t * len) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t uart_sel = 0;
    uint8_t baudrate_index[2] = {0, };
    uint8_t databit_index[2] = {0, };
    uint8_t stopbit_index[2] = {0, };
    char rip_str[DNS_DOMAIN_SIZE];

    // for UART0 / UART1, uart_sel = 0 or 1
    if (uart_sel > 1) {
        uart_sel = 0;
    }

    /* Prefer domain name if set, otherwise format IP */
    if (dev_config->network_connection.dns_domain_name[0] != 0) {
        strncpy(rip_str, dev_config->network_connection.dns_domain_name, DNS_DOMAIN_SIZE - 1);
        rip_str[DNS_DOMAIN_SIZE - 1] = '\0';
    } else {
        snprintf(rip_str, sizeof(rip_str), "%d.%d.%d.%d",
                 dev_config->network_connection.remote_ip[0],
                 dev_config->network_connection.remote_ip[1],
                 dev_config->network_connection.remote_ip[2],
                 dev_config->network_connection.remote_ip[3]);
    }

    baudrate_index[uart_sel] = dev_config->serial_option.baud_rate;
    databit_index[uart_sel] = dev_config->serial_option.data_bits;
    stopbit_index[uart_sel] = dev_config->serial_option.stop_bits;


    *len = sprintf((char *)buf, "DevinfoCallback({\"fwver\":\"%d.%d.%d_%s\","\
                   "\"devname\":\"%s\","\
                   "\"pcode\":\"%d-%d-%d\","\
                   "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","\
                   "\"ip\":\"%d.%d.%d.%d\","\
                   "\"gw\":\"%d.%d.%d.%d\","\
                   "\"sub\":\"%d.%d.%d.%d\","\
                   "\"dns\":\"%d.%d.%d.%d\","\
                   "\"dhcp\":\"%d\","\
                   "\"opmode\":\"%d\","\
                   "\"lport\":\"%d\","\
                   "\"rip\":\"%s\","\
                   "\"rport\":\"%d\","\
                   "\"modbus\":\"%d\","\
                   "\"uart\":\"%d\","\
                   "\"baud\":\"%d\","\
                   "\"databit\":\"%d\","\
                   "\"parity\":\"%d\","\
                   "\"stopbit\":\"%d\","\
                   "\"flow\":\"%d\","
                   "\"rootca_opt\":\"%d\","
                   "\"clicert_en\":\"%d\""
                   "});",
                   dev_config->device_common.fw_ver[0], dev_config->device_common.fw_ver[1], dev_config->device_common.fw_ver[2], STR_VERSION_STATUS,
                   dev_config->device_common.device_name,
                   dev_config->device_common.device_type[0], dev_config->device_common.device_type[1], dev_config->device_common.device_type[2],
                   dev_config->network_common.mac[0], dev_config->network_common.mac[1], dev_config->network_common.mac[2], dev_config->network_common.mac[3], dev_config->network_common.mac[4], dev_config->network_common.mac[5],
                   dev_config->network_common.local_ip[0], dev_config->network_common.local_ip[1], dev_config->network_common.local_ip[2], dev_config->network_common.local_ip[3],
                   dev_config->network_common.gateway[0], dev_config->network_common.gateway[1], dev_config->network_common.gateway[2], dev_config->network_common.gateway[3],
                   dev_config->network_common.subnet[0], dev_config->network_common.subnet[1], dev_config->network_common.subnet[2], dev_config->network_common.subnet[3],
                   dev_config->network_connection.dns_domain_name[0], dev_config->network_connection.dns_domain_name[1], dev_config->network_connection.dns_domain_name[2], dev_config->network_connection.dns_domain_name[3],
                   dev_config->network_option.dhcp_use,
                   dev_config->network_connection.working_mode,
                   dev_config->network_connection.local_port,
                   rip_str,
                   dev_config->network_connection.remote_port,
                   dev_config->serial_option.protocol,
                   uart_sel,
                   baudrate_index[uart_sel],
                   databit_index[uart_sel],
                   dev_config->serial_option.parity,
                   stopbit_index[uart_sel],
                   dev_config->serial_option.flow_control,
                   dev_config->ssl_option.root_ca_option,
                   dev_config->ssl_option.client_cert_enable
                  );
}

uint8_t set_devinfo(uint8_t * uri) {
    uint8_t ret = 0;
    uint8_t * param;
    uint8_t str_size;
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t *temp_buf;

    temp_buf = pvPortMalloc(256);
    memset(temp_buf, 0x00, 256);

    if ((param = get_http_param_value((char *)uri, "devname", temp_buf))) {
        memset(dev_config->device_common.device_name, 0x00, DEVICE_NAME_SIZE);
        if ((str_size = strlen((char *)param)) > (DEVICE_NAME_SIZE - 1)) {
            str_size = DEVICE_NAME_SIZE - 1;    // exception handling
        }
        memcpy(dev_config->device_common.device_name, param, str_size);
        ret = 1;
    }

    if ((param = get_http_param_value((char *)uri, "dhcp", temp_buf))) {
        if (strstr((char const *)param, "1") != NULL) {
            dev_config->network_option.dhcp_use = 1;    // DHCP mode
        } else {
            dev_config->network_option.dhcp_use = 0;    // Static mode
        }
        ret = 1;
    }

    if (dev_config->network_option.dhcp_use == 0) { // Static mode
        if ((param = get_http_param_value((char *)uri, "ip", temp_buf))) {
            inet_addr_((unsigned char*)param, dev_config->network_common.local_ip);
            ret = 1;
        }
        if ((param = get_http_param_value((char *)uri, "gw", temp_buf))) {
            inet_addr_((unsigned char*)param, dev_config->network_common.gateway);
            ret = 1;
        }
        if ((param = get_http_param_value((char *)uri, "sub", temp_buf))) {
            inet_addr_((unsigned char*)param, dev_config->network_common.subnet);
            ret = 1;
        }
        if ((param = get_http_param_value((char *)uri, "dns", temp_buf))) {
            inet_addr_((unsigned char*)param, dev_config->network_connection.dns_domain_name);
            ret = 1;
        }
    }

    if ((param = get_http_param_value((char *)uri, "opmode", temp_buf))) {
        dev_config->network_connection.working_mode = ATOI(param, 10);
#ifndef __USE_S2E_OVER_TLS__
        if (dev_config->network_connection.working_mode == SSL_TCP_CLIENT_MODE
                || dev_config->network_connection.working_mode == SSL_TCP_SERVER_MODE
                || dev_config->network_connection.working_mode == MQTTS_CLIENT_MODE) {
            dev_config->network_connection.working_mode = TCP_SERVER_MODE;
        }
#endif
    }

    if ((param = get_http_param_value((char *)uri, "lport", temp_buf))) {
        dev_config->network_connection.local_port = ATOI(param, 10);
        ret = 1;
    }

    if ((dev_config->network_connection.working_mode != TCP_SERVER_MODE)
            && (dev_config->network_connection.working_mode != SSL_TCP_SERVER_MODE)) {
        if ((param = get_http_param_value((char *)uri, "rip", temp_buf))) {
            uint8_t tmp_ip[4];
            if (is_ipaddr(param, tmp_ip)) {
                /* Numeric IP entered */
                dev_config->network_connection.dns_use = 0;
                memcpy(dev_config->network_connection.remote_ip, tmp_ip, 4);
                strncpy(dev_config->network_connection.dns_domain_name,
                        (char *)param, DNS_DOMAIN_SIZE - 1);
                dev_config->network_connection.dns_domain_name[DNS_DOMAIN_SIZE - 1] = '\0';
            } else {
                /* Hostname/domain entered */
                dev_config->network_connection.dns_use = 1;
                memset(dev_config->network_connection.remote_ip, 0, 4);
                strncpy(dev_config->network_connection.dns_domain_name,
                        (char *)param, DNS_DOMAIN_SIZE - 1);
                dev_config->network_connection.dns_domain_name[DNS_DOMAIN_SIZE - 1] = '\0';
            }
            ret = 1;
        }

        if ((param = get_http_param_value((char *)uri, "rport", temp_buf))) {
            dev_config->network_connection.remote_port = ATOI(param, 10);
            ret = 1;
        }
    }
    if ((param = get_http_param_value((char *)uri, "modbus", temp_buf))) {
        dev_config->serial_option.protocol = ATOI(param, 10);
        ret = 1;
    }


    if ((param = get_http_param_value((char *)uri, "baud", temp_buf))) {
        uint8_t baudrate_idx = ATOI(param, 10);
#if (DEVICE_BOARD_NAME == W232N)
        if (baudrate_idx > baud_230400) {
            baudrate_idx = baud_115200;
        }
#else
        if (baudrate_idx > baud_max) {
            baudrate_idx = baud_115200;
        }
#endif
        dev_config->serial_option.baud_rate = baudrate_idx;
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "databit", temp_buf))) {
        dev_config->serial_option.data_bits = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "parity", temp_buf))) {
        dev_config->serial_option.parity = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "stopbit", temp_buf))) {
        dev_config->serial_option.stop_bits = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "flow", temp_buf))) {
        dev_config->serial_option.flow_control = ATOI(param, 10);
        ret = 1;
    }
#ifdef __USE_S2E_OVER_TLS__
    if ((param = get_http_param_value((char *)uri, "rootca_opt", temp_buf))) {
        uint8_t val = ATOI(param, 10);
        if (val <= 2) {
            dev_config->ssl_option.root_ca_option = val;
            ret = 1;
        }
    }
    if ((param = get_http_param_value((char *)uri, "clicert_en", temp_buf))) {
        dev_config->ssl_option.client_cert_enable = ATOI(param, 10) ? 1 : 0;
        ret = 1;
    }
#endif
    vPortFree(temp_buf);

    if (ret == 1) {
        save_DevConfig_to_storage();
    }
    return ret;
}


uint8_t set_devreset(uint8_t * uri) {
    uint8_t ret = 0;

    ret = 1;
    return ret;
}

uint8_t set_devfacreset(uint8_t * uri) {
    uint8_t ret = 0;

    device_set_factory_default();

    ret = 1;
    return ret;
}

uint8_t update_module_firmware(st_http_request * p_http_request, uint8_t *buf) {
    uint8_t ret = 0, sock, end_flag = 0;
    uint8_t *body, *boundary_pos;
    uint8_t boundary[128];
    uint8_t boundary_len;
    uint16_t body_len;
    uint32_t f_addr = FLASH_START_ADDR_BANK1_OFFSET;
    uint32_t remain_len = 0, buf_len = 0;
    struct __firmware_update *fwupdate = (struct __firmware_update *) & (get_DevConfig_pointer()->firmware_update);
    uint8_t *temp_buf;

    sock = p_http_request->socket;
    /* strtok in parse_http_request sets pHTTP_RX[4]='\0'; search from +5 */
    boundary_pos = strstr((char *)pHTTP_RX + 5, "boundary=");
    if (boundary_pos == NULL) {
        return 0;
    }
    sscanf(boundary_pos, "boundary=%127s", boundary);
    boundary_len = strlen(boundary);
    PRT_HTTP("boundary = %s, boundary_len = %d\r\n", boundary, boundary_len);

    body = strstr((char*)pHTTP_RX + p_http_request->header_len + 4, "\r\n\r\n");
    if (body != NULL) {
        body += 4;
        body_len = p_http_request->recv_len - (body - pHTTP_RX);

        temp_buf = pvPortMalloc(FLASH_SECTOR_SIZE);
        memset(temp_buf, 0x00, FLASH_SECTOR_SIZE);

        memcpy(temp_buf, body, body_len);
        buf_len = body_len;

        xTimerStart(reset_timer, 0);
        while (1) {
            if ((body_len = getSn_RX_RSR(sock)) > 0) {
                device_wdt_reset();
                xTimerReset(reset_timer, 0);
                memset(pHTTP_RX, 0x00, CONFIG_BUF_SIZE);
                body_len = recv(sock, (uint8_t *)pHTTP_RX, body_len);

                if (!(memcmp(pHTTP_RX + (body_len - boundary_len - 4), boundary, boundary_len))) { //end boundary = "\r\n--boundary--\r\n"
                    PRT_HTTP("find end boundary\r\n");
                    memset(pHTTP_RX + (body_len - boundary_len - 8), 0xFF, boundary_len + 8);
                    end_flag = 1;
                }

                if (buf_len + body_len < FLASH_SECTOR_SIZE) {
                    memcpy(temp_buf + buf_len, pHTTP_RX, body_len);
                    buf_len += body_len;
                } else {
                    if (f_addr >= FLASH_START_ADDR_BANK1_OFFSET + FLASH_APP_BANK_SIZE) {
                        vPortFree(temp_buf);
                        return 0;
                    }

                    remain_len = (buf_len + body_len) - FLASH_SECTOR_SIZE;
                    memcpy(temp_buf + buf_len, pHTTP_RX, body_len - remain_len);
                    write_flash(f_addr, (uint8_t *)temp_buf, FLASH_SECTOR_SIZE);
                    f_addr += FLASH_SECTOR_SIZE;

                    memset(temp_buf, 0xFF, FLASH_SECTOR_SIZE);
                    memcpy(temp_buf, pHTTP_RX + (body_len - remain_len), remain_len);
                    buf_len = remain_len;
                }
                if (end_flag) {
                    break;
                }
            }
        }

        xTimerStop(reset_timer, 0);
        if (buf_len > 0) {
            write_flash(f_addr, temp_buf, FLASH_SECTOR_SIZE);
        }

        vPortFree(temp_buf);
        fwupdate->fwup_copy_flag = 1;
        remain_len = FLASH_START_ADDR_BANK1_OFFSET;
        fwupdate->fwup_size = (f_addr - remain_len) + buf_len;
        PRT_HTTP("Download Finished fwup_size = %d\r\n", fwupdate->fwup_size);
        save_DevConfig_to_storage();
    }
    ret = 1;
    return ret;
}


#ifdef __USE_S2E_OVER_TLS__
/* Validator signature: returns 0 on valid, <0 on invalid */
typedef int (*ssl_cert_validator_t)(uint8_t *data, uint32_t len);

static int validate_ca(uint8_t *data, uint32_t len) {
    return check_ca(data, len);
}

static int validate_pkey(uint8_t *data, uint32_t len) {
    return check_pkey(&s2e_tlsContext, data, len);
}

static uint8_t upload_ssl_cert(st_http_request *p_http_request, uint32_t flash_addr, uint32_t *cert_len_out, ssl_cert_validator_t validator) {
    uint8_t ret = 0, sock, end_flag = 0;
    uint8_t *body, *boundary_pos;
    uint8_t boundary[128];
    uint8_t boundary_len;
    uint16_t body_len;
    uint32_t buf_len = 0;
    uint8_t *temp_buf;

    sock = p_http_request->socket;
    /*  Search for boundary in raw HTTP headers (Content-Type header)
        Note: strtok() in parse_http_request() sets pHTTP_RX[4]='\0' (replaces
        the space after "POST"), so we must start searching from pHTTP_RX+5. */
    boundary_pos = strstr((char *)pHTTP_RX + 5, "boundary=");
    if (boundary_pos == NULL) {
        return 0;
    }
    sscanf(boundary_pos, "boundary=%127s", boundary);
    boundary_len = strlen(boundary);

    /* Skip HTTP headers, then skip multipart part headers to reach file data */
    body = strstr((char*)pHTTP_RX + p_http_request->header_len + 4, "\r\n\r\n");
    if (body == NULL) {
        return 0;
    }
    body += 4; /* skip \r\n\r\n after multipart part headers */
    body_len = p_http_request->recv_len - (body - pHTTP_RX);

    temp_buf = pvPortMalloc(FLASH_SECTOR_SIZE);
    if (temp_buf == NULL) {
        return 0;
    }
    memset(temp_buf, 0xFF, FLASH_SECTOR_SIZE);

    /* Copy initial body chunk to temp_buf */
    if (body_len <= FLASH_SECTOR_SIZE) {
        memcpy(temp_buf, body, body_len);
        buf_len = body_len;
    } else {
        vPortFree(temp_buf);
        return 0;
    }
    /* Check for end boundary in accumulated data */
    if (buf_len >= (uint16_t)(boundary_len + 4)) {
        if (!(memcmp(temp_buf + buf_len - boundary_len - 4, boundary, boundary_len))) {
            buf_len -= (boundary_len + 8);
            end_flag = 1;
        }
    }

    xTimerStart(reset_timer, 0);
    while (!end_flag) {
        if ((body_len = getSn_RX_RSR(sock)) > 0) {
            device_wdt_reset();
            xTimerReset(reset_timer, 0);
            memset(pHTTP_RX, 0x00, CONFIG_BUF_SIZE);
            body_len = recv(sock, (uint8_t *)pHTTP_RX, body_len);
            if (buf_len + body_len <= FLASH_SECTOR_SIZE) {
                memcpy(temp_buf + buf_len, pHTTP_RX, body_len);
                buf_len += body_len;
            } else {
                xTimerStop(reset_timer, 0);
                vPortFree(temp_buf);
                return 0; /* Cert exceeds one flash sector */
            }
            /* Check for end boundary in accumulated data (handles boundary split across recv calls) */
            if (buf_len >= (uint16_t)(boundary_len + 4)) {
                if (!(memcmp(temp_buf + buf_len - boundary_len - 4, boundary, boundary_len))) {
                    buf_len -= (boundary_len + 8);
                    end_flag = 1;
                }
            }
            if (end_flag) {
                break;
            }
        }
    }
    xTimerStop(reset_timer, 0);

    /* Ensure space for NUL terminator */
    if (buf_len >= FLASH_SECTOR_SIZE) {
        vPortFree(temp_buf);
        return 0;
    }
    temp_buf[buf_len] = 0;

    /* Validate the cert/key BEFORE writing to flash */
    if (validator != NULL) {
        if (validator(temp_buf, buf_len) < 0) {
            vPortFree(temp_buf);
            return 0;
        }
    }

    write_flash(flash_addr, temp_buf, FLASH_SECTOR_SIZE);
    *cert_len_out = buf_len;
    vPortFree(temp_buf);
    ret = 1;
    return ret;
}

uint8_t update_ssl_rootca(st_http_request *p_http_request, uint8_t *buf) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint32_t cert_len = 0;
    if (!upload_ssl_cert(p_http_request, FLASH_ROOTCA_ADDR, &cert_len, validate_ca)) {
        return 0;
    }
    dev_config->ssl_option.rootca_len = cert_len;
    save_DevConfig_to_storage();
    return 1;
}

uint8_t update_ssl_clica(st_http_request *p_http_request, uint8_t *buf) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint32_t cert_len = 0;
    if (!upload_ssl_cert(p_http_request, FLASH_CLICA_ADDR, &cert_len, validate_ca)) {
        return 0;
    }
    dev_config->ssl_option.clica_len = cert_len;
    save_DevConfig_to_storage();
    return 1;
}

uint8_t update_ssl_prikey(st_http_request *p_http_request, uint8_t *buf) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint32_t cert_len = 0;
    if (!upload_ssl_cert(p_http_request, FLASH_PRIKEY_ADDR, &cert_len, validate_pkey)) {
        return 0;
    }
    dev_config->ssl_option.pkey_len = cert_len;
    save_DevConfig_to_storage();
    return 1;
}
#endif /* __USE_S2E_OVER_TLS__ */


void http_webserver_task(void *argument)  {
    const uint8_t socknumlist[MAX_HTTPSOCK] = {SOCK_HTTPSERVER_1, SOCK_HTTPSERVER_2, SOCK_HTTPSERVER_3, SOCK_HTTPSERVER_4};
    uint8_t i;

    httpServer_init(gSEGCPREQ, gSEGCPREP, MAX_HTTPSOCK, socknumlist);
    reg_httpServer_cbfunc(device_raw_reboot, NULL);
    reg_httpServer_webContent("index.html", _acWeb_page);

    while (1) {

        if (get_net_status() == NET_LINK_DISCONNECTED) {
            PRT_SEGCP("get_net_status() != NET_LINK_DISCONNECTED\r\n");
            xSemaphoreTake(net_http_webserver_sem, portMAX_DELAY);
            PRT_SEGCP("xSemaphoreTake(net_http_webserver_sem, portMAX_DELAY)\r\n");
        }
        for (i = 0; i < MAX_HTTPSOCK; i++) {
            httpServer_run(i);
        }
        vTaskDelay(200);
    }
}



