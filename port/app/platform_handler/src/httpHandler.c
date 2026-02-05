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

extern xSemaphoreHandle net_http_webserver_sem;
extern TimerHandle_t reset_timer;

extern uint8_t gSEGCPREQ[];
extern uint8_t gSEGCPREP[];

extern uint8_t *pHTTP_RX;
extern uint8_t *pHTTP_TX;

void make_json_devinfo(uint8_t * buf, uint16_t * len) {
    DevConfig *dev_config = get_DevConfig_pointer();

    *len = sprintf((char *)buf, "DevinfoCallback({\"fwver\":\"%d.%d.%d_%s\","\
                   "\"devname\":\"%s\","\
                   "\"pcode\":\"%d-%d-%d\","\
                   "\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","\
                   "\"ip\":\"%d.%d.%d.%d\","\
                   "\"gw\":\"%d.%d.%d.%d\","\
                   "\"sub\":\"%d.%d.%d.%d\","\
                   "\"dns\":\"%d.%d.%d.%d\","\
                   "\"dhcp\":\"%d\","\
                   "\"opmode0\":\"%d\","\
                   "\"lport0\":\"%d\","\
                   "\"rip0\":\"%d.%d.%d.%d\","\
                   "\"rport0\":\"%d\","\
                   "\"sslRecvTimeout0\":\"%d\","\
                   "\"modbus0\":\"%d\","\
                   "\"baud0\":\"%d\","\
                   "\"databit0\":\"%d\","\
                   "\"parity0\":\"%d\","\
                   "\"stopbit0\":\"%d\","\
                   "\"flow0\":\"%d\","\
                   "\"packChar0\":\"%02X\","\
                   "\"packSize0\":\"%d\","\
                   "\"packTimer0\":\"%d\","\
                   "\"inTimer0\":\"%d\","\
                   "\"reConInterval0\":\"%d\","\
                   "\"keepAliveEn0\":\"%d\","\
                   "\"keepAliveInitInterval0\":\"%d\","\
                   "\"keepAliveRetryInterval0\":\"%d\","\
                   "\"opmode1\":\"%d\","\
                   "\"lport1\":\"%d\","\
                   "\"rip1\":\"%d.%d.%d.%d\","\
                   "\"rport1\":\"%d\","\
                   "\"sslRecvTimeout1\":\"%d\","\
                   "\"modbus1\":\"%d\","\
                   "\"baud1\":\"%d\","\
                   "\"databit1\":\"%d\","\
                   "\"parity1\":\"%d\","\
                   "\"stopbit1\":\"%d\","\
                   "\"flow1\":\"%d\","\
                   "\"packChar1\":\"%02X\","\
                   "\"packSize1\":\"%d\","\
                   "\"packTimer1\":\"%d\","\
                   "\"inTimer1\":\"%d\","\
                   "\"reConInterval1\":\"%d\","\
                   "\"keepAliveEn1\":\"%d\","\
                   "\"keepAliveInitInterval1\":\"%d\","\
                   "\"keepAliveRetryInterval1\":\"%d\""\
                   "});",
                   dev_config->device_common.fw_ver[0], dev_config->device_common.fw_ver[1], dev_config->device_common.fw_ver[2], STR_VERSION_STATUS,
                   dev_config->device_common.device_name,
                   dev_config->device_common.device_type[0], dev_config->device_common.device_type[1], dev_config->device_common.device_type[2],
                   dev_config->network_common.mac[0], dev_config->network_common.mac[1], dev_config->network_common.mac[2], dev_config->network_common.mac[3], dev_config->network_common.mac[4], dev_config->network_common.mac[5],
                   dev_config->network_common.local_ip[0], dev_config->network_common.local_ip[1], dev_config->network_common.local_ip[2], dev_config->network_common.local_ip[3],
                   dev_config->network_common.gateway[0], dev_config->network_common.gateway[1], dev_config->network_common.gateway[2], dev_config->network_common.gateway[3],
                   dev_config->network_common.subnet[0], dev_config->network_common.subnet[1], dev_config->network_common.subnet[2], dev_config->network_common.subnet[3],
                   dev_config->network_connection[0].dns_domain_name[0], dev_config->network_connection[0].dns_domain_name[1], dev_config->network_connection[0].dns_domain_name[2], dev_config->network_connection[0].dns_domain_name[3],
                   dev_config->network_option.dhcp_use,
                   dev_config->network_connection[0].working_mode, //ch0 network
                   dev_config->network_connection[0].local_port,
                   dev_config->network_connection[0].remote_ip[0], dev_config->network_connection[0].remote_ip[1], dev_config->network_connection[0].remote_ip[2], dev_config->network_connection[0].remote_ip[3],
                   dev_config->network_connection[0].remote_port,
                   dev_config->ssl_option[0].recv_timeout,
                   dev_config->serial_option[0].protocol,
                   dev_config->serial_option[0].baud_rate,     //ch0 serial options
                   dev_config->serial_option[0].data_bits,
                   dev_config->serial_option[0].parity,
                   dev_config->serial_option[0].stop_bits,
                   dev_config->serial_option[0].flow_control,
                   dev_config->serial_data_packing[0].packing_delimiter[0],  //ch0 serial data packing
                   dev_config->serial_data_packing[0].packing_size,
                   dev_config->serial_data_packing[0].packing_time,
                   dev_config->tcp_option[0].inactivity,      //ch0 Timer interval
                   dev_config->tcp_option[0].reconnection,
                   dev_config->tcp_option[0].keepalive_en,     //ch0 keepalive
                   dev_config->tcp_option[0].keepalive_wait_time,
                   dev_config->tcp_option[0].keepalive_retry_time,
                   dev_config->network_connection[1].working_mode, //ch1
                   dev_config->network_connection[1].local_port,
                   dev_config->network_connection[1].remote_ip[0], dev_config->network_connection[1].remote_ip[1], dev_config->network_connection[1].remote_ip[2], dev_config->network_connection[1].remote_ip[3],
                   dev_config->network_connection[1].remote_port,
                   dev_config->ssl_option[1].recv_timeout,
                   dev_config->serial_option[1].protocol,
                   dev_config->serial_option[1].baud_rate,
                   dev_config->serial_option[1].data_bits,
                   dev_config->serial_option[1].parity,
                   dev_config->serial_option[1].stop_bits,
                   dev_config->serial_option[1].flow_control,
                   dev_config->serial_data_packing[1].packing_delimiter[0],  //ch1 serial data packing
                   dev_config->serial_data_packing[1].packing_size,
                   dev_config->serial_data_packing[1].packing_time,
                   dev_config->tcp_option[1].inactivity,      //ch1 Timer interval
                   dev_config->tcp_option[1].reconnection,
                   dev_config->tcp_option[1].keepalive_en,     //ch1 keepalive
                   dev_config->tcp_option[1].keepalive_wait_time,
                   dev_config->tcp_option[1].keepalive_retry_time
                  );
}

uint8_t set_devinfo(uint8_t * uri) {
    uint8_t ret = 0;
    uint8_t * param;
    uint8_t str_size;
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t *temp_buf;
    uint8_t  temp_byte = 0;

    temp_buf = pvPortMalloc(256);
    memset(temp_buf, 0x00, 256);

    if ((param = get_http_param_value((char *)uri, "devname", temp_buf))) {
        memset(dev_config->device_common.device_name, 0x00, DEVICE_NAME_SIZE);
        if ((str_size = strlen((char *)param)) > DEVICE_NAME_SIZE - 1) {
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
            inet_addr_((unsigned char*)param, dev_config->network_connection[0].dns_domain_name);
            ret = 1;
        }
    }

    if ((param = get_http_param_value((char *)uri, "opmode0", temp_buf))) {
        dev_config->network_connection[0].working_mode = ATOI(param, 10);
    }

    if ((param = get_http_param_value((char *)uri, "lport0", temp_buf))) {
        dev_config->network_connection[0].local_port = ATOI(param, 10);
        ret = 1;
    }

    if (dev_config->network_connection[0].working_mode != TCP_SERVER_MODE) {
        if ((param = get_http_param_value((char *)uri, "rip0", temp_buf))) {
            inet_addr_((unsigned char*)param, dev_config->network_connection[0].remote_ip);
            ret = 1;
        }

        if ((param = get_http_param_value((char *)uri, "rport0", temp_buf))) {
            dev_config->network_connection[0].remote_port = ATOI(param, 10);
            ret = 1;
        }
    }

    if ((param = get_http_param_value((char *)uri, "sslRecvTimeout0", temp_buf))) {
        dev_config->ssl_option[0].recv_timeout = ATOI(param, 10);
        ret = 1;
    }

    if ((param = get_http_param_value((char *)uri, "modbus0", temp_buf))) {
        dev_config->serial_option[0].protocol = ATOI(param, 10);
        ret = 1;
    }

    if ((param = get_http_param_value((char *)uri, "baud0", temp_buf))) {
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
        dev_config->serial_option[0].baud_rate = baudrate_idx;
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "databit0", temp_buf))) {
        dev_config->serial_option[0].data_bits = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "parity0", temp_buf))) {
        dev_config->serial_option[0].parity = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "stopbit0", temp_buf))) {
        dev_config->serial_option[0].stop_bits = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "flow0", temp_buf))) {
        dev_config->serial_option[0].flow_control = ATOI(param, 10);
        ret = 1;
    }

    if ((param = get_http_param_value((char *)uri, "packChar0", temp_buf))) {
        str_to_hex(temp_buf, &temp_byte);

        dev_config->serial_data_packing[0].packing_delimiter[0] = temp_byte;
        if (dev_config->serial_data_packing[0].packing_delimiter[0] == 0x00) {
            dev_config->serial_data_packing[0].packing_delimiter_length = 0;
        } else {
            dev_config->serial_data_packing[0].packing_delimiter_length = 1;
        }
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "packSize0", temp_buf))) {
        dev_config->serial_data_packing[0].packing_size = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "packTimer0", temp_buf))) {
        dev_config->serial_data_packing[0].packing_time = ATOI(param, 10);
    }

    if ((param = get_http_param_value((char *)uri, "inTimer0", temp_buf))) {
        dev_config->tcp_option[0].inactivity = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "reConInterval0", temp_buf))) {
        dev_config->tcp_option[0].reconnection = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "keepAliveEn0", temp_buf))) {
        temp_byte = is_hex(*param);
        dev_config->tcp_option[0].keepalive_en = temp_byte;
    }
    if ((param = get_http_param_value((char *)uri, "keepAliveInitInterval0", temp_buf))) {
        dev_config->tcp_option[0].keepalive_wait_time = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "keepAliveRetryInterval0", temp_buf))) {
        dev_config->tcp_option[0].keepalive_retry_time = ATOI(param, 10);
    }

    if ((param = get_http_param_value((char *)uri, "opmode1", temp_buf))) {
        dev_config->network_connection[1].working_mode = ATOI(param, 10);
    }

    if ((param = get_http_param_value((char *)uri, "lport1", temp_buf))) {
        dev_config->network_connection[1].local_port = ATOI(param, 10);
        ret = 1;
    }

    if (dev_config->network_connection[1].working_mode != TCP_SERVER_MODE) {
        if ((param = get_http_param_value((char *)uri, "rip1", temp_buf))) {
            inet_addr_((unsigned char*)param, dev_config->network_connection[1].remote_ip);
            ret = 1;
        }

        if ((param = get_http_param_value((char *)uri, "rport1", temp_buf))) {
            dev_config->network_connection[1].remote_port = ATOI(param, 10);
            ret = 1;
        }
    }

    if ((param = get_http_param_value((char *)uri, "sslRecvTimeout1", temp_buf))) {
        dev_config->ssl_option[1].recv_timeout = ATOI(param, 10);
        ret = 1;
    }

    if ((param = get_http_param_value((char *)uri, "modbus1", temp_buf))) {
        dev_config->serial_option[1].protocol = ATOI(param, 10);
        ret = 1;
    }

    if ((param = get_http_param_value((char *)uri, "baud1", temp_buf))) {
        uint8_t baudrate_idx = ATOI(param, 10);
#if (DEVICE_BOARD_NAME == W232N)
        if (baudrate_idx > baud_230400) {
            baudrate_idx = baud_115200;
        }
#else
        if (baudrate_idx >= baud_max) {
            baudrate_idx = baud_115200;
        }
#endif
        dev_config->serial_option[1].baud_rate = baudrate_idx;
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "databit1", temp_buf))) {
        dev_config->serial_option[1].data_bits = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "parity1", temp_buf))) {
        dev_config->serial_option[1].parity = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "stopbit1", temp_buf))) {
        dev_config->serial_option[1].stop_bits = ATOI(param, 10);
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "flow1", temp_buf))) {
        dev_config->serial_option[1].flow_control = ATOI(param, 10);
        ret = 1;
    }

    if ((param = get_http_param_value((char *)uri, "packChar1", temp_buf))) {
        str_to_hex(temp_buf, &temp_byte);

        dev_config->serial_data_packing[1].packing_delimiter[0] = temp_byte;
        if (dev_config->serial_data_packing[1].packing_delimiter[0] == 0x00) {
            dev_config->serial_data_packing[1].packing_delimiter_length = 0;
        } else {
            dev_config->serial_data_packing[1].packing_delimiter_length = 1;
        }
        ret = 1;
    }
    if ((param = get_http_param_value((char *)uri, "packSize1", temp_buf))) {
        dev_config->serial_data_packing[1].packing_size = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "packTimer1", temp_buf))) {
        dev_config->serial_data_packing[1].packing_time = ATOI(param, 10);
    }

    if ((param = get_http_param_value((char *)uri, "inTimer1", temp_buf))) {
        dev_config->tcp_option[1].inactivity = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "reConInterval1", temp_buf))) {
        dev_config->tcp_option[1].reconnection = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "keepAliveEn1", temp_buf))) {
        temp_byte = is_hex(*param);
        dev_config->tcp_option[1].keepalive_en = temp_byte;
    }
    if ((param = get_http_param_value((char *)uri, "keepAliveInitInterval1", temp_buf))) {
        dev_config->tcp_option[1].keepalive_wait_time = ATOI(param, 10);
    }
    if ((param = get_http_param_value((char *)uri, "keepAliveRetryInterval1", temp_buf))) {
        dev_config->tcp_option[1].keepalive_retry_time = ATOI(param, 10);
    }

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
    uint8_t *uri = p_http_request->URI;
    uint32_t f_addr = FLASH_START_ADDR_BANK1_OFFSET;
    uint32_t remain_len = 0, buf_len = 0;
    struct __firmware_update *fwupdate = (struct __firmware_update *) & (get_DevConfig_pointer()->firmware_update);
    uint8_t *temp_buf;

    sock = p_http_request->socket;
    boundary_pos = strstr(uri, "boundary=");
    if (boundary_pos != NULL) {
        sscanf(boundary_pos, "boundary=%127s", boundary);
        boundary_len = strlen(boundary);
        PRT_HTTP("boundary = %s, boundary_len = %d\r\n", boundary, boundary_len);
    }

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


void http_webserver_task(void *argument)  {
    const uint8_t socknumlist[MAX_HTTPSOCK] = {SOCK_HTTPSERVER_1, SOCK_HTTPSERVER_2, SOCK_HTTPSERVER_3};
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



