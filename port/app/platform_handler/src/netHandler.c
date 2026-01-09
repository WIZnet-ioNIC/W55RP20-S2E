#include <stdlib.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "WIZnet_board.h"
#include "port_common.h"
#include "netHandler.h"
#include "deviceHandler.h"
#include "dnsHandler.h"
#include "common.h"
#include "ConfigData.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "dhcp_cb.h"
#include "seg.h"
#include "WIZ5XXSR-RP_Debug.h"
#include "socket.h"
#include "w5x00_spi.h"
#include "w5x00_gpio_irq.h"

#include "gpioHandler.h"
extern xSemaphoreHandle net_segcp_udp_sem;
extern xSemaphoreHandle net_segcp_tcp_sem;
extern xSemaphoreHandle net_http_webserver_sem;
extern xSemaphoreHandle net_seg_sem[DEVICE_UART_CNT];

extern uint8_t g_send_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];
extern uint8_t g_recv_mqtt_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];

NetStatus g_net_status = NET_LINK_DISCONNECTED;

uint8_t flag_process_dhcp_success = OFF;
uint8_t flag_dhcp_stop = OFF;
uint8_t flag_process_dns_success[DEVICE_UART_CNT] = {OFF, };

NetStatus get_net_status(void) {
    return g_net_status;
}

void net_status_task(void *argument) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t phylink_count;
    int ret;

    while (1) {
        switch (g_net_status) {
        case NET_LINK_DISCONNECTED:
            phylink_count = 0;
            while (check_phylink_status() == PHY_LINK_OFF) {
                vTaskDelay(100);
                phylink_count++;
                if (phylink_count == 50) {
                    phylink_count = 0;
                    PRT_INFO("NET_LINK_DISCONNECTED\r\n");
                    wizchip_recovery();

                }
            }
            g_net_status = NET_LINK_CONNECTED;
            break;

        case NET_LINK_CONNECTED:
            xSemaphoreGive(net_segcp_udp_sem);
            if (dev_config->network_option.dhcp_use) {
                set_stop_dhcp_flag(0);
                //PRT_INFO("DHCP waiting 3 seconds...\r\n");
                //vTaskDelay(3000); // Wait for 3 seconds before starting DHCP
                if (process_dhcp() == DHCP_IP_LEASED) { // DHCP success
                    flag_process_dhcp_success = ON;
                } else {  // DHCP failed
                    //dev_config->network_option.dhcp_use = 0;
                    //Net_Conf(); // Set default static IP settings
                    PRT_ERR("NET_LINK_CONNECTED DHCP Failed\r\n");
                    wizchip_recovery();
                    break;
                }
            }
            display_Net_Info();
            display_Dev_Info_dhcp();

            if (dev_config->network_connection[SEG_DATA0_CH].working_mode != TCP_SERVER_MODE)  {
                if (dev_config->network_connection[SEG_DATA0_CH].dns_use) {
                    //PRT_INFO("DNS waiting 3 seconds...\r\n");
                    //vTaskDelay(3000); // Wait for 3 seconds before starting DHCP
                    if (process_dns(SEG_DATA0_CH) == DNS_RET_SUCCESS) {
                        flag_process_dns_success[SEG_DATA0_CH] = ON;
                        PRT_INFO("flag_process_dns_success[SEG_DATA0_CH] = ON\r\n");
                    } else {
                        PRT_ERR("NET_LINK_CONNECTED DNS CH0 Failed\r\n");
                        flag_process_dns_success[SEG_DATA0_CH] = OFF;
                        break;
                    }
                    display_Dev_Info_dns(SEG_DATA0_CH);
                }
            }

            if (dev_config->network_connection[SEG_DATA1_CH].working_mode != TCP_SERVER_MODE)  {
                if (dev_config->network_connection[SEG_DATA1_CH].dns_use) {
                    if (strcmp(dev_config->network_connection[SEG_DATA0_CH].dns_domain_name, dev_config->network_connection[SEG_DATA1_CH].dns_domain_name) == 0) {
                        memcpy(dev_config->network_connection[SEG_DATA1_CH].remote_ip, dev_config->network_connection[SEG_DATA0_CH].remote_ip, 4);
                    } else {
                        if (process_dns(SEG_DATA1_CH)) {
                            flag_process_dns_success[SEG_DATA1_CH] = ON;
                            printf("flag_process_dns_success[SEG_DATA1_CH] = ON\r\n");
                        } else {
                            PRT_ERR("NET_LINK_CONNECTED DNS CH1 Failed\r\n");
                            flag_process_dns_success[SEG_DATA1_CH] = OFF;
                            break;
                        }
                    }
                    display_Dev_Info_dns(SEG_DATA1_CH);
                }
            }

            g_net_status = NET_IP_UP;
            xSemaphoreGive(net_seg_sem[SEG_DATA0_CH]);
            xSemaphoreGive(net_seg_sem[SEG_DATA1_CH]);
            xSemaphoreGive(net_segcp_tcp_sem);
            xSemaphoreGive(net_http_webserver_sem);
            break;

        case NET_IP_UP:
            while (1) {
                if (flag_process_dhcp_success == ON) {
                    if (get_stop_dhcp_flag() == 0) {
                        ret = DHCP_run();
                        if (ret == DHCP_FAILED) {
                            PRT_ERR("NET_IP_UP DHCP Failed\r\n");
                            wizchip_recovery();
                            process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA0_CH, TRUE);
                            process_socket_termination(SEG_DATA1_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA1_CH, TRUE);
                            break;
                        }
                    }
                }
                if (check_phylink_status() == PHY_LINK_OFF) {
#if 1   //restore status
                    PRT_ERR("NET_IP_UP PHY_LINK_OFF\r\n");
                    wizchip_recovery();
                    if (get_device_status(SEG_DATA0_CH) != ST_ATMODE) {
                        set_device_status(ST_OPEN, SEG_DATA0_CH);
                        set_device_status(ST_OPEN, SEG_DATA1_CH);
                    }
                    process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA0_CH, TRUE);
                    process_socket_termination(SEG_DATA1_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA1_CH, TRUE);
#else   //device reset
                    device_raw_reboot();
#endif
                    break;
                }
                vTaskDelay(2000);
            }
            break;
        }
        vTaskDelay(10);
    }
}

uint8_t set_stop_dhcp_flag(uint8_t flag) { //0:start, 1:stop
    flag_dhcp_stop = flag;
}

uint8_t get_stop_dhcp_flag(void) {
    return flag_dhcp_stop;
}

int8_t process_dhcp(void) {
    uint8_t ret = 0;
    uint8_t dhcp_retry = 0;

    PRT_DHCP(" - DHCP Client running\r\n");

    close(SOCK_DHCP);
    DHCP_init(SOCK_DHCP, g_recv_mqtt_buf[SEG_DATA0_CH]);
    reg_dhcp_cbfunc(w5x00_dhcp_assign, w5x00_dhcp_assign, NULL);
    if (get_device_status(SEG_DATA0_CH) != ST_ATMODE) {
        set_device_status(ST_UPGRADE, SEG_DATA0_CH);
        set_device_status(ST_UPGRADE, SEG_DATA1_CH);
    }
    while (1) {
        ret = DHCP_run();
        vTaskDelay(10);

        if (ret == DHCP_IP_LEASED) {
            PRT_DHCP(" - DHCP Success\r\n");
            break;
        } else if (ret == DHCP_FAILED) {
            dhcp_retry++;
            if (dhcp_retry <= DHCP_RETRY_COUNT) {
                PRT_DHCP(" - DHCP Timeout occurred and retry [%d]\r\n", dhcp_retry);
            }
        }

        if (dhcp_retry > DHCP_RETRY_COUNT) {
#ifndef __USE_DHCP_INFINITE_LOOP__
            PRT_DHCP(" - DHCP Failed\r\n\r\n");
            DHCP_stop();
            break;
#else // If DHCP allocation failed, process_dhcp() function will try to DHCP steps again.
            PRT_DHCP(" - DHCP Failed, Reboot...\r\n\r\n");
            device_raw_reboot();
            //            DHCP_init(SOCK_DHCP, g_recv_mqtt_buf);
            //            dhcp_retry = 0;
#endif
        }
    }

    if (get_device_status(SEG_DATA0_CH) != ST_ATMODE) {
        set_device_status(ST_OPEN, SEG_DATA0_CH);
        set_device_status(ST_OPEN, SEG_DATA1_CH);
    }

    return ret;
}

void wizchip_recovery(void) {
    PRT_INFO("W5500 RESET\r\n");

    g_net_status = NET_LINK_DISCONNECTED;
    flag_process_dns_success[SEG_DATA0_CH] = OFF;
    flag_process_dns_success[SEG_DATA1_CH] = OFF;
    flag_process_dhcp_success = OFF;

    wizchip_reset();
    wizchip_initialize();
    Net_Conf();
}