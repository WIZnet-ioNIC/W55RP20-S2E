
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "netHandler.h"
#include "common.h"
#include "ConfigData.h"
#include "wizchip_conf.h"
#include "dhcp.h"
#include "dhcp_cb.h"
#include "seg.h"
#include "WIZ5XXSR-RP_Debug.h"
#include "socket.h"

extern xSemaphoreHandle net_segcp_udp_sem;
extern xSemaphoreHandle net_segcp_tcp_sem;
extern xSemaphoreHandle net_http_webserver_sem;
extern xSemaphoreHandle net_seg_sem;
extern xSemaphoreHandle seg_sem;

extern uint8_t g_send_buf[DATA_BUF_SIZE];
extern uint8_t g_recv_mqtt_buf[DATA_BUF_SIZE];

NetStatus g_net_status = NET_LINK_DISCONNECTED;

uint8_t flag_process_dhcp_success = OFF;
uint8_t flag_dhcp_stop = OFF;
uint8_t flag_process_dns_success = OFF;

NetStatus get_net_status (void) {
    return g_net_status;
}

void net_status_task(void *argument)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t phylink_count; 
    int ret;

    while(1)
    {
        switch (g_net_status) {
            case NET_LINK_DISCONNECTED:
                phylink_count = 0;
                while (check_phylink_status() == PHY_LINK_OFF) {
                    vTaskDelay(100);
                    phylink_count++;
                    if (phylink_count == 50) {
                        phylink_count = 0;
                        PRT_INFO("W5500 RESET\r\n");
                        wizchip_reset();
                        wizchip_initialize();

                        switch(dev_config->network_connection.working_mode)
                        {
                            case TCP_CLIENT_MODE:
                            case TCP_SERVER_MODE:
                            case TCP_MIXED_MODE:
                            case SSL_TCP_CLIENT_MODE:
                              wizchip_gpio_interrupt_initialize(SEG_DATA0_SOCK, (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT));  
                              break;

                            case UDP_MODE:
                              wizchip_gpio_interrupt_initialize(SEG_DATA0_SOCK, SIK_RECEIVED);
                              break;

                            default:
                              break;
                        }
                    }
                }
                g_net_status = NET_LINK_CONNECTED;
                
                break;

            case NET_LINK_CONNECTED:
                    Net_Conf();
                    xSemaphoreGive(net_segcp_udp_sem);
                    if(dev_config->network_option.dhcp_use) {
                        set_stop_dhcp_flag(0);
                        if(process_dhcp() == DHCP_IP_LEASED) // DHCP success
                            flag_process_dhcp_success = ON;
                        else // DHCP failed
                            Net_Conf(); // Set default static IP settings
                    }
                    display_Net_Info();
                    display_Dev_Info_dhcp();

                    if(dev_config->network_connection.working_mode != TCP_SERVER_MODE)  {
                        if(dev_config->network_connection.dns_use) {
                            if(process_dns()) {
                                flag_process_dns_success = ON;
                                printf("flag_process_dns_success = ON\r\n");
                            }
                            display_Dev_Info_dns();
                        }
                    }
                    g_net_status = NET_IP_UP;
                    xSemaphoreGive(net_seg_sem);
                    xSemaphoreGive(net_segcp_tcp_sem);
                    xSemaphoreGive(net_http_webserver_sem);
                    break;

            case NET_IP_UP:
                while(1) {
                    if(flag_process_dhcp_success == ON) {
                        if (get_stop_dhcp_flag() == 0) {
                            ret = DHCP_run();
                            if (ret == DHCP_FAILED) {
                                g_net_status = NET_LINK_DISCONNECTED;
                                process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY);
                                xSemaphoreGive(seg_sem);
                                break;
                            }
                        }
                    }
                    if (check_phylink_status() == PHY_LINK_OFF) {

#if 1   //restore status
                        g_net_status = NET_LINK_DISCONNECTED;
                        if (get_device_status() != ST_ATMODE)
                          set_device_status(ST_OPEN);
                        process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY);
                        xSemaphoreGive(seg_sem);
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

uint8_t set_stop_dhcp_flag(uint8_t flag) //0:start, 1:stop
{
    flag_dhcp_stop = flag;
}

uint8_t get_stop_dhcp_flag(void)
{
    return flag_dhcp_stop;
}

int8_t process_dhcp(void)
{
    uint8_t ret = 0;
    uint8_t dhcp_retry = 0;

    PRT_DHCP(" - DHCP Client running\r\n");

    close(SOCK_DHCP);
    DHCP_init(SOCK_DHCP, g_recv_mqtt_buf);
    reg_dhcp_cbfunc(w5x00_dhcp_assign, w5x00_dhcp_assign, NULL);
    if (get_device_status() != ST_ATMODE)
        set_device_status(ST_UPGRADE);
    while(1)
    {
        ret = DHCP_run();
        vTaskDelay(10);

        if(ret == DHCP_IP_LEASED)
        {
            PRT_DHCP(" - DHCP Success\r\n");
            break;
        }
        else if(ret == DHCP_FAILED)
        {
            dhcp_retry++;
            if(dhcp_retry <= DHCP_RETRY_COUNT) PRT_DHCP(" - DHCP Timeout occurred and retry [%d]\r\n", dhcp_retry);
        }

        if(dhcp_retry > DHCP_RETRY_COUNT)
        {
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
    if (get_device_status() != ST_ATMODE)
        set_device_status(ST_OPEN);

    return ret;
}

