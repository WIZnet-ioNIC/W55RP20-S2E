/**
    Copyright (c) 2022 WIZnet Co.,Ltd

    SPDX-License-Identifier: BSD-3-Clause
*/

/**
    ----------------------------------------------------------------------------------------------------
    Includes
    ----------------------------------------------------------------------------------------------------
*/
#include "tusb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "WIZnet_board.h"
#include "port_common.h"
#include "dhcp.h"
#include "dhcp_cb.h"
#include "dns.h"
#include "seg.h"
#include "segcp.h"
#include "ConfigData.h"
#include "timerHandler.h"
#include "uartHandler.h"
#include "deviceHandler.h"
#include "dnsHandler.h"
#include "ConfigData.h"
#include "flashHandler.h"
#include "httpHandler.h"
#include "gpioHandler.h"
#include "storageHandler.h"
#include "wizchip_conf.h"
#include "netHandler.h"
#include "socket.h"
#include "mbrtu.h"
#include "mbascii.h"

#include "w5x00_gpio_irq.h"
#include "w5x00_spi.h"

/**
    ----------------------------------------------------------------------------------------------------
    Macros
    ----------------------------------------------------------------------------------------------------
*/
/* Task */

#define NET_TASK_STACK_SIZE 1024
#define NET_TASK_PRIORITY 8

#define SEGCP_UDP_TASK_STACK_SIZE 1024
#define SEGCP_UDP_TASK_PRIORITY 52

#define SEGCP_TCP_TASK_STACK_SIZE 1024
#define SEGCP_TCP_TASK_PRIORITY 51

#define SEGCP_SERIAL_TASK_STACK_SIZE 512
#define SEGCP_SERIAL_TASK_PRIORITY 50

#define SEG_TASK_STACK_SIZE (1024 * 8)
#define SEG_TASK_PRIORITY 18

#define SEG_TIMER_TASK_STACK_SIZE 256
#define SEG_TIMER_TASK_PRIORITY 45

#define SEG_U2E_TASK_STACK_SIZE 1024
#define SEG_U2E_TASK_PRIORITY 41

#define SEG_RECV_TASK_STACK_SIZE 1024
#define SEG_RECV_TASK_PRIORITY 40

#define HTTP_WEBSERVER_TASK_STACK_SIZE 2048
#define HTTP_WEBSERVER_TASK_PRIORITY 23

#define START_TASK_STACK_SIZE 512
#define START_TASK_PRIORITY 65

/**
    ----------------------------------------------------------------------------------------------------
    Variables
    ----------------------------------------------------------------------------------------------------
*/
xSemaphoreHandle net_segcp_udp_sem = NULL;
xSemaphoreHandle net_segcp_tcp_sem = NULL;
xSemaphoreHandle net_http_webserver_sem = NULL;
xSemaphoreHandle net_seg_sem[DEVICE_UART_CNT] = {NULL, };
xSemaphoreHandle segcp_uart_sem = NULL;
xSemaphoreHandle seg_u2e_sem[DEVICE_UART_CNT] = {NULL, };
xSemaphoreHandle seg_e2s_sem = NULL;
xSemaphoreHandle seg_sem[DEVICE_UART_CNT] = {NULL, };
xSemaphoreHandle seg_socket_sem[DEVICE_UART_CNT] = {NULL, };
xSemaphoreHandle seg_timer_sem = NULL;
xSemaphoreHandle wizchip_critical_sem = NULL;
xSemaphoreHandle flash_critical_sem = NULL;

TimerHandle_t seg_inactivity_timer[DEVICE_UART_CNT] = {NULL, };
TimerHandle_t seg_keepalive_timer[DEVICE_UART_CNT] = {NULL, };
TimerHandle_t seg_auth_timer[DEVICE_UART_CNT] = {NULL, };
TimerHandle_t reset_timer = NULL;

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
static void RP2040_Init(void);
static void RP2040_W5X00_Init(void);
static void set_W5X00_NetTimeout(void);
void start_task(void *argument);

/**
    ----------------------------------------------------------------------------------------------------
    Main
    ----------------------------------------------------------------------------------------------------
*/
int main() {
    xTaskCreate(start_task, "Start_Task", START_TASK_STACK_SIZE, NULL, START_TASK_PRIORITY, NULL);
    vTaskStartScheduler();

    while (1) {
        ;
    }
}

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
/* Task */

static void RP2040_Init(void) {
#if 0
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
#endif
    //SystemCoreClockUpdate();
    flash_critical_section_init();
    sleep_ms(10);
}

static void RP2040_W5X00_Init(void) {
    wizchip_spi_initialize((PLL_SYS_KHZ * 1000 / 4)); //33.25Mhz
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
}

static void set_W5X00_NetTimeout(void) {
    DevConfig *dev_config = get_DevConfig_pointer();
    wiz_NetTimeout net_timeout;

    net_timeout.retry_cnt = dev_config->network_option.tcp_rcr_val;
    net_timeout.time_100us = 2000;
    wizchip_settimeout(&net_timeout);

    wizchip_gettimeout(&net_timeout); // TCP timeout settings
    PRT_INFO(" - Network Timeout Settings - RCR: %d, RTR: %d\r\n", net_timeout.retry_cnt, net_timeout.time_100us);
}


void start_task(void *argument) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t serial_mode;

    RP2040_Init();
    RP2040_W5X00_Init();
    load_DevConfig_from_storage();
    RP2040_Board_Init();
    DATA_UART_Configuration();
    check_mac_address();

    DATA_UART_Interrupt_Enable();
    if (get_hw_trig_pin() == 0) {
        init_trigger_modeswitch(DEVICE_AT_MODE);
    }

    Net_Conf();
    devConfig_print_all();
    //display_Dev_Info_main();
    //display_Net_Info();

    set_W5X00_NetTimeout();

    Timer_Configuration();
    init_connection_status_io();

    serial_mode = get_serial_communation_protocol(SEG_DATA0_CH);
    if (serial_mode == SEG_SERIAL_MODBUS_RTU) {
        PRT_INFO(" > CH0 Modbus Mode\r\n");
        eMBRTUInit(dev_config->serial_option[SEG_DATA0_CH].baud_rate, SEG_DATA0_CH);
    } else if (serial_mode == SEG_SERIAL_MODBUS_ASCII) {
        PRT_INFO(" > CH0 Modbus ASCII Mode\r\n");
        eMBAsciiInit(SEG_DATA0_CH);
    }

    serial_mode = get_serial_communation_protocol(SEG_DATA1_CH);
    if (serial_mode == SEG_SERIAL_MODBUS_RTU) {
        PRT_INFO(" > CH1 Modbus Mode\r\n");
        eMBRTUInit(dev_config->serial_option[SEG_DATA1_CH].baud_rate, SEG_DATA1_CH);
    } else if (serial_mode == SEG_SERIAL_MODBUS_ASCII) {
        PRT_INFO(" > CH1 Modbus ASCII Mode\r\n");
        eMBAsciiInit(SEG_DATA1_CH);
    }

    net_segcp_udp_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_segcp_tcp_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_http_webserver_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_seg_sem[SEG_DATA0_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_seg_sem[SEG_DATA1_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    segcp_uart_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_e2s_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_u2e_sem[SEG_DATA0_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_u2e_sem[SEG_DATA1_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_timer_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_sem[SEG_DATA0_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_sem[SEG_DATA1_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_socket_sem[SEG_DATA0_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)1);
    seg_socket_sem[SEG_DATA1_CH] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)1);

    xTaskCreate(net_status_task, "Net_Status_Task", NET_TASK_STACK_SIZE, NULL, NET_TASK_PRIORITY, NULL);
    xTaskCreate(segcp_udp_task, "SEGCP_udp_Task", SEGCP_UDP_TASK_STACK_SIZE, NULL, SEGCP_UDP_TASK_PRIORITY, NULL);
    xTaskCreate(segcp_serial_task, "SEGCP_serial_Task", SEGCP_SERIAL_TASK_STACK_SIZE, NULL, SEGCP_SERIAL_TASK_PRIORITY, NULL);
    xTaskCreate(segcp_tcp_task, "SEGCP_tcp_Task", SEGCP_TCP_TASK_STACK_SIZE, NULL, SEGCP_TCP_TASK_PRIORITY, NULL);

    //xTaskCreate(seg_task, "SEG_Task", SEG_TASK_STACK_SIZE, NULL, SEG_TASK_PRIORITY, NULL);
    xTaskCreate(seg0_task, "SEG0_Task", SEG_TASK_STACK_SIZE, NULL, SEG_TASK_PRIORITY, NULL);
    xTaskCreate(seg1_task, "SEG1_Task", SEG_TASK_STACK_SIZE, NULL, SEG_TASK_PRIORITY, NULL);
    xTaskCreate(seg0_u2e_task, "SEG0_U2E_Task", SEG_U2E_TASK_STACK_SIZE, NULL, SEG_U2E_TASK_PRIORITY, NULL);
    xTaskCreate(seg1_u2e_task, "SEG1_U2E_Task", SEG_U2E_TASK_STACK_SIZE, NULL, SEG_U2E_TASK_PRIORITY, NULL);
    //xTaskCreate(seg_recv_task, "SEG1_Recv_Task", SEG_RECV_TASK_STACK_SIZE, NULL, SEG_RECV_TASK_PRIORITY, NULL);
    xTaskCreate(seg0_recv_task, "SEG0_Recv_Task", SEG_RECV_TASK_STACK_SIZE, NULL, SEG_RECV_TASK_PRIORITY, NULL);
    xTaskCreate(seg1_recv_task, "SEG1_Recv_Task", SEG_RECV_TASK_STACK_SIZE, NULL, SEG_RECV_TASK_PRIORITY + 1, NULL);
    xTaskCreate(seg_timer_task, "SEG_Timer_task", SEG_TIMER_TASK_STACK_SIZE, NULL, SEG_TIMER_TASK_PRIORITY, NULL);
    xTaskCreate(http_webserver_task, "http_webserver_task", HTTP_WEBSERVER_TASK_STACK_SIZE, NULL, HTTP_WEBSERVER_TASK_PRIORITY, NULL);

#if defined(MBEDTLS_PLATFORM_C) && defined(MBEDTLS_PLATFORM_MEMORY)
    mbedtls_platform_set_calloc_free(pvPortCalloc, vPortFree);
#endif
    reset_timer = xTimerCreate("reset_timer", pdMS_TO_TICKS(5000), pdFALSE, 0, reset_timer_callback);
#ifdef __USE_WATCHDOG__
    watchdog_enable(8388, 0);
#endif

    while (1) {
        vTaskDelay(1000000000);
    }

}

void vApplicationPassiveIdleHook(void) {
#ifdef __USE_WATCHDOG__
    static uint8_t core_num = 0;
    uint8_t core_num_tmp = get_core_num();

    if (core_num != core_num_tmp) {
        device_wdt_reset();
        core_num = core_num_tmp;
    }
#endif

}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void) pcTaskName;
    (void) pxTask;

    /*  Run time stack overflow checking is performed if
        configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
        function is called if a stack overflow is detected. */

    /* Force an assert. */
    printf("vApplicationStackOverflowHook [%s]\r\n", pcTaskName);
    configASSERT((volatile void *) NULL);
}
