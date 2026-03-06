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
#include "spiHandler.h"
#include "storageHandler.h"
#include "wizchip_conf.h"
#include "netHandler.h"
#include "socket.h"
#include "mbrtu.h"

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

#define SEG_SPI_TRANSFER_TASK_SIZE 1024
#define SEG_SPI_TRANSFER_PRIORITY 61

#define SEG_TIMER_TASK_STACK_SIZE 256
#define SEG_TIMER_TASK_PRIORITY 45

#define SEG_U2E_TASK_STACK_SIZE 1024
#define SEG_U2E_TASK_PRIORITY 41

#define SEG_RECV_TASK_STACK_SIZE 1024
#define SEG_RECV_TASK_PRIORITY 40

#define HTTP_WEBSERVER_TASK_STACK_SIZE 2048
#define HTTP_WEBSERVER_TASK_PRIORITY 23

#define ETH_INTERRUPT_TASK_STACK_SIZE 512
#define ETH_INTERRUPT_TASK_PRIORITY 60

#define START_TASK_STACK_SIZE 512
#define START_TASK_PRIORITY 65

#define SEG_MQTT_YIELD_STACK_SIZE 512
#define SEG_MQTT_YIELD_PRIORITY 10

/**
    ----------------------------------------------------------------------------------------------------
    Variables
    ----------------------------------------------------------------------------------------------------
*/
#ifdef ENABLE_SEGCP
xSemaphoreHandle net_segcp_udp_sem = NULL;
xSemaphoreHandle net_segcp_tcp_sem = NULL;
xSemaphoreHandle segcp_udp_sem = NULL;
xSemaphoreHandle segcp_tcp_sem = NULL;
xSemaphoreHandle segcp_uart_sem = NULL;
#endif
xSemaphoreHandle net_http_webserver_sem = NULL;
xSemaphoreHandle net_seg_sem = NULL;
xSemaphoreHandle eth_interrupt_sem = NULL;
xSemaphoreHandle seg_u2e_sem = NULL;
xSemaphoreHandle seg_e2u_sem = NULL;
xSemaphoreHandle seg_spi_pending_sem = NULL;
xSemaphoreHandle seg_sem = NULL;
xSemaphoreHandle seg_critical_sem = NULL;
xSemaphoreHandle seg_timer_sem = NULL;
xSemaphoreHandle wizchip_critical_sem = NULL;
xSemaphoreHandle flash_critical_sem = NULL;

TimerHandle_t seg_inactivity_timer = NULL;
TimerHandle_t seg_keepalive_timer = NULL;
TimerHandle_t seg_auth_timer = NULL;
TimerHandle_t spi_reset_timer = NULL;
TimerHandle_t reset_timer = NULL;

#if OPMODE == MQTT_CLIENT_MODE || OPMODE == MQTTS_CLIENT_MODE
TaskHandle_t seg_mqtt_yield_task_handle = NULL;
#endif

/**
    ----------------------------------------------------------------------------------------------------
    Functions
    ----------------------------------------------------------------------------------------------------
*/
static void RP2040_Init(void);
static void RP2040_W5X00_Init(void);
static void set_W5X00_NetTimeout(void);
static void serial_interface_init(void);
static void gpio_irq_init(void);
static void semaphore_init(void);
static void task_create(void);
void start_task(void *argument);
void eth_interrupt_task(void *argument);

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

/*  semaphore_init
    태스크 간 동기화에 사용되는 모든 카운팅 세마포어를 생성한다.
    반드시 xTaskCreate() 호출 이전에 실행되어야 한다.
    (세마포어가 NULL인 상태에서 다른 태스크가 Give/Take하면 크래시 발생)
*/
static void semaphore_init(void) {
#ifdef ENABLE_SEGCP
    net_segcp_udp_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_segcp_tcp_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    segcp_udp_sem     = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    segcp_tcp_sem     = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    segcp_uart_sem    = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
#endif
    net_http_webserver_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_seg_sem            = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    eth_interrupt_sem      = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_e2u_sem            = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_u2e_sem            = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    if (get_uart_spi_if()) {
        seg_spi_pending_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    }
    seg_sem          = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_timer_sem    = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_critical_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)1);
}

/*  serial_interface_init
    UART/SPI 인터페이스 선택 핀을 읽어 시리얼 모드를 결정하고 초기화한다.
    - SPI 슬레이브 모드: UART를 해제하고 SPI 핀/클럭을 설정한다.
    - UART 모드: UART 인터럽트를 활성화하고, HW 트리거 핀 상태에 따라 AT 모드로 전환한다.
*/
static void serial_interface_init(void) {
    DevConfig *dev_config = get_DevConfig_pointer();

    init_uart_spi_if_sel_pin();
    if (get_uart_spi_if()) {
        PRT_INFO(" > Serial SPI Mode\r\n");
        DATA0_UART_Deinit();
        gpio_init(DATA0_SPI_INT_PIN);
        GPIO_Configuration(DATA0_SPI_INT_PIN, IO_OUTPUT, IO_PULLUP);
        GPIO_Output_Set(DATA0_SPI_INT_PIN);
        DATA0_SPI_Configuration(PLL_SYS_KHZ * 1000);
        dev_config->serial_option.uart_interface = SPI_IF_SLAVE;
    } else {
        DATA0_UART_Interrupt_Enable();
        if (get_hw_trig_pin() == 0) {
            init_trigger_modeswitch(DEVICE_AT_MODE);
        }
    }
}

/*  gpio_irq_init
    연결 상태 LED/IO 초기화, Modbus RTU 초기화(해당 시),
    동작 모드에 따라 WIZchip 소켓 수신 인터럽트를 활성화하고
    WIZchip IRQ 핀(WIZCHIP_PIN_IRQ)을 하강 엣지 인터럽트로 설정 및 콜백을 등록한다.
*/
static void gpio_irq_init(void) {
    DevConfig *dev_config = get_DevConfig_pointer();
    uint8_t serial_mode;

    init_connection_status_io();

    serial_mode = get_serial_communation_protocol();
    if (serial_mode == SEG_SERIAL_MODBUS_RTU) {
        PRT_INFO(" > Modbus Mode\r\n");
        eMBRTUInit(dev_config->serial_option.baud_rate);
    }

    switch (dev_config->network_connection.working_mode) {
    case TCP_CLIENT_MODE:
    case TCP_SERVER_MODE:
    case TCP_MIXED_MODE:
    case SSL_TCP_CLIENT_MODE:
    case UDP_MODE:
        wizchip_gpio_interrupt_initialize(SEG_DATA0_SOCK, SIK_RECEIVED);
        break;

    default:
        break;
    }

    GPIO_Configuration(WIZCHIP_PIN_IRQ, IO_INPUT, IO_PULLUP); //Set interrupt Pin
    GPIO_Configuration_IRQ(WIZCHIP_PIN_IRQ, IO_IRQ_FALL);
    GPIO_Configuration_Callback();
}

/*  task_create
    애플리케이션에서 사용하는 모든 FreeRTOS 태스크를 생성한다.
    semaphore_init() 이후에 호출되어야 하며,
    ENABLE_SEGCP / OPMODE 컴파일 옵션에 따라 생성되는 태스크가 달라진다.
*/
static void task_create(void) {
    //TaskHandle_t task_handle;

    xTaskCreate(net_status_task, "Net_Status_Task", NET_TASK_STACK_SIZE, NULL, NET_TASK_PRIORITY, NULL);

#ifdef ENABLE_SEGCP
    xTaskCreate(segcp_udp_task, "SEGCP_udp_Task", SEGCP_UDP_TASK_STACK_SIZE, NULL, SEGCP_UDP_TASK_PRIORITY, NULL);
    xTaskCreate(segcp_serial_task, "SEGCP_serial_Task", SEGCP_SERIAL_TASK_STACK_SIZE, NULL, SEGCP_SERIAL_TASK_PRIORITY, NULL);
    xTaskCreate(segcp_tcp_task, "SEGCP_tcp_Task", SEGCP_TCP_TASK_STACK_SIZE, NULL, SEGCP_TCP_TASK_PRIORITY, NULL);
#endif

    xTaskCreate(eth_interrupt_task, "ETH_INTERRUPT_Task", ETH_INTERRUPT_TASK_STACK_SIZE, NULL, ETH_INTERRUPT_TASK_PRIORITY, NULL);
    xTaskCreate(seg_task, "SEG_Task", SEG_TASK_STACK_SIZE, NULL, SEG_TASK_PRIORITY, NULL);
    if (get_uart_spi_if()) {
        xTaskCreate(spi_data_transfer_task, "SPI_TRANSFER_TASK", SEG_SPI_TRANSFER_TASK_SIZE, NULL, SEG_SPI_TRANSFER_PRIORITY, NULL);
    }
    //xTaskCreate(spi_data_transfer_task, "SPI_TRANSFER_TASK", SEG_SPI_TRANSFER_TASK_SIZE, NULL, SEG_SPI_TRANSFER_PRIORITY, &task_handle);
    //vTaskCoreAffinitySet(task_handle, 1 << 1); // Set SPI transfer task to core 1

    xTaskCreate(seg_u2e_task, "SEG_U2E_Task", SEG_U2E_TASK_STACK_SIZE, NULL, SEG_U2E_TASK_PRIORITY, NULL);
    xTaskCreate(seg_recv_task, "SEG_Recv_Task", SEG_RECV_TASK_STACK_SIZE, NULL, SEG_RECV_TASK_PRIORITY, NULL);
    xTaskCreate(seg_timer_task, "SEG_Timer_task", SEG_TIMER_TASK_STACK_SIZE, NULL, SEG_TIMER_TASK_PRIORITY, NULL);

#if OPMODE == MQTT_CLIENT_MODE || OPMODE == MQTTS_CLIENT_MODE
    xTaskCreate(seg_mqtt_yield_task, "SEG_MQTT_YIELD_Task", SEG_MQTT_YIELD_STACK_SIZE, NULL, SEG_MQTT_YIELD_PRIORITY, &seg_mqtt_yield_task_handle);
#endif
}

void start_task(void *argument) {
    // RP2040 및 WIZchip SPI/이더넷 칩 하드웨어 초기화
    RP2040_Init();
    RP2040_W5X00_Init();

    // Flash에서 장치 설정(DevConfig) 로드 및 보드·UART 초기화, MAC 주소 검증
    load_DevConfig_from_storage();
    RP2040_Board_Init();
    DATA0_UART_Configuration();
    check_mac_address();

    // 시리얼 인터페이스(UART or SPI 슬레이브) 선택 및 초기화
    serial_interface_init();

    // 네트워크 설정 적용 및 디바이스/네트워크 정보 출력
    Net_Conf();
    display_Dev_Info_main();
    display_Net_Info();

    // WIZchip TCP 재전송 타임아웃 설정
    set_W5X00_NetTimeout();

    // 소프트웨어 타이머, GPIO/IRQ, Modbus 초기화
    Timer_Configuration();
    gpio_irq_init();

    // Semaphore init by Lihan
    semaphore_init();

    // FreeRTOS 태스크 생성
    task_create();

    // if (dev_config->config_common.pw_search[0] == 0) {
    //     xTaskCreate(http_webserver_task, "http_webserver_task", HTTP_WEBSERVER_TASK_STACK_SIZE, NULL, HTTP_WEBSERVER_TASK_PRIORITY, NULL);
    // }

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

void eth_interrupt_task(void *argument) {
    uint16_t reg_val;

    while (1) {
        xSemaphoreTake(eth_interrupt_sem, portMAX_DELAY);
        ctlsocket(SEG_DATA0_SOCK, CS_GET_INTERRUPT, (void *)&reg_val);
        if (reg_val & SIK_RECEIVED) {
            xSemaphoreGive(seg_e2u_sem);
        }
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
