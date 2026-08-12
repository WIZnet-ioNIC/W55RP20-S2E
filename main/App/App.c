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

#include "hardware/watchdog.h"

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

#define SEG_TASK_STACK_SIZE (1024 * 4)
// Left at 18, which is below everything it shares the data path with.
//
// configMAX_PRIORITIES is 32, so xTaskCreate silently clamps every priority above 31
// to 31 - that is all of the seg and segcp tasks. This one is therefore alone at 18
// while eight data-path tasks share 31, and it holds seg_critical_sem across the whole
// of do_seg(), so anything that keeps it off the CPU stops the channel outright.
//
// Two things used to do exactly that, both since fixed: the drain task retook
// seg_critical_sem without ever yielding, and the transmit wait spun on taskYIELD()
// through an entire DMA. Raising this to 31 instead was tried and reverted - it
// removed the stall but caused ring overflow and lost frames, because the task then
// contends for the semaphore with the drain task it was waiting on, and silent loss is
// worse than a detectable stall. The structural fix is to stop holding
// seg_critical_sem across all of do_seg(); until then this task is only as safe as the
// priority 31 tasks are willing to block.
#define SEG_TASK_PRIORITY 18

#define SEG_TIMER_TASK_STACK_SIZE 256
#define SEG_TIMER_TASK_PRIORITY 45

#define SEG_U2E_TASK_STACK_SIZE 512
#define SEG_U2E_TASK_PRIORITY 41

#define SEG_RECV_TASK_STACK_SIZE 512
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
xSemaphoreHandle net_seg_u2e_sem[DEVICE_UART_CNT] = {NULL, };
xSemaphoreHandle segcp_uart_sem = NULL;
xSemaphoreHandle seg_u2e_sem[DEVICE_UART_CNT] = {NULL, };
xSemaphoreHandle seg_e2s_sem = NULL;
xSemaphoreHandle seg_sem[DEVICE_UART_CNT] = {NULL, };
xSemaphoreHandle seg_critical_sem[DEVICE_UART_CNT] = {NULL, };
// Serializes the normal receive task with the bounded CLOSE_WAIT drain for the
// same channel. Both paths share g_recv_buf/e2u_size and must never run at once.
xSemaphoreHandle seg_recv_sem[DEVICE_UART_CNT] = {NULL, };
// Serializes whole ioLibrary socket operations across every SEG data socket.
// ioLibrary keeps sock_is_sending and sock_io_mode as shared bitmaps, so a
// per-channel lock is not sufficient on the dual-core FreeRTOS build.
xSemaphoreHandle seg_socket_sem = NULL;
xSemaphoreHandle seg_timer_sem = NULL;
xSemaphoreHandle wizchip_critical_sem = NULL;
xSemaphoreHandle flash_critical_sem = NULL;

TimerHandle_t seg_inactivity_timer[DEVICE_UART_CNT] = {NULL, };
TimerHandle_t seg_keepalive_timer[DEVICE_UART_CNT] = {NULL, };
TimerHandle_t seg_auth_timer[DEVICE_UART_CNT] = {NULL, };
TimerHandle_t reset_timer = NULL;

// Stack high water marks have never been measured, and the hook that is supposed to
// catch an overflow could not stop one in a Release build, so an overrun would have
// run on silently and surfaced later as something unrelated. Handles are kept so the
// headroom can be read back; the count is the four common tasks plus three per
// channel plus the timer task.
#define WATCHED_TASK_MAX (5 + (3 * DEVICE_UART_CNT))
static TaskHandle_t watched_task[WATCHED_TASK_MAX];
static int watched_task_cnt;

// Why the last reset happened, latched at startup. Both clear means it was not a
// watchdog reset at all - supply or brownout rather than firmware.
static int boot_wdt;
static int boot_wdt_timeout;

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
    //devConfig_print_all();
    // Resets have been observed with nothing logged before them - no stall report, no
    // overflow hook, no reboot message - and stacks and heap were steady right up to
    // each one. These two bits are what separates the remaining possibilities: a
    // watchdog that ran out because nothing fed it, a deliberate watchdog_reboot()
    // from device_raw_reboot() or the reset timer, and a supply or brownout event
    // that is not a watchdog reset at all.
    // Read here and report later. This runs before the USB CDC has re-enumerated,
    // so anything printed now is discarded - the banner that follows went missing the
    // same way. Reading cannot wait either: watchdog_enable() below overwrites the
    // scratch register that watchdog_enable_caused_reboot() reads.
    boot_wdt = (int)watchdog_caused_reboot();
    boot_wdt_timeout = (int)watchdog_enable_caused_reboot();

    // Must run before any task can leave a breadcrumb, and it reads the previous
    // run's record before clearing it for this one.
    seg_postmortem_init();

    display_Dev_Info_main();
    display_Net_Info();
    printf("[BOOT] watchdog=%d timeout=%d\r\n", boot_wdt, boot_wdt_timeout);
    seg_postmortem_report();

    set_W5X00_NetTimeout();

    Timer_Configuration();
    init_connection_status_io();

    for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
        serial_mode = get_serial_communation_protocol(ch);
        if (serial_mode == SEG_SERIAL_MODBUS_RTU) {
            PRT_INFO(" > CH%d Modbus Mode\r\n", ch);
            eMBRTUInit(dev_config->serial_option[ch].baud_rate, ch);
        } else if (serial_mode == SEG_SERIAL_MODBUS_ASCII) {
            PRT_INFO(" > CH%d Modbus ASCII Mode\r\n", ch);
            eMBAsciiInit(ch);
        }
    }

    net_segcp_udp_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_segcp_tcp_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    net_http_webserver_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    segcp_uart_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_e2s_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_timer_sem = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
    seg_socket_sem = xSemaphoreCreateMutex();

    for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
        net_seg_sem[ch]      = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
        net_seg_u2e_sem[ch]  = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
        seg_u2e_sem[ch]      = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
        seg_sem[ch]          = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)0);
        seg_critical_sem[ch] = xSemaphoreCreateCounting((unsigned portBASE_TYPE)0x7fffffff, (unsigned portBASE_TYPE)1);
        seg_recv_sem[ch]     = xSemaphoreCreateMutex();
    }

    xTaskCreate(net_status_task, "Net_Status_Task", NET_TASK_STACK_SIZE, NULL, NET_TASK_PRIORITY, &watched_task[watched_task_cnt++]);
    xTaskCreate(segcp_udp_task, "SEGCP_udp_Task", SEGCP_UDP_TASK_STACK_SIZE, NULL, SEGCP_UDP_TASK_PRIORITY, &watched_task[watched_task_cnt++]);
    xTaskCreate(segcp_serial_task, "SEGCP_serial_Task", SEGCP_SERIAL_TASK_STACK_SIZE, NULL, SEGCP_SERIAL_TASK_PRIORITY, &watched_task[watched_task_cnt++]);
    xTaskCreate(segcp_tcp_task, "SEGCP_tcp_Task", SEGCP_TCP_TASK_STACK_SIZE, NULL, SEGCP_TCP_TASK_PRIORITY, &watched_task[watched_task_cnt++]);

    for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
        xTaskCreate(seg_ch_task,      "SEG_Task",      SEG_TASK_STACK_SIZE, (void *)(uintptr_t)ch, SEG_TASK_PRIORITY,          &watched_task[watched_task_cnt++]);
        xTaskCreate(seg_ch_u2e_task,  "SEG_U2E_Task",  SEG_U2E_TASK_STACK_SIZE, (void *)(uintptr_t)ch, SEG_U2E_TASK_PRIORITY,      &watched_task[watched_task_cnt++]);
        xTaskCreate(seg_ch_recv_task, "SEG_Recv_Task", SEG_RECV_TASK_STACK_SIZE, (void *)(uintptr_t)ch, SEG_RECV_TASK_PRIORITY + ch, &watched_task[watched_task_cnt++]);
    }
    xTaskCreate(seg_timer_task, "SEG_Timer_task", SEG_TIMER_TASK_STACK_SIZE, NULL, SEG_TIMER_TASK_PRIORITY, &watched_task[watched_task_cnt++]);
#if SEG_FLOW_STALL_DIAG_ENABLE || SEG_S2E_STALL_RECOVERY_ENABLE
    // S2E monitor. Reports from its own task so a channel whose seg_ch_task and
    // seg_ch_recv_task are both stuck can still be diagnosed and recovered.
    // Priority 31 is the ceiling here (configMAX_PRIORITIES is 32), which is the
    // same level the seg tasks actually run at; it only wakes every 250 ms, and the
    // spin waits in the data path call taskYIELD(), so it does get scheduled.
    xTaskCreate(seg_flow_diag_task, "SEG_Flow_Diag", 1024, NULL, 31, NULL);
#endif
    // HTTP web server removed (sockets reassigned to DATA2/DATA3)
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

    // start_task has nothing left to do, so it carries the stack report rather than
    // sleeping forever. Words remaining, not bytes: a value approaching zero is a task
    // about to overrun.
    //
    // Hourly, not by the minute. At a minute the device began resetting every few
    // hours, and a two second report elsewhere took that down to under ninety seconds -
    // with no traffic at all, so not the data path. stdio is spin locked rather than
    // mutexed, so a caller descheduled while holding it leaves the next one spinning at
    // priority 31 and feeding no watchdog. Twelve lines over a twelve hour run is close
    // enough to the silence the last clean run had.
    // The banner above goes out before USB CDC has re-enumerated and is discarded,
    // which is exactly what happened to the first post-mortem line. Repeat it while
    // a capture that attaches shortly after boot can still catch it, then stop.
    for (int i = 0; i < 6; i++) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        printf("[BOOT] watchdog=%d timeout=%d\r\n", boot_wdt, boot_wdt_timeout);
        seg_postmortem_report();
    }

    // Wake every minute for the watchdog margin, but only speak when a new worst
    // gap appears, so a healthy run stays as quiet as it was before. The stack
    // report keeps its hourly cadence underneath.
    uint32_t reported_gap_ms = 0;
    int stack_countdown = 60;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));

        uint32_t gap_ms = device_wdt_max_gap_ms();

        if (gap_ms > reported_gap_ms) {
            reported_gap_ms = gap_ms;
            printf("[WDT_MARGIN] max_gap_ms=%lu at_ms=%lu deadline_ms=8388\r\n",
                   (unsigned long)gap_ms,
                   (unsigned long)device_wdt_max_gap_at_ms());
        }

        if (--stack_countdown > 0) {
            continue;
        }
        stack_countdown = 60;

        printf("[STACK] boot_wdt=%d/%d", boot_wdt, boot_wdt_timeout);
        for (int i = 0; i < watched_task_cnt; i++) {
            if (watched_task[i] == NULL) {
                continue;
            }
            printf(" %s=%lu", pcTaskGetName(watched_task[i]),
                   (unsigned long)uxTaskGetStackHighWaterMark(watched_task[i]));
        }
        printf(" heap_free=%lu heap_min=%lu\r\n",
               (unsigned long)xPortGetFreeHeapSize(),
               (unsigned long)xPortGetMinimumEverFreeHeapSize());
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

    // configASSERT is assert(), which NDEBUG removes from a Release build, so this
    // hook used to print and then return - leaving the system running on a stack that
    // has already written past its own end. The damage lands somewhere else entirely
    // and much later, which is exactly the kind of fault that cannot be traced back.
    //
    // Stop here instead. The watchdog is the only thing still running, so the reboot
    // it forces is indistinguishable from any other watchdog reboot - except that this
    // line went out first and names the task.
    //
    // Original:
    //     configASSERT((volatile void *) NULL);
    //
    // The line below is written to USB CDC, which is lost if nothing is capturing
    // it, so leave a record that survives the reboot as well.
    seg_postmortem_record(SEG_PM_REASON_STACK, (uint32_t)(uintptr_t)pxTask);
    printf("vApplicationStackOverflowHook [%s]\r\n", pcTaskName);
    for (;;) {
        ;
    }
}
