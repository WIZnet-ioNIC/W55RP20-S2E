
#include <string.h>
#include <stdlib.h>
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/timer.h"
#include "common.h"
#include "ConfigData.h"
#include "wizchip_conf.h"

#include "socket.h"
#include "seg.h"
#include "segcp.h"
#include "flashHandler.h"
#include "storageHandler.h"
#include "gpioHandler.h"
#include "deviceHandler.h"
#include "uartHandler.h"
#include "timerHandler.h"
#include "netHandler.h"
#include "util.h"

#include "dns.h"
#include "dhcp.h"

uint16_t get_firmware_from_network(uint8_t sock, uint8_t * buf);
uint16_t get_firmware_from_server(uint8_t sock, uint8_t * server_ip, uint8_t * buf);

void reset_fw_update_timer(void);
uint16_t get_any_port(void);

uint8_t reset_flag = 0;
static uint16_t any_port = 0;

uint8_t g_send_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];
uint8_t g_recv_mqtt_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];
uint8_t g_recv_buf[DEVICE_UART_CNT][DATA_BUF_SIZE];

extern TimerHandle_t reset_timer;

void device_set_factory_default(void) {
    set_DevConfig_to_factory_value();
    save_DevConfig_to_storage();
}


void device_socket_termination(void) {
    for (int ch = 0; ch < DEVICE_UART_CNT; ch++) {
        process_socket_termination(seg_data_sock[ch], SOCK_TERMINATION_DELAY, ch, FALSE);
    }

    seg_wizchip_api_lock();
    for (int i = SEG_DATA3_SOCK + 1; i < _WIZCHIP_SOCK_NUM_; i++) {
        close(i);
    }
    seg_wizchip_api_unlock();
}

void device_reboot(void) {
    device_socket_termination();
    device_raw_reboot();
    while (1);
}

void device_raw_reboot(void) {
    //NVIC_SystemReset();
    reset_flag = 1;
    watchdog_reboot(0, SRAM_END, 10);
    while (1);
}

// The watchdog only reports the one gap that killed the device, and a run that
// survives says nothing about how close it came. Track the largest gap between
// feeds so a short run can show the near misses: 8.388 s is the deadline, and a
// device that normally feeds every few milliseconds but occasionally goes quiet
// for seconds is already showing the fault, reset or not.
static volatile uint32_t wdt_feed_last_ms;

void device_wdt_reset(void) {
    if (get_reset_flag() == 0) {
        // millis() counts in the 1 ms repeating-timer callback, so it stops
        // whenever that interrupt is starved - which is one of the ways the
        // watchdog can expire. Read the hardware timer instead.
        wdt_feed_last_ms = time_us_32() / 1000U;

        watchdog_update();
    }
}

// How long since anything last fed the watchdog. Read from the passive idle
// hook, which keeps running when the tasks that feed it do not.
uint32_t device_wdt_since_feed_ms(void) {
    uint32_t last = wdt_feed_last_ms;

    if (last == 0) {
        return 0;
    }

    return (time_us_32() / 1000U) - last;
}

// "SGPM". Scratch survives the reset but powers up holding whatever was there,
// so the record is only believed when this marker is present.
#define SEG_PM_MAGIC 0x5347504DU

static uint32_t seg_pm_boot_crumb;
static uint32_t seg_pm_boot_reason;
static uint32_t seg_pm_boot_detail;
static uint8_t seg_pm_boot_valid;

void seg_postmortem_init(void) {
    if (watchdog_hw->scratch[0] == SEG_PM_MAGIC) {
        seg_pm_boot_valid = 1;
        seg_pm_boot_crumb = watchdog_hw->scratch[1];
        seg_pm_boot_reason = watchdog_hw->scratch[2];
        seg_pm_boot_detail = watchdog_hw->scratch[3];
    }

    watchdog_hw->scratch[0] = SEG_PM_MAGIC;
    watchdog_hw->scratch[1] = 0;
    watchdog_hw->scratch[2] = SEG_PM_REASON_NONE;
    watchdog_hw->scratch[3] = 0;
}

void seg_postmortem_report(void) {
    // A clean start has nothing to say. Only a run that left a record speaks.
    if (!seg_pm_boot_valid) {
        return;
    }

    // The timestamp is the millisecond count when that task last completed a pass.
    // Comparing it with the reset moment gives the length of the silence.
    PRT_INFO("[POSTMORTEM] last_task=%lu ch=%lu at_ms=%lu reason=%lu detail=0x%08lX\r\n",
             (unsigned long)(seg_pm_boot_crumb >> 28),
             (unsigned long)((seg_pm_boot_crumb >> 24) & 0x0FU),
             (unsigned long)(seg_pm_boot_crumb & 0x00FFFFFFU),
             (unsigned long)seg_pm_boot_reason,
             (unsigned long)seg_pm_boot_detail);

    if (seg_pm_boot_reason == SEG_PM_REASON_SOCKLOCK) {
        // The holder is always SEG_U2E_Task, so the record now spends its room
        // on the two things still unknown: which call it stopped in, and who
        // held the SPI critical section underneath it.
        //   step  1 free size   2 getSn_SR    3 getSn_IR   4 TxMAX+free size
        //         5 send data   6 Sn_CR poll  7 recv       8 getSn_RX_RSR
        //   spi   name character at offset four - T SEG_Task, U SEG_U2E_Task,
        //         R SEG_Recv_Task, P SEGCP_*, '-' nobody holds it
        //   wait  someone is queued on the SPI lock, which an unheld lock and a
        //         contended one otherwise look identical from outside
        unsigned int spi_owner = (unsigned int)((seg_pm_boot_detail >> 16) & 0xFFU);

        PRT_INFO("[POSTMORTEM] socket mutex held past %u ms by SEG_U2E step=%u, "
                 "spi owner=%c held=%u ms wait=%u, lock %s\r\n",
                 (unsigned int)SEG_SOCKET_LOCK_STUCK_MS,
                 (unsigned int)(seg_pm_boot_detail >> 24),
                 (spi_owner != 0U) ? (char)spi_owner : '-',
                 (unsigned int)(seg_pm_boot_detail & 0x3FFFU),
                 (unsigned int)((seg_pm_boot_detail >> 15) & 1U),
                 ((seg_pm_boot_detail >> 14) & 1U) ? "parked" : "spinning");
    }

    if (seg_pm_boot_reason == SEG_PM_REASON_WDT_GAP) {
        // One byte per channel, saying where its receive loop was standing when
        // the feeding stopped.
        //   1 head        2 CTS gate   3 getSn_RX_RSR   4 uart tx wait
        //   5 recv/uart   7 tail       0 never ran
        PRT_INFO("[POSTMORTEM] no watchdog feed for %u ms, recv steps "
                 "ch0=%u ch1=%u ch2=%u ch3=%u\r\n",
                 (unsigned int)SEG_SOCKET_LOCK_STUCK_MS,
                 (unsigned int)(seg_pm_boot_detail & 0xFFU),
                 (unsigned int)((seg_pm_boot_detail >> 8) & 0xFFU),
                 (unsigned int)((seg_pm_boot_detail >> 16) & 0xFFU),
                 (unsigned int)((seg_pm_boot_detail >> 24) & 0xFFU));
    }
}

void seg_postmortem_mark(uint8_t task_id, uint8_t channel) {
    // Hardware timer, not millis(): the breadcrumb has to keep its own time even
    // when the 1 ms callback that drives millis() is the thing that stopped.
    watchdog_hw->scratch[1] = ((uint32_t)task_id << 28) |
                              (((uint32_t)channel & 0x0FU) << 24) |
                              ((time_us_32() / 1000U) & 0x00FFFFFFU);
}

void seg_postmortem_record(uint32_t reason, uint32_t detail) {
    watchdog_hw->scratch[0] = SEG_PM_MAGIC;
    watchdog_hw->scratch[2] = reason;
    watchdog_hw->scratch[3] = detail;
}

// The stacked PC says where the fault happened. Which stack holds the frame
// depends on bit 2 of the EXC_RETURN value still in lr on entry.
void __attribute__((naked)) isr_hardfault(void) {
    __asm volatile(
        "movs r0, #4                          \n"
        "mov  r1, lr                          \n"
        "tst  r0, r1                          \n"
        "beq  1f                              \n"
        "mrs  r0, psp                         \n"
        "b    2f                              \n"
        "1:                                   \n"
        "mrs  r0, msp                         \n"
        "2:                                   \n"
        "ldr  r1, =seg_postmortem_hardfault   \n"
        "bx   r1                              \n"
    );
}

void seg_postmortem_hardfault(uint32_t *frame) {
    // frame[6] is the stacked PC on Cortex-M0+.
    seg_postmortem_record(SEG_PM_REASON_HARDFAULT, frame[6]);
    for (;;) {
        ;
    }
}

void reset_timer_callback(TimerHandle_t xTimer) {
    PRT_INFO("Timer Reset\r\n");
    reset_flag = 1;
    watchdog_reboot(0, SRAM_END, 1);
}

uint8_t get_reset_flag(void) {
    return reset_flag;
}

#if 0
void disable_interrupts(void) {
    SysTick->CTRL &= ~1;

    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICPR[0] = 0xFFFFFFFF;
}
#endif

void reset_peripherals(void) {
    reset_block(~(
                    RESETS_RESET_IO_QSPI_BITS |
                    RESETS_RESET_PADS_QSPI_BITS |
                    RESETS_RESET_SYSCFG_BITS |
                    RESETS_RESET_PLL_SYS_BITS
                ));
}


void jump_to_app(uint32_t app_addr) {
    uint32_t reset_vector = *(volatile uint32_t *)(app_addr + 0x04);
    SCB->VTOR = app_addr;

    asm volatile("msr msp, %0"::"g"
                 (*(volatile uint32_t *)app_addr));
    asm volatile("bx %0"::"r"(reset_vector));
}


uint8_t device_bank_update(void) {
    struct __firmware_update *fwupdate = (struct __firmware_update *) & (get_DevConfig_pointer()->firmware_update);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);

    uint8_t ret = DEVICE_FWUP_RET_PROGRESS;
    uint16_t recv_len = 0;
    static uint32_t write_fw_len;
    uint32_t f_addr;
    uint32_t remain_len = 0, buf_len = 0;
    uint8_t *temp_buf;
    uint8_t *recv_temp_buf;

    if ((fwupdate->fwup_size == 0) || (fwupdate->fwup_size > FLASH_APP_BANK_SIZE)) {
        if (serial_common->serial_debug_en)
            PRT_INFO(" > SEGCP:BU_UPDATE:FAILED - Invalid firmware size: %ld bytes (Firmware size must be within %d bytes)\r\n",
                     fwupdate->fwup_size,
                     FLASH_APP_BANK_SIZE);

        return DEVICE_FWUP_RET_FAILED;
    }

    if (serial_common->serial_debug_en) {
        PRT_INFO(" > SEGCP:BU_UPDATE:NETWORK - Firmware size: [%ld] bytes\r\n", fwupdate->fwup_size);
    }

    write_fw_len = 0;
    f_addr = FLASH_START_ADDR_BANK1_OFFSET;
    set_stop_dhcp_flag(1);
    close(SOCK_FWUPDATE);
    xTimerStart(reset_timer, 0);

    temp_buf = pvPortMalloc(FLASH_SECTOR_SIZE);
    memset(temp_buf, 0x00, FLASH_SECTOR_SIZE);

    recv_temp_buf = pvPortMalloc(FLASH_SECTOR_SIZE);
    memset(recv_temp_buf, 0x00, FLASH_SECTOR_SIZE);


    do {

#ifdef __USE_WATCHDOG__
        device_wdt_reset();
#endif
        recv_len = get_firmware_from_network(SOCK_FWUPDATE, recv_temp_buf);
        if (recv_len > 0) {
            xTimerReset(reset_timer, 0);
            if (buf_len + recv_len < FLASH_SECTOR_SIZE) {
                memcpy(temp_buf + buf_len, recv_temp_buf, recv_len);
                buf_len += recv_len;
            } else {
                //printf("f_addr = 0x%x\r\n", f_addr);
                remain_len = (buf_len + recv_len) - FLASH_SECTOR_SIZE;
                memcpy(temp_buf + buf_len, recv_temp_buf, recv_len - remain_len);

                PRT_INFO("Write_addr = 0x%08X\r\n", f_addr);
                write_flash(f_addr, (uint8_t *)temp_buf, FLASH_SECTOR_SIZE);
                f_addr += FLASH_SECTOR_SIZE;

                memset(temp_buf, 0xFF, FLASH_SECTOR_SIZE);
                memcpy(temp_buf, recv_temp_buf + (recv_len - remain_len), remain_len);
                buf_len = remain_len;
            }
            write_fw_len += recv_len;
        }
    } while (write_fw_len < fwupdate->fwup_size);
    set_stop_dhcp_flag(0);

    PRT_INFO("write_fw_len = %ld, fwup_size = %ld bytes\r\n", write_fw_len, fwupdate->fwup_size);
    if (write_fw_len == fwupdate->fwup_size) {
        if (buf_len > 0) {
            PRT_INFO("buf_len > 0, Write_addr = 0x%08X\r\n", f_addr);
            delay_ms(10);
            write_flash(f_addr, (uint8_t *)temp_buf, FLASH_SECTOR_SIZE);
        }

        PRT_INFO(" > SEGCP:BU_UPDATE:SUCCESS\r\n");

        fwupdate->fwup_copy_flag = 1;
        ret = DEVICE_FWUP_RET_SUCCESS;
    }
    vPortFree(temp_buf);
    vPortFree(recv_temp_buf);
    xTimerStop(reset_timer, 0);
    return ret;
}

int device_bank_copy(void) {
    struct __firmware_update *fwupdate = (struct __firmware_update *) & (get_DevConfig_pointer()->firmware_update);
    struct __serial_common *serial_common = (struct __serial_common *) & (get_DevConfig_pointer()->serial_common);

    uint32_t write_fw_len;
    uint32_t f_addr_src, f_addr_dst;

    if ((fwupdate->fwup_size == 0) || (fwupdate->fwup_size > FLASH_APP_BANK_SIZE)) {
        if (serial_common->serial_debug_en)
            PRT_INFO(" > SEGCP:BU_COPY:FAILED - Invalid firmware size: %ld bytes (Firmware size must be within %d bytes)\r\n",
                     fwupdate->fwup_size,
                     FLASH_APP_BANK_SIZE);
    }

    if (serial_common->serial_debug_en) {
        PRT_INFO(" > SEGCP:BU_COPY:NETWORK - Firmware size: [%ld] bytes\r\n", fwupdate->fwup_size);
    }

    f_addr_src = FLASH_START_ADDR_BANK1;
    f_addr_dst = FLASH_START_ADDR_BANK0_OFFSET;

    for (write_fw_len = 0; write_fw_len < (fwupdate->fwup_size + FLASH_SECTOR_SIZE); write_fw_len += FLASH_SECTOR_SIZE) {
        write_flash(f_addr_dst, (uint8_t *)f_addr_src, FLASH_SECTOR_SIZE);
        f_addr_dst += FLASH_SECTOR_SIZE;
        f_addr_src += FLASH_SECTOR_SIZE;
    }
    PRT_INFO("write_fw_len = %d, fwupdate->fwup_size = %d\r\n", write_fw_len, fwupdate->fwup_size);

    return 0;
}


int device_bank_check(uint8_t bank_num) {
    uint32_t fw_data;

    if (bank_num == 0) {
        fw_data = *(uint32_t *)(FLASH_START_ADDR_BANK0);
    } else if (bank_num == 1) {
        fw_data = *(uint32_t *)(FLASH_START_ADDR_BANK1);
    } else {
        return -1;
    }
    PRT_INFO("fw_data = 0x%08X\r\n", fw_data);

    if ((fw_data == 0xFFFFFFFF) || (fw_data == 0x00000000)) {
        return -1;
    }
    return 0;
}

uint16_t get_any_port(void) {
    if (any_port) {
        if (any_port < 0xffff) {
            any_port++;
        } else {
            any_port = 0;
        }
    }

    if (any_port == 0) {
        any_port = 50001;
    }

    return any_port;
}

uint16_t get_firmware_from_network(uint8_t sock, uint8_t * buf) {
    struct __firmware_update *fwupdate = (struct __firmware_update *) & (get_DevConfig_pointer()->firmware_update);
    uint8_t len_buf[2] = {0, };
    uint16_t len = 0;
    uint8_t state = getSn_SR(sock);

    static uint32_t recv_fwsize;

    switch (state) {
    case SOCK_INIT:
        //listen(sock);
        break;

    case SOCK_LISTEN:
        break;

    case SOCK_ESTABLISHED:
        if (getSn_IR(sock) & Sn_IR_CON) {
            setSn_IR(sock, Sn_IR_CON);
        }

        // DATA_BUF_SIZE
        if ((len = getSn_RX_RSR(sock)) > 0) {
            if (len > DATA_BUF_SIZE) {
                len = DATA_BUF_SIZE;
            }
            if (recv_fwsize + len > fwupdate->fwup_size) {
                len = fwupdate->fwup_size - recv_fwsize;    // remain
            }

            len = recv(sock, buf, len);
            recv_fwsize += len;
#ifdef _FWUP_DEBUG_
            printf(" > SEGCP:UPDATE:RECV_LEN - %d bytes | [%d] bytes\r\n", len, recv_fwsize);
#endif
            // Send ACK - receviced length - to configuration tool
            len_buf[0] = (uint8_t)((0xff00 & len) >> 8); // endian-independent code: Datatype translation, byte order regardless
            len_buf[1] = (uint8_t)(0x00ff & len);
            send(sock, len_buf, 2);

            if (recv_fwsize >= fwupdate->fwup_size) {
#ifdef _FWUP_DEBUG_
                printf(" > SEGCP:UPDATE:NETWORK - UPDATE END | [%d] bytes\r\n", recv_fwsize);
#endif
                // socket close
                disconnect(sock);
            }
        }
        break;

    case SOCK_CLOSE_WAIT:
        disconnect(sock);
        break;

    case SOCK_FIN_WAIT:
    case SOCK_CLOSED:
        if (socket(sock, Sn_MR_TCP, DEVICE_FWUP_PORT, SF_TCP_NODELAY) == sock) {
            recv_fwsize = 0;
            listen(sock);

#ifdef _FWUP_DEBUG_
            printf(" > SEGCP:UPDATE:SOCKOPEN\r\n");
#endif
        }
        break;

    default:
        break;
    }

    return len;
}


void display_Dev_Info_header(void) {
    DevConfig *dev_config = get_DevConfig_pointer();

    printf("\r\n");
    PRT_INFO("%s\r\n", STR_BAR);

    PRT_INFO(" %s \r\n", DEVICE_ID_DEFAULT); //PRT_INFO(" %s \r\n", dev_config->device_common.device_name);
    PRT_INFO(" >> WIZnet Device Server\r\n");

    PRT_INFO(" >> Firmware version: %d.%d.%d %s\r\n", dev_config->device_common.fw_ver[0],
             dev_config->device_common.fw_ver[1],
             dev_config->device_common.fw_ver[2],
             STR_VERSION_STATUS);
    PRT_INFO("%s\r\n", STR_BAR);
}

// Only for Serial 1-channel device
void display_Dev_Info_main(void) {
    uint8_t serial_mode;
    DevConfig *dev_config = get_DevConfig_pointer();

    PRT_INFO(" - System clock: %lu Hz\r\n", clock_get_hz(clk_sys));
    PRT_INFO(" - Peri clock: %lu Hz\r\n", clock_get_hz(clk_peri));
    PRT_INFO(" - Device type: %s\r\n", dev_config->device_common.device_name);
    PRT_INFO(" - Device name: %s\r\n", dev_config->device_option.device_alias);
    PRT_INFO(" - Device group: %s\r\n", dev_config->device_option.device_group);

    PRT_INFO(" - 0 Ch Device mode: %s\r\n", str_working[dev_config->network_connection[0].working_mode]);
    PRT_INFO(" - 1 Ch Device mode: %s\r\n", str_working[dev_config->network_connection[1].working_mode]);

    PRT_INFO(" - 0 Ch Serial %s mode\r\n", (uart_if_table[dev_config->serial_option[0].uart_interface]));
    PRT_INFO(" - 1 Ch Serial %s mode\r\n", (uart_if_table[dev_config->serial_option[1].uart_interface]));
    PRT_INFO(" - Network settings: \r\n");

    PRT_INFO("\t- Obtaining IP settings: [%s]\r\n", (dev_config->network_option.dhcp_use == 1) ? "Automatic - DHCP" : "Static");
    PRT_INFO("\t- TCP/UDP ports\r\n");
    PRT_INFO("\t   + 0 Ch S2E data port: [%d]\r\n", dev_config->network_connection[0].local_port);
    PRT_INFO("\t   + 1 Ch S2E data port: [%d]\r\n", dev_config->network_connection[1].local_port);
    PRT_INFO("\t   + TCP/UDP setting port: [%d]\r\n", DEVICE_SEGCP_PORT);
    PRT_INFO("\t   + Firmware update port: [%d]\r\n", DEVICE_FWUP_PORT);
    PRT_INFO("\t- TCP Retransmission retry: [%d]\r\n", getRCR());

    PRT_INFO(" - Search ID code: \r\n");
    PRT_INFO("\t- %s: [%s]\r\n", (dev_config->config_common.pw_search != 0) ? "Enabled" : "Disabled", (dev_config->config_common.pw_search != 0) ? dev_config->config_common.pw_search : "None");

    PRT_INFO(" - 0 Ch Ethernet connection password: \r\n");
    PRT_INFO("\t- %s %s\r\n", (dev_config->tcp_option[0].pw_connect_en == 1) ? "Enabled" : "Disabled", "(TCP server / mixed mode only)");

    PRT_INFO(" - 1 Ch Ethernet connection password: \r\n");
    PRT_INFO("\t- %s %s\r\n", (dev_config->tcp_option[1].pw_connect_en == 1) ? "Enabled" : "Disabled", "(TCP server / mixed mode only)");

    PRT_INFO(" - Connection timer settings: \r\n");
    PRT_INFO("\t- 0 Ch Inactivity timer: ");
    if (dev_config->tcp_option[0].inactivity) {
        PRT_INFO("[%d] (sec)\r\n", dev_config->tcp_option[0].inactivity);
    } else {
        PRT_INFO("%s\r\n", STR_DISABLED);
    }
    PRT_INFO("\t- 0 Ch Reconnect interval: ");
    if (dev_config->tcp_option[0].reconnection) {
        PRT_INFO("[%d] (msec)\r\n", dev_config->tcp_option[0].reconnection);
    } else {
        PRT_INFO("%s\r\n", STR_DISABLED);
    }

    PRT_INFO("\t- 1 Ch Inactivity timer: ");
    if (dev_config->tcp_option[1].inactivity) {
        PRT_INFO("[%d] (sec)\r\n", dev_config->tcp_option[1].inactivity);
    } else {
        PRT_INFO("%s\r\n", STR_DISABLED);
    }
    PRT_INFO("\t- 1 Ch Reconnect interval: ");
    if (dev_config->tcp_option[1].reconnection) {
        PRT_INFO("[%d] (msec)\r\n", dev_config->tcp_option[1].reconnection);
    } else {
        PRT_INFO("%s\r\n", STR_DISABLED);
    }

    for (int i = 0; i < DEVICE_UART_CNT; i++) {
        //todo:
        PRT_INFO(" - %d CH Serial settings: \r\n", i);
        PRT_INFO("\t- Communication Protocol: ");
        serial_mode = get_serial_communation_protocol(i);
        if (serial_mode) {
            PRT_INFO("[%s]\r\n", (serial_mode == SEG_SERIAL_MODBUS_RTU) ? STR_MODBUS_RTU : STR_MODBUS_ASCII);
        } else {
            PRT_INFO("[%s]\r\n", STR_DISABLED);
        }

        PRT_INFO("\t- Data %s port:\r\n", STR_UART);
        PRT_INFO("\t   + UART IF: [%s]\r\n", uart_if_table[dev_config->serial_option[i].uart_interface]);
        printf("\t   + %ld-", baud_table[dev_config->serial_option[i].baud_rate]);
        printf("%d-", word_len_table[dev_config->serial_option[i].data_bits]);
        printf("%s-", parity_table[dev_config->serial_option[i].parity]);
        printf("%d / ", stop_bit_table[dev_config->serial_option[i].stop_bits]);
        if (dev_config->serial_option[i].uart_interface == UART_IF_RS232_TTL) {
            printf("Flow control: %s", flow_ctrl_table[dev_config->serial_option[i].flow_control]);
        } else {
            if ((dev_config->serial_option[i].flow_control == flow_rtsonly) || (dev_config->serial_option[i].flow_control == flow_reverserts)) {
                printf("Flow control: %s", flow_ctrl_table[dev_config->serial_option[i].flow_control]);
            } else {
                printf("Flow control: %s", flow_ctrl_table[0]); // RS-422/485; flow control - NONE only
            }
        }
        PRT_INFO("\r\n");

        PRT_INFO(" - Serial data packing options:\r\n");
        PRT_INFO("\t- Time: ");
        if (dev_config->serial_data_packing[i].packing_time) {
            PRT_INFO("[%d] (msec)\r\n", dev_config->serial_data_packing[i].packing_time);
        } else {
            PRT_INFO("%s\r\n", STR_DISABLED);
        }
        PRT_INFO("\t- Size: ");
        if (dev_config->serial_data_packing[i].packing_size) {
            PRT_INFO("[%d] (bytes)\r\n", dev_config->serial_data_packing[i].packing_size);
        } else {
            PRT_INFO("%s\r\n", STR_DISABLED);
        }
        PRT_INFO("\t- Char: ");
        if (dev_config->serial_data_packing[i].packing_delimiter_length == 1) {
            PRT_INFO("[%.2X] (hex only)\r\n", dev_config->serial_data_packing[i].packing_delimiter[0]);
        } else {
            PRT_INFO("%s\r\n", STR_DISABLED);
        }

        PRT_INFO(" - Serial command mode switch code:\r\n");
        PRT_INFO("\t- %s\r\n", (dev_config->serial_command.serial_command == 1) ? STR_ENABLED : STR_DISABLED);
        PRT_INFO("\t- [%.2X][%.2X][%.2X] (Hex only)\r\n",
                 dev_config->serial_command.serial_trigger[0],
                 dev_config->serial_command.serial_trigger[1],
                 dev_config->serial_command.serial_trigger[2]);
    }
    PRT_INFO("\t- Debug %s port:\r\n", STR_UART);
    PRT_INFO("\t   + %s / %s %s\r\n", "921600-8-N-1", "NONE", "(fixed)");


#ifdef __USE_USERS_GPIO__ // not used
    PRT_INFO(" - Hardware information: User I/O pins\r\n");
    PRT_INFO("\t- UserIO A: [%s] - %s / %s\r\n", "%s", USER_IO_TYPE_STR[get_user_io_type(USER_IO_SEL[0])], USER_IO_DIR_STR[get_user_io_direction(USER_IO_SEL[0])], USER_IO_PIN_STR[0]);
    PRT_INFO("\t- UserIO B: [%s] - %s / %s\r\n", "%s", USER_IO_TYPE_STR[get_user_io_type(USER_IO_SEL[1])], USER_IO_DIR_STR[get_user_io_direction(USER_IO_SEL[1])], USER_IO_PIN_STR[1]);
#endif

    PRT_INFO("%s\r\n", STR_BAR);
}


void display_Dev_Info_dhcp(void) {
    DevConfig *dev_config = get_DevConfig_pointer();

    if (dev_config->network_option.dhcp_use) {
        if (flag_process_dhcp_success == ON) {
            PRT_INFO(" # DHCP IP Leased time : %ld seconds\r\n", getDHCPLeasetime());
        } else {
            PRT_INFO(" # DHCP Failed\r\n");
        }
    }
}


void display_Dev_Info_dns(int channel) {
    DevConfig *dev_config = get_DevConfig_pointer();

    if (dev_config->network_connection[channel].dns_use) {
        if (flag_process_dns_success[channel] == ON) {
            PRT_INFO(" # DNS: %s => %d.%d.%d.%d : %d\r\n", dev_config->network_connection[channel].dns_domain_name,
                     dev_config->network_connection[channel].remote_ip[0],
                     dev_config->network_connection[channel].remote_ip[1],
                     dev_config->network_connection[channel].remote_ip[2],
                     dev_config->network_connection[channel].remote_ip[3],
                     dev_config->network_connection[channel].remote_port);
        } else {
            PRT_INFO(" # DNS Failed\r\n");
        }
    }
}


#ifdef __USE_WATCHDOG__
void wdt_reset(void) {
    //Reload the Watchdog time counter
    //__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}
#endif
