
#include <string.h>
#include <stdlib.h>
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
    process_socket_termination(SEG_DATA0_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA0_CH);
    process_socket_termination(SEG_DATA1_SOCK, SOCK_TERMINATION_DELAY, SEG_DATA1_CH);

    for (int i = SEG_DATA1_SOCK + 1; i < _WIZCHIP_SOCK_NUM_; i++) {
        close(i);
    }
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

void device_wdt_reset(void) {
    if (get_reset_flag() == 0) {
        watchdog_update();
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

    do {

#ifdef __USE_WATCHDOG__
        device_wdt_reset();
#endif
        recv_len = get_firmware_from_network(SOCK_FWUPDATE, g_recv_buf[SEG_DATA0_CH]);
        if (recv_len > 0) {
            xTimerReset(reset_timer, 0);
            if (buf_len + recv_len < FLASH_SECTOR_SIZE) {
                memcpy(temp_buf + buf_len, g_recv_buf[SEG_DATA0_CH], recv_len);
                buf_len += recv_len;
            } else {
                //printf("f_addr = 0x%x\r\n", f_addr);
                remain_len = (buf_len + recv_len) - FLASH_SECTOR_SIZE;
                memcpy(temp_buf + buf_len, g_recv_buf[SEG_DATA0_CH], recv_len - remain_len);

                PRT_INFO("Write_addr = 0x%08X\r\n", f_addr);
                write_flash(f_addr, (uint8_t *)temp_buf, FLASH_SECTOR_SIZE);
                f_addr += FLASH_SECTOR_SIZE;

                memset(temp_buf, 0xFF, FLASH_SECTOR_SIZE);
                memcpy(temp_buf, g_recv_buf[SEG_DATA0_CH] + (recv_len - remain_len), remain_len);
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
        } else if ((dev_config->serial_option[i].uart_interface == UART_IF_RS422) || (dev_config->serial_option[i].uart_interface == UART_IF_RS485)) {
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
        if (flag_process_dns_success == ON) {
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

void devConfig_print_all(void) {
    DevConfig *dev_config = get_DevConfig_pointer();
    int i, j;

    printf("\r\n===== Device Configuration Information with SEGCP Command =====\r\n");

    // 1. Device Common Information
    printf("\r\n[device_common]\r\n");
    printf("fw_ver (VR): %d.%d.%d\r\n", dev_config->device_common.fw_ver[0],
           dev_config->device_common.fw_ver[1], dev_config->device_common.fw_ver[2]);
    printf("device_type (MN): %d.%d.%d\r\n", dev_config->device_common.device_type[0],
           dev_config->device_common.device_type[1], dev_config->device_common.device_type[2]);
    printf("device_name (DH): %s\r\n", dev_config->device_common.device_name);
    printf("device_mode: %d\r\n", dev_config->device_common.device_mode);

    // 2. Config Common Information
    printf("\r\n[config_common]\r\n");
    printf("app_protocol: %d\r\n", dev_config->config_common.app_protocol);
    printf("packet_size: %d\r\n", dev_config->config_common.packet_size);
    printf("pw_search (SP): %s\r\n", dev_config->config_common.pw_search);

    // 3. Network Common Information
    printf("\r\n[network_common]\r\n");
    printf("mac (MC): %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           dev_config->network_common.mac[0], dev_config->network_common.mac[1],
           dev_config->network_common.mac[2], dev_config->network_common.mac[3],
           dev_config->network_common.mac[4], dev_config->network_common.mac[5]);
    printf("local_ip (LI): %d.%d.%d.%d\r\n",
           dev_config->network_common.local_ip[0], dev_config->network_common.local_ip[1],
           dev_config->network_common.local_ip[2], dev_config->network_common.local_ip[3]);
    printf("gateway (GW): %d.%d.%d.%d\r\n",
           dev_config->network_common.gateway[0], dev_config->network_common.gateway[1],
           dev_config->network_common.gateway[2], dev_config->network_common.gateway[3]);
    printf("subnet (SM): %d.%d.%d.%d\r\n",
           dev_config->network_common.subnet[0], dev_config->network_common.subnet[1],
           dev_config->network_common.subnet[2], dev_config->network_common.subnet[3]);

    // 4. Network Connection Information
    printf("\r\n[network_connection]\r\n");
    for (i = 0; i < DEVICE_UART_CNT; i++) {
        printf("UART%d:\r\n", i);
        if (i == 0) {
            printf("  working_mode (OP): %d\r\n", dev_config->network_connection[i].working_mode);
            printf("  working_state (ST): %d\r\n", dev_config->network_connection[i].working_state);
            printf("  local_port (LP): %d\r\n", dev_config->network_connection[i].local_port);
            printf("  remote_port (RP): %d\r\n", dev_config->network_connection[i].remote_port);
            printf("  remote_ip/host (RH): %d.%d.%d.%d / %s\r\n",
                   dev_config->network_connection[i].remote_ip[0], dev_config->network_connection[i].remote_ip[1],
                   dev_config->network_connection[i].remote_ip[2], dev_config->network_connection[i].remote_ip[3],
                   dev_config->network_connection[i].dns_domain_name);
        } else {
            printf("  working_mode (AO): %d\r\n", dev_config->network_connection[i].working_mode);
            printf("  working_state (QS): %d\r\n", dev_config->network_connection[i].working_state);
            printf("  local_port (QL): %d\r\n", dev_config->network_connection[i].local_port);
            printf("  remote_port (AP): %d\r\n", dev_config->network_connection[i].remote_port);
            printf("  remote_ip/host (QH): %d.%d.%d.%d / %s\r\n",
                   dev_config->network_connection[i].remote_ip[0], dev_config->network_connection[i].remote_ip[1],
                   dev_config->network_connection[i].remote_ip[2], dev_config->network_connection[i].remote_ip[3],
                   dev_config->network_connection[i].dns_domain_name);
        }
        printf("  fixed_local_port: %d\r\n", dev_config->network_connection[i].fixed_local_port);
        printf("  dns_use: %d\r\n", dev_config->network_connection[i].dns_use);
    }

    // 5. Network Option Information
    printf("\r\n[network_option]\r\n");
    printf("dhcp_use (IM): %d\r\n", dev_config->network_option.dhcp_use);
    printf("dns_server_ip (DS): %d.%d.%d.%d\r\n",
           dev_config->network_option.dns_server_ip[0], dev_config->network_option.dns_server_ip[1],
           dev_config->network_option.dns_server_ip[2], dev_config->network_option.dns_server_ip[3]);
    printf("tcp_rcr_val (TR): %d\r\n", dev_config->network_option.tcp_rcr_val);

    // 6. TCP Option Information
    printf("\r\n[tcp_option]\r\n");
    for (i = 0; i < DEVICE_UART_CNT; i++) {
        printf("UART%d:\r\n", i);
        if (i == 0) {
            printf("  inactivity (IT): %d\r\n", dev_config->tcp_option[i].inactivity);
            printf("  reconnection (RI): %d\r\n", dev_config->tcp_option[i].reconnection);
            printf("  keepalive_en (KA): %d\r\n", dev_config->tcp_option[i].keepalive_en);
            printf("  keepalive_wait_time (KI): %d\r\n", dev_config->tcp_option[i].keepalive_wait_time);
            printf("  keepalive_retry_time (KE): %d\r\n", dev_config->tcp_option[i].keepalive_retry_time);
            printf("  pw_connect (NP): %s\r\n", dev_config->tcp_option[i].pw_connect);
            printf("  pw_connect_en (CP): %d\r\n", dev_config->tcp_option[i].pw_connect_en);
        } else {
            printf("  inactivity (RV): %d\r\n", dev_config->tcp_option[i].inactivity);
            printf("  reconnection (RR): %d\r\n", dev_config->tcp_option[i].reconnection);
            printf("  keepalive_en (RA): %d\r\n", dev_config->tcp_option[i].keepalive_en);
            printf("  keepalive_wait_time (RS): %d\r\n", dev_config->tcp_option[i].keepalive_wait_time);
            printf("  keepalive_retry_time (RE): %d\r\n", dev_config->tcp_option[i].keepalive_retry_time);
            printf("  pw_connect: %s\r\n", dev_config->tcp_option[i].pw_connect);
            printf("  pw_connect_en: %d\r\n", dev_config->tcp_option[i].pw_connect_en);
        }
    }

    // 7. Serial Common Information
    printf("\r\n[serial_common]\r\n");
    printf("uart_interface_cnt: %d\r\n", dev_config->serial_common.uart_interface_cnt);
    printf("serial_debug_en (DG): %d\r\n", dev_config->serial_common.serial_debug_en);

    // 8. Serial Command Information
    printf("\r\n[serial_command]\r\n");
    printf("serial_command (TE): %d\r\n", dev_config->serial_command.serial_command);
    printf("serial_trigger (SS): %02X %02X %02X\r\n",
           dev_config->serial_command.serial_trigger[0],
           dev_config->serial_command.serial_trigger[1],
           dev_config->serial_command.serial_trigger[2]);
    printf("serial_command_echo (EC): %d\r\n", dev_config->serial_command.serial_command_echo);

    // 9. Serial Option Information
    printf("\r\n[serial_option]\r\n");
    for (i = 0; i < DEVICE_UART_CNT; i++) {
        printf("UART%d:\r\n", i);
        if (i == 0) {
            printf("  uart_interface (UI/UN): %d\r\n", dev_config->serial_option[i].uart_interface);
            printf("  protocol (PO): %d\r\n", dev_config->serial_option[i].protocol);
            printf("  baud_rate (BR): %d\r\n", dev_config->serial_option[i].baud_rate);
            printf("  data_bits (DB): %d\r\n", dev_config->serial_option[i].data_bits);
            printf("  parity (PR): %d\r\n", dev_config->serial_option[i].parity);
            printf("  stop_bits (SB): %d\r\n", dev_config->serial_option[i].stop_bits);
            printf("  flow_control (FL): %d\r\n", dev_config->serial_option[i].flow_control);
            printf("  dtr_en (SC): %d\r\n", dev_config->serial_option[i].dtr_en);
            printf("  dsr_en (SC): %d\r\n", dev_config->serial_option[i].dsr_en);
        } else {
            printf("  uart_interface (EI/EN): %d\r\n", dev_config->serial_option[i].uart_interface);
            printf("  protocol (EO): %d\r\n", dev_config->serial_option[i].protocol);
            printf("  baud_rate (EB): %d\r\n", dev_config->serial_option[i].baud_rate);
            printf("  data_bits (ED): %d\r\n", dev_config->serial_option[i].data_bits);
            printf("  parity (EP): %d\r\n", dev_config->serial_option[i].parity);
            printf("  stop_bits (ES): %d\r\n", dev_config->serial_option[i].stop_bits);
            printf("  flow_control (EF): %d\r\n", dev_config->serial_option[i].flow_control);
            printf("  dtr_en: %d\r\n", dev_config->serial_option[i].dtr_en);
            printf("  dsr_en: %d\r\n", dev_config->serial_option[i].dsr_en);
        }
    }

    // 10. Serial Data Packing Information
    printf("\r\n[serial_data_packing]\r\n");
    for (i = 0; i < DEVICE_UART_CNT; i++) {
        printf("UART%d:\r\n", i);
        if (i == 0) {
            printf("  packing_time (PT): %d\r\n", dev_config->serial_data_packing[i].packing_time);
            printf("  packing_size (PS): %d\r\n", dev_config->serial_data_packing[i].packing_size);
            printf("  packing_delimiter (PD): ");
            for (j = 0; j < dev_config->serial_data_packing[i].packing_delimiter_length; j++) {
                printf("%02X ", dev_config->serial_data_packing[i].packing_delimiter[j]);
            }
            printf("\r\n");
            printf("  packing_delimiter_length: %d\r\n", dev_config->serial_data_packing[i].packing_delimiter_length);
            printf("  packing_data_appendix: %d\r\n", dev_config->serial_data_packing[i].packing_data_appendix);
        } else {
            printf("  packing_time (AT): %d\r\n", dev_config->serial_data_packing[i].packing_time);
            printf("  packing_size (NS): %d\r\n", dev_config->serial_data_packing[i].packing_size);
            printf("  packing_delimiter (ND): ");
            for (j = 0; j < dev_config->serial_data_packing[i].packing_delimiter_length; j++) {
                printf("%02X ", dev_config->serial_data_packing[i].packing_delimiter[j]);
            }
            printf("\r\n");
            printf("  packing_delimiter_length: %d\r\n", dev_config->serial_data_packing[i].packing_delimiter_length);
            printf("  packing_data_appendix: %d\r\n", dev_config->serial_data_packing[i].packing_data_appendix);
        }
    }

    // 11. User IO Information
    printf("\r\n[user_io_info]\r\n");
    printf("user_io_enable: %d\r\n", dev_config->user_io_info.user_io_enable);
    printf("user_io_type (GA/GB/CA/CB): %d\r\n", dev_config->user_io_info.user_io_type);
    printf("user_io_direction (CA/CB): %d\r\n", dev_config->user_io_info.user_io_direction);
    printf("user_io_status (GA/GB): %d\r\n", dev_config->user_io_info.user_io_status);

    // 12. Firmware Update Information
    printf("\r\n[firmware_update]\r\n");
    printf("fwup_flag: %d\r\n", dev_config->firmware_update.fwup_flag);
    printf("fwup_port: %d\r\n", dev_config->firmware_update.fwup_port);
    printf("fwup_size (FW): %d\r\n", dev_config->firmware_update.fwup_size);
    printf("fwup_server_flag: %d\r\n", dev_config->firmware_update.fwup_server_flag);
    printf("fwup_server_port: %d\r\n", dev_config->firmware_update.fwup_server_port);
    printf("fwup_copy_flag (UF): %d\r\n", dev_config->firmware_update.fwup_copy_flag);

    // 13. SSL Option Information
    printf("\r\n[ssl_option]\r\n");
    for (i = 0; i < DEVICE_UART_CNT; i++) {
        printf("UART%d:\r\n", i);
        if (i == 0) {
            printf("  root_ca_option (RC): %d\r\n", dev_config->ssl_option[i].root_ca_option);
            printf("  client_cert_enable (CE): %d\r\n", dev_config->ssl_option[i].client_cert_enable);
            printf("  rootca_len (OC): %d\r\n", dev_config->ssl_option[i].rootca_len);
            printf("  clica_len (LC): %d\r\n", dev_config->ssl_option[i].clica_len);
            printf("  pkey_len (PK): %d\r\n", dev_config->ssl_option[i].pkey_len);
            printf("  recv_timeout (SO): %d\r\n", dev_config->ssl_option[i].recv_timeout);
        } else {
            printf("  root_ca_option: %d\r\n", dev_config->ssl_option[i].root_ca_option);
            printf("  client_cert_enable: %d\r\n", dev_config->ssl_option[i].client_cert_enable);
            printf("  rootca_len: %d\r\n", dev_config->ssl_option[i].rootca_len);
            printf("  clica_len: %d\r\n", dev_config->ssl_option[i].clica_len);
            printf("  pkey_len: %d\r\n", dev_config->ssl_option[i].pkey_len);
            printf("  recv_timeout (RO): %d\r\n", dev_config->ssl_option[i].recv_timeout);
        }
    }

    // 14. MQTT Option Information
    printf("\r\n[mqtt_option]\r\n");
    for (i = 0; i < DEVICE_UART_CNT; i++) {
        printf("UART%d:\r\n", i);
        if (i == 0) {
            printf("  pub_topic (PU): %s\r\n", dev_config->mqtt_option[i].pub_topic);
            printf("  sub_topic_0 (U0): %s\r\n", dev_config->mqtt_option[i].sub_topic_0);
            printf("  sub_topic_1 (U1): %s\r\n", dev_config->mqtt_option[i].sub_topic_1);
            printf("  sub_topic_2 (U2): %s\r\n", dev_config->mqtt_option[i].sub_topic_2);
            printf("  user_name (QU): %s\r\n", dev_config->mqtt_option[i].user_name);
            printf("  client_id (QC): %s\r\n", dev_config->mqtt_option[i].client_id);
            printf("  password (QP): %s\r\n", dev_config->mqtt_option[i].password);
            printf("  keepalive (QK): %d\r\n", dev_config->mqtt_option[i].keepalive);
            printf("  qos (QO): %d\r\n", dev_config->mqtt_option[i].qos);
        } else {
            printf("  pub_topic: %s\r\n", dev_config->mqtt_option[i].pub_topic);
            printf("  sub_topic_0: %s\r\n", dev_config->mqtt_option[i].sub_topic_0);
            printf("  sub_topic_1: %s\r\n", dev_config->mqtt_option[i].sub_topic_1);
            printf("  sub_topic_2: %s\r\n", dev_config->mqtt_option[i].sub_topic_2);
            printf("  user_name: %s\r\n", dev_config->mqtt_option[i].user_name);
            printf("  client_id: %s\r\n", dev_config->mqtt_option[i].client_id);
            printf("  password: %s\r\n", dev_config->mqtt_option[i].password);
            printf("  keepalive: %d\r\n", dev_config->mqtt_option[i].keepalive);
            printf("  qos: %d\r\n", dev_config->mqtt_option[i].qos);
        }
    }

    // 15. Device Option Information
    printf("\r\n[device_option]\r\n");
    printf("pw_setting_en: %d\r\n", dev_config->device_option.pw_setting_en);
    printf("pw_setting: %s\r\n", dev_config->device_option.pw_setting);
    printf("device_alias: %s\r\n", dev_config->device_option.device_alias);
    printf("device_group: %s\r\n", dev_config->device_option.device_group);

    for (i = 0; i < DEVICE_UART_CNT; i++) {
        printf("UART%d:\r\n", i);
        if (i == 0) {
            printf("  device_serial_connect_data (SD): %s\r\n", dev_config->device_option.device_serial_connect_data[i]);
            printf("  device_serial_disconnect_data (DD): %s\r\n", dev_config->device_option.device_serial_disconnect_data[i]);
            printf("  device_eth_connect_data (SE): %s\r\n", dev_config->device_option.device_eth_connect_data[i]);
        } else {
            printf("  device_serial_connect_data (RD): %s\r\n", dev_config->device_option.device_serial_connect_data[i]);
            printf("  device_serial_disconnect_data (RF): %s\r\n", dev_config->device_option.device_serial_disconnect_data[i]);
            printf("  device_eth_connect_data (EE): %s\r\n", dev_config->device_option.device_eth_connect_data[i]);
        }
    }

    // 16. Device Config Version
    printf("\r\n[devConfigVer]\r\n");
    printf("devConfigVer: %d\r\n", dev_config->devConfigVer);

    printf("\r\n======================================\r\n");
}


#ifdef __USE_WATCHDOG__
void wdt_reset(void) {
    //Reload the Watchdog time counter
    //__HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}
#endif

