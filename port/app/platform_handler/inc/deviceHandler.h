#ifndef DEVICEHANDLER_H_
#define DEVICEHANDLER_H_

#include <stdint.h>
#include "WIZnet_board.h"
#include "storageHandler.h"
#include "port_common.h"

/* Debug message enable */
#define _FWUP_DEBUG_

/* Application Port */
#define DEVICE_SEGCP_PORT     50001 // Search / Setting Port (UDP Broadcast / TCP unicast)
#define DEVICE_FWUP_PORT      50002 // Firmware Update Port
#define DEVICE_HTTP_PORT			50003
#define DEVICE_DDNS_PORT      3030  // Not  used

#define FLASH_SIZE                0x00200000
#define FLASH_BOOTLOADER_SIZE     0x20000  //128KB
#define FLASH_PARAMETER_SIZE      0x10000
#define FLASH_APP_BANK_SIZE       0x80000  //512KB

#define FLASH_START_ADDR_BANK0_OFFSET    FLASH_BOOTLOADER_SIZE
#define FLASH_START_ADDR_BANK1_OFFSET    FLASH_START_ADDR_BANK0_OFFSET + FLASH_APP_BANK_SIZE

#define FLASH_START_ADDR_BANK0    XIP_BASE + FLASH_BOOTLOADER_SIZE
#define FLASH_START_ADDR_BANK1    FLASH_START_ADDR_BANK0 + FLASH_APP_BANK_SIZE

#define FLASH_END_ADDR (XIP_BASE + FLASH_SIZE - 1)

#define FLASH_DEV_INFO_ADDR FLASH_START_ADDR_BANK1_OFFSET + FLASH_APP_BANK_SIZE
#define FLASH_ROOTCA0_ADDR   FLASH_DEV_INFO_ADDR + 0x1000
#define FLASH_CLICA0_ADDR    FLASH_ROOTCA0_ADDR + 0x1000
#define FLASH_PRIKEY0_ADDR   FLASH_CLICA0_ADDR + 0x1000
#define FLASH_MAC_ADDR      FLASH_PRIKEY0_ADDR + 0x1000

#define FLASH_ROOTCA1_ADDR   FLASH_MAC_ADDR + 0x1000
#define FLASH_CLICA1_ADDR    FLASH_ROOTCA1_ADDR + 0x1000
#define FLASH_PRIKEY1_ADDR   FLASH_CLICA1_ADDR + 0x1000

#define DEVICE_APP_SIZE         FLASH_APP_BANK_SIZE

#define DEVICE_BOOT_ADDR          (0)
#define DEVICE_APP_MAIN_ADDR      (DEVICE_BOOT_ADDR + DEVICE_BOOT_SIZE)
#define DEVICE_APP_BACKUP_ADDR    (DEVICE_APP_MAIN_ADDR + DEVICE_APP_SIZE)
#define DEVICE_CONFIG_ADDR        (FLASH_DEV_INFO_ADDR)
#define DEVICE_MAC_ADDR           (FLASH_MAC_ADDR)


/* Defines for firmware update */
#define DEVICE_FWUP_SIZE        DEVICE_APP_SIZE // Firmware size - 50kB MAX
#define DEVICE_FWUP_TIMEOUT     20000 // 20 secs.

#define DEVICE_FWUP_RET_SUCCESS   0x80
#define DEVICE_FWUP_RET_FAILED    0x40
#define DEVICE_FWUP_RET_PROGRESS  0x20
#define DEVICE_FWUP_RET_NONE      0x00

#define DEVICE_WDT_TIMEOUT    30000 // 30 secs

void device_set_factory_default(void);
void device_socket_termination(void);
void device_reboot(void);
void device_raw_reboot(void);
void device_wdt_reset(void);

// A watchdog reset takes RAM with it, so nothing survives to say what the device
// was doing when it stopped. These four scratch registers do survive - the SDK
// only claims scratch[4] upwards - and carry a breadcrumb and a fault record
// across the reset, which the next boot prints.
#define SEG_PM_TASK_NONE    0U
#define SEG_PM_TASK_U2E     1U
#define SEG_PM_TASK_RECV    2U
#define SEG_PM_TASK_SEG     3U
#define SEG_PM_TASK_PIO_RX  4U

#define SEG_PM_REASON_NONE      0U
#define SEG_PM_REASON_HARDFAULT 1U
#define SEG_PM_REASON_STACK     2U
#define SEG_PM_REASON_SOCKLOCK  3U
#define SEG_PM_REASON_WDT_GAP   4U

// Under the 8.388 s watchdog, leaving room for the record to be written and for
// the reset to be attributed to the holder that caused it rather than the next.
#define SEG_SOCKET_LOCK_STUCK_MS 5000U

// Largest observed gap between watchdog feeds. The deadline is 8.388 s, so a run
// that never resets still shows how close it came.
uint32_t device_wdt_max_gap_ms(void);
uint32_t device_wdt_since_feed_ms(void);
uint32_t device_wdt_max_gap_at_ms(void);

void seg_postmortem_init(void);
void seg_postmortem_report(void);
void seg_postmortem_mark(uint8_t task_id, uint8_t channel);
void seg_postmortem_record(uint32_t reason, uint32_t detail);
void reset_timer_callback(TimerHandle_t xTimer);
uint8_t get_reset_flag(void);
//void disable_interrupts(void);
void reset_peripherals(void);
void jump_to_app(uint32_t app_addr);

void display_Dev_Info_main(void);
void display_Dev_Info_dhcp(void);
void display_Dev_Info_dns(int channel);

int device_bank_check(uint8_t bank_num);
int device_bank_copy(void);
uint8_t device_bank_update(void);
uint8_t device_firmware_update(teDATASTORAGE stype); // Firmware update by Configuration tool / Flash to Flash

// function for timer
void device_timer_msec(void);

#endif /* DEVICEHANDLER_H_ */
