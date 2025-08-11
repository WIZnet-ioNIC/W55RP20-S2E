
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "WIZnet_board.h"
#include "flashHandler.h"
#include "deviceHandler.h"
#include "storageHandler.h"

void read_storage(teDATASTORAGE stype, void *data, uint16_t size) {
    switch (stype) {
    case STORAGE_MAC:
        read_flash(FLASH_MAC_ADDR, data, size);
        break;

    case STORAGE_CONFIG:
        read_flash(FLASH_DEV_INFO_ADDR, data, size);
        break;
    default:
        break;
    }
}


void write_storage(teDATASTORAGE stype, uint32_t addr, void *data, uint16_t size) {
    switch (stype) {
    case STORAGE_MAC:
        write_flash(FLASH_MAC_ADDR, data, size);
        break;

    case STORAGE_CONFIG:
        write_flash(FLASH_DEV_INFO_ADDR, data, size);
        break;

    case STORAGE_APPBOOT:
        write_flash(addr, data, size);
        break;

    case STORAGE_ROOTCA:
        write_flash(FLASH_ROOTCA_ADDR, data, size);
        break;

    case STORAGE_CLICA:
        write_flash(FLASH_CLICA_ADDR, data, size);
        break;

    case STORAGE_PKEY:
        write_flash(FLASH_PRIKEY_ADDR, data, size);
        break;

    default:
        break;
    }
}

void erase_storage(teDATASTORAGE stype) {
    uint16_t i;
    uint32_t address, working_address;
    uint32_t sectors = 0;

    switch (stype) {
    case STORAGE_MAC:
        printf("can't erase MAC in f/w\r\n");
        break;

    case STORAGE_CONFIG:
        erase_flash_sector(FLASH_DEV_INFO_ADDR);
        break;

    case STORAGE_APPBOOT:
        address = DEVICE_BOOT_ADDR;
        break;

    case STORAGE_APPBANK:
        address = FLASH_START_ADDR_BANK0_OFFSET;
        break;

    case STORAGE_BINBANK:
        address = FLASH_START_ADDR_BANK1_OFFSET;
        break;

    case STORAGE_ROOTCA:
        erase_flash_sector(FLASH_ROOTCA_ADDR);
        break;
    case STORAGE_CLICA:
        erase_flash_sector(FLASH_CLICA_ADDR);
    default:
        break;
    }

    if ((stype == STORAGE_APPBANK) || (stype == STORAGE_BINBANK)) {
        if (stype == STORAGE_APPBANK) {
            PRT_INFO(" > STORAGE:ERASE_START APP Bank\r\n");
        } else if (stype == STORAGE_BINBANK) {
            PRT_INFO(" > STORAGE:ERASE_START BIN Bank\r\n");
        }
        working_address = address;
        sectors = FLASH_APP_BANK_SIZE / FLASH_SECTOR_SIZE;

        for (i = 0; i < sectors; i++) {
            erase_flash_sector(working_address);
            working_address += FLASH_SECTOR_SIZE;
        }
        working_address += (sectors * FLASH_SECTOR_SIZE);
        PRT_INFO(" > STORAGE:ERASE_END:ADDR_RANGE - [0x%x ~ 0x%x]\r\n", address, working_address - 1);
    }
#ifdef __USE_WATCHDOG__
    device_wdt_reset();
#endif
}
