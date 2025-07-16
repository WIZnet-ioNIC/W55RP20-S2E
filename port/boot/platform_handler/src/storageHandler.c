
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "WIZnet_board.h"
#include "flashHandler.h"
#include "deviceHandler.h"
#include "storageHandler.h"

void read_storage(teDATASTORAGE stype, void *data, uint16_t size)
{
    switch(stype)
    {
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


void write_storage(teDATASTORAGE stype, uint32_t addr, void *data, uint16_t size)
{
    switch(stype)
    {
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

void erase_storage(teDATASTORAGE stype)
{
    switch(stype)
    {
        case STORAGE_MAC:
            printf("can't erase MAC in f/w\r\n");
            break;
        
        case STORAGE_CONFIG:
            erase_flash_sector(FLASH_DEV_INFO_ADDR);
            break;
        
        case STORAGE_APPBOOT:
            break;

        case STORAGE_APPBANK:
            break;
            
        case STORAGE_BINBANK:
            break;

        case STORAGE_ROOTCA:
            erase_flash_sector(FLASH_ROOTCA_ADDR);
            break;
        case STORAGE_CLICA:
            erase_flash_sector(FLASH_CLICA_ADDR);
        default:
            break;
    }

    if (stype == STORAGE_APPBANK){
        PRT_INFO(" > STORAGE:ERASE_START APP Bank\r\n");
        erase_flash_bank(0);
    }
    else if (stype == STORAGE_BINBANK)
    {
        erase_flash_bank(1);
        PRT_INFO(" > STORAGE:ERASE_START BIN Bank\r\n");
    }
    PRT_INFO(" > STORAGE:ERASE_END\r\n");

#ifdef __USE_WATCHDOG__
    watchdog_update();
#endif
}
