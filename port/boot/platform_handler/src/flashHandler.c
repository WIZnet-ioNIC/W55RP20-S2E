#include <string.h>
#include "port_common.h"
#include "common.h"
#include "flashHandler.h"
#include "deviceHandler.h"

#ifdef _FLASH_DEBUG_
	#include <stdio.h>
#endif

static uint8_t *flash_buf = NULL;
static critical_section_t g_flash_cri_sec;

static void flash_critical_section_lock(void)
{
    critical_section_enter_blocking(&g_flash_cri_sec);
}

static void flash_critical_section_unlock(void)
{
    critical_section_exit(&g_flash_cri_sec);
}

void flash_critical_section_init(void)
{
    critical_section_init(&g_flash_cri_sec);
}

void write_flash(uint32_t addr, uint8_t * data, uint32_t data_len)
{
    uint32_t i, access_len;

    flash_buf = malloc(FLASH_SECTOR_SIZE);
    if (data_len && ((data_len % FLASH_SECTOR_SIZE) == 0))
    {
        for (i=0; i<data_len; i+=FLASH_SECTOR_SIZE)
        {
            erase_flash_sector(addr + i);
            memcpy(flash_buf, data, FLASH_SECTOR_SIZE);
            flash_critical_section_lock();
            flash_range_program(addr + i, flash_buf, FLASH_SECTOR_SIZE);
            flash_critical_section_unlock();
        }
    }
    else
    {
        for (i=0; i<data_len; i+=FLASH_SECTOR_SIZE)
        {
            memset(flash_buf, 0xFF, FLASH_SECTOR_SIZE);
            read_flash(addr + i, flash_buf, FLASH_SECTOR_SIZE);
            erase_flash_sector(addr + i);
            access_len = MIN(data_len - i, FLASH_SECTOR_SIZE);
            memcpy(flash_buf, data + i, access_len);
            flash_critical_section_lock();
            flash_range_program(addr + i, flash_buf, FLASH_SECTOR_SIZE);
            flash_critical_section_unlock();
        }
    }
    free(flash_buf);
}

void read_flash(uint32_t addr, uint8_t *data, uint32_t data_len)
{
    addr += XIP_BASE;
    memcpy(data, addr, data_len);
}

void erase_flash_sector(uint32_t addr)
{
    flash_critical_section_lock();
    flash_range_erase(addr, FLASH_SECTOR_SIZE);
    flash_critical_section_unlock();
}

void erase_flash_bank(uint8_t bank_num)
{
    if (bank_num == 0)
    {
        flash_critical_section_lock();
        flash_range_erase(FLASH_START_ADDR_BANK0_OFFSET, FLASH_APP_BANK_SIZE);
        flash_critical_section_unlock();
    }
    else if (bank_num == 1)
    {
        flash_critical_section_lock();
        flash_range_erase(FLASH_START_ADDR_BANK1_OFFSET, FLASH_APP_BANK_SIZE);
        flash_critical_section_unlock();
    }
}
