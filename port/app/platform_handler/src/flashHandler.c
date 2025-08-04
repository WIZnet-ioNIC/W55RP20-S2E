#include <string.h>
#include "port_common.h"
#include "common.h"
#include "flashHandler.h"
#include "deviceHandler.h"

#ifdef _FLASH_DEBUG_
#include <stdio.h>
#endif

typedef struct {
    bool op_is_erase;
    uint32_t addr;
    uint8_t *data;
    uint32_t data_len;
} mutation_operation_t;

static uint8_t *flash_buf = NULL;
static critical_section_t g_flash_cri_sec;
extern xSemaphoreHandle flash_critical_sem;

static void flash_critical_section_lock(void);
static void flash_critical_section_unlock(void);
void flash_critical_section_init(void);


static void flash_critical_section_lock(void) {
    xSemaphoreTake(flash_critical_sem, portMAX_DELAY);
    critical_section_enter_blocking(&g_flash_cri_sec);
}

static void flash_critical_section_unlock(void) {
    xSemaphoreGive(flash_critical_sem);
    critical_section_exit(&g_flash_cri_sec);
}

void flash_critical_section_init(void) {
    critical_section_init(&g_flash_cri_sec);
    flash_critical_sem = xSemaphoreCreateMutex();
}

static void flash_mudation_operation(void *param) {
    const mutation_operation_t *mop = (const mutation_operation_t *)param;
    uint32_t i, access_len;

    if (mop->op_is_erase == 0) {
        if (mop->data_len && ((mop->data_len % FLASH_SECTOR_SIZE) == 0)) {
            for (i = 0; i < mop->data_len; i += FLASH_SECTOR_SIZE) {
                flash_critical_section_lock();
                flash_range_erase(mop->addr + i, FLASH_SECTOR_SIZE);
                flash_critical_section_unlock();
                memcpy(flash_buf, mop->data, FLASH_SECTOR_SIZE);
                flash_critical_section_lock();
                flash_range_program(mop->addr + i, flash_buf, FLASH_SECTOR_SIZE);
                flash_critical_section_unlock();
            }
        } else {
            for (i = 0; i < mop->data_len; i += FLASH_SECTOR_SIZE) {
                memset(flash_buf, 0xFF, FLASH_SECTOR_SIZE);
                read_flash(mop->addr + i, flash_buf, FLASH_SECTOR_SIZE);
                flash_critical_section_lock();
                flash_range_erase(mop->addr + i, FLASH_SECTOR_SIZE);
                flash_critical_section_unlock();
                access_len = MIN(mop->data_len - i, FLASH_SECTOR_SIZE);
                memcpy(flash_buf, mop->data + i, access_len);
                flash_critical_section_lock();
                flash_range_program(mop->addr + i, flash_buf, FLASH_SECTOR_SIZE);
                flash_critical_section_unlock();
            }
        }

    } else {
        flash_critical_section_lock();
        flash_range_erase(mop->addr, FLASH_SECTOR_SIZE);
        flash_critical_section_unlock();
    }
}

void write_flash(uint32_t addr, uint8_t * data, uint32_t data_len) {
    mutation_operation_t mop;

    mop.op_is_erase = 0;
    mop.addr = addr;
    mop.data = data;
    mop.data_len = data_len;

    flash_buf = pvPortMalloc(FLASH_SECTOR_SIZE);
    memset(flash_buf, 0x00, FLASH_SECTOR_SIZE);
    flash_safe_execute(flash_mudation_operation, &mop, 0xFFFFFFFF);
    vPortFree(flash_buf);
}


void read_flash(uint32_t addr, uint8_t *data, uint32_t data_len) {
    addr += XIP_BASE;
    memcpy(data, (void *)(addr), data_len);
}

void erase_flash_sector(uint32_t addr) {
    mutation_operation_t mop;

    mop.op_is_erase = 1;
    mop.addr = addr;
    flash_safe_execute(flash_mudation_operation, &mop, 0xFFFFFFFF);
}