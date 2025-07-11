/**
  ******************************************************************************
  * @file    RP2040 Serial to Ethernet Project - WIZ5XXSR-RP Boot
  * @author  Mason Lee, PaaS Team
  * @version v1.0.0
  * @date    April-2022
  * @brief   Main program body
  ******************************************************************************
  * @attention
  * @par Revision history
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, WIZnet SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2022 WIZnet Co., Ltd.</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/

#include "tusb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "WIZnet_board.h"
#include "port_common.h"
#include "dhcp.h"
#include "dhcp_cb.h"
#include "segcp.h"
#include "ConfigData.h"
#include "timerHandler.h"
#include "uartHandler.h"
#include "deviceHandler.h"
#include "ConfigData.h"
#include "flashHandler.h"
#include "gpioHandler.h"
#include "storageHandler.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "seg.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void RP2040_Init(void);
static void RP2040_W5X00_Init(void);
static void set_W5X00_NetTimeout(void);
static uint8_t boot_mode_pin_get(void);

/* Private variables ---------------------------------------------------------*/
extern uint8_t flag_process_dhcp_success;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();

    RP2040_Init();
    stdio_init_all();
    //RP2040_W5X00_Init();
    PRT_INFO("Hello Bootloader\r\n");
    //while(1);
    RP2040_Board_Init();
    
    load_boot_DevConfig_from_storage();
    
    //while(!(stdio_usb_connected()));

    if ((!boot_mode_pin_get() || ((device_bank_check(0) < 0) && (device_bank_check(1) < 0))))
    {
        PRT_INFO("Run Bootloader Config\r\n");
        set_device_status_all(ST_BOOT);
        RP2040_W5X00_Init();
        set_W5X00_NetTimeout();
        Timer_Configuration();
        init_connection_status_io();
        sleep_ms(100);

        while (check_phylink_status() == PHY_LINK_OFF)
          sleep_ms(100);
        display_Dev_Info_header();
        display_Dev_Info_main();
        
        Net_Conf();
        if(dev_config->network_option.dhcp_use)
        {
            if(process_dhcp() == DHCP_IP_LEASED) // DHCP success
                flag_process_dhcp_success = ON;
            else // DHCP failed
                Net_Conf(); // Set default static IP settings
        }

        display_Net_Info();
        display_Dev_Info_dhcp();
        
#ifdef __USE_WATCHDOG__
        watchdog_enable(DEVICE_WDT_TIMEOUT, 0);
#endif
        while (1)
        {
            if (flag_check_phylink)
            {
                flag_check_phylink = 0;
                if (check_phylink_status() == PHY_LINK_OFF)
                    device_reboot();
            }
            do_segcp();
            if(flag_process_dhcp_success == ON) DHCP_run(); // DHCP client handler for IP renewal
#ifdef __USE_WATCHDOG__
            watchdog_update();
#endif
        }   
        return 0;
    }

    if (dev_config->firmware_update.fwup_copy_flag == 1)
    {
        if (device_bank_check(1) < 0)
        {
          printf("device_bank_check failed\r\n");
          device_raw_reboot();
        }
        else
        {
          erase_storage(STORAGE_APPBANK);
          if (device_bank_copy() < 0 )
          {
            printf("device_bank_copy failed\r\n");
            device_raw_reboot();
          }
        }
        dev_config->firmware_update.fwup_copy_flag = 0;
        write_storage(STORAGE_CONFIG, 0, dev_config, sizeof(DevConfig));
    }   

    printf("jump addr = 0x%08X\r\n", FLASH_START_ADDR_BANK0);
    sleep_ms(100);

    disable_interrupts();
    reset_peripherals();
    jump_to_app(FLASH_START_ADDR_BANK0);

    while(1)
    {

    }
    return 0;
}



/*****************************************************************************
 * Private functions
 ****************************************************************************/
static void RP2040_Init(void)
{
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
    //SystemCoreClockUpdate();
    flash_critical_section_init();
    sleep_ms(10);
}

static void RP2040_W5X00_Init(void)
{
    wizchip_spi_initialize((PLL_SYS_KHZ * 1000 / 4)); //33.25Mhz
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();
}

static void set_W5X00_NetTimeout(void)
{
    DevConfig *dev_config = get_DevConfig_pointer();
    wiz_NetTimeout net_timeout;
    
    net_timeout.retry_cnt = dev_config->network_option.tcp_rcr_val;
    net_timeout.time_100us = 2000;
    wizchip_settimeout(&net_timeout);
    
    wizchip_gettimeout(&net_timeout); // TCP timeout settings
    PRT_INFO(" - Network Timeout Settings - RCR: %d, RTR: %d\r\n", net_timeout.retry_cnt, net_timeout.time_100us);
}

static uint8_t boot_mode_pin_get(void)
{
  GPIO_Configuration(BOOT_MODE_PIN, IO_INPUT, IO_PULLUP);
  sleep_ms(10);
  return GPIO_Input_Read(BOOT_MODE_PIN);
}


