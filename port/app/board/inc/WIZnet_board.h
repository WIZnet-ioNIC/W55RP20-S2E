/*
*
@file   wiznet_board.h
@brief
*/

#ifndef __WIZNET_BOARD_H__ 
#define __WIZNET_BOARD_H__

#include <stdint.h>
#include "common.h"

////////////////////////////////
// Product Configurations     //
////////////////////////////////

#define WIZ5XXSR_RP 0
#define W55RP20_S2E 1
#define W232N       2

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#if ((DEVICE_BOARD_NAME == WIZ5XXSR_RP) || DEVICE_BOARD_NAME == W55RP20_S2E || DEVICE_BOARD_NAME == W232N) // Chip product
    #define __USE_DHCP_INFINITE_LOOP__          // When this option is enabled, if DHCP IP allocation failed, process_dhcp() function will try to DHCP steps again.
    #define __USE_DNS_INFINITE_LOOP__           // When this option is enabled, if DNS query failed, process_dns() function will try to DNS steps again.
    #define __USE_HW_FACTORY_RESET__            // Use Factory reset pin
    #define __USE_SAFE_SAVE__                   // When this option is enabled, data verify is additionally performed in the flash save of config-data.
    #define __USE_WATCHDOG__                  // WDT timeout 30 Second
    #define __USE_S2E_OVER_TLS__                // Use S2E TCP client over SSL/TLS mode
    #define __USE_UART_485_422__
    //#define __USE_USERS_GPIO__
#if (DEVICE_BOARD_NAME == WIZ5XXSR_RP)
    #define DEVICE_ID_DEFAULT                   "WIZ5XXSR-RP"//"S2E_SSL-MB" // Device name
#elif (DEVICE_BOARD_NAME == W55RP20_S2E)
    #define __USE_UART_IF_SELECTOR__            // Use Serial interface port selector pin
    #define DEVICE_ID_DEFAULT                   "W55RP20-S2E"//"S2E_SSL-MB" // Device name
    #define __USE_UART_SPI_IF_SELECTOR__        // Use UART or SPI interface port selector pin
#elif (DEVICE_BOARD_NAME == W232N)
    #define DEVICE_ID_DEFAULT                   "W232N"//"S2E_SSL-MB" // Device name
#endif
    #define DEVICE_CLOCK_SELECT                 CLOCK_SOURCE_EXTERNAL // or CLOCK_SOURCE_INTERNAL
    #define DEVICE_UART_CNT                     (1)
    #define DEVICE_SETTING_PASSWORD_DEFAULT     "00000000"
    #define DEVICE_GROUP_DEFAULT                "WORKGROUP" // Device group
    #define DEVICE_TARGET_SYSTEM_CLOCK   PLL_SYS_KHZ
#endif

/* PHY Link check  */
#define PHYLINK_CHECK_CYCLE_MSEC  1000

/* Factory Reset period  */
#define FACTORY_RESET_TIME_MS   5000

////////////////////////////////
// Pin definitions        //
////////////////////////////////

#if (DEVICE_BOARD_NAME == WIZ5XXSR_RP)
  #define DTR_PIN                 8
  #define DSR_PIN                 9
    
  #define STATUS_PHYLINK_PIN      10
  #define STATUS_TCPCONNECT_PIN   11

  // UART1
  #define DATA0_UART_TX_PIN      4
  #define DATA0_UART_RX_PIN      5
  #define DATA0_UART_CTS_PIN     6
  #define DATA0_UART_RTS_PIN     7

  #define WIZCHIP_PIN_SCK 18
  #define WIZCHIP_PIN_MOSI 19
  #define WIZCHIP_PIN_MISO 16
  #define WIZCHIP_PIN_CS 17
  #define WIZCHIP_PIN_RST 20
  #define WIZCHIP_PIN_IRQ 21

  #define BOOT_MODE_PIN          13
  #define FAC_RSTn_PIN           28
  #define HW_TRIG_PIN            29
  #define DATA0_UART_PORTNUM          (1)

  #define LED1_PIN      STATUS_PHYLINK_PIN        //STATUS_PHYLINK
  #define LED2_PIN      STATUS_TCPCONNECT_PIN    //STATUS_TCP_PIN
  #define LED3_PIN      12    //Blink
  #define LEDn    3

#elif ((DEVICE_BOARD_NAME == W55RP20_S2E) || (DEVICE_BOARD_NAME == W232N))
  #define UART_IF_SEL_PIN        12   //High : 485/422, Low or NC : TTL/232
  #define UART_SPI_IF_SEL_PIN    13   //High : SPI, Low or NC : UART

  #define DTR_PIN                 8
  #define DSR_PIN                 9
  
  #define STATUS_PHYLINK_PIN      10
  #define STATUS_TCPCONNECT_PIN   11
  
    // UART1
  #define DATA0_UART_TX_PIN      4
  #define DATA0_UART_RX_PIN      5
  #define DATA0_UART_CTS_PIN     6
  #define DATA0_UART_RTS_PIN     7

    // SPI0
  #define DATA0_SPI_SCK_PIN      2
  #define DATA0_SPI_TX_PIN       3
  #define DATA0_SPI_RX_PIN       4
  #define DATA0_SPI_CSn_PIN      5
  #define DATA0_SPI_INT_PIN      26
  
  #define WIZCHIP_PIN_SCK        21
  #define WIZCHIP_PIN_MOSI       23
  #define WIZCHIP_PIN_MISO       22
  #define WIZCHIP_PIN_CS         20
  #define WIZCHIP_PIN_RST        25
  #define WIZCHIP_PIN_IRQ        24
  
  #define BOOT_MODE_PIN          15    //When this pin is Low during a device reset, it enters AT Command Mode  
  #define FAC_RSTn_PIN           18    //Holding Low for more than 5 seconds triggers a factory reset
  #define HW_TRIG_PIN            14    //When this pin is Low during a device reset, it enters AT Command Mode
  #define DATA0_UART_PORTNUM          (1)
  
  #define LED1_PIN      STATUS_PHYLINK_PIN        //STATUS_PHYLINK
  #define LED2_PIN      STATUS_TCPCONNECT_PIN    //STATUS_TCP_PIN
  #define LED3_PIN      19    //Blink
  #define LEDn          3
#endif

#ifdef __USE_UART_SPI_IF_SELECTOR__
  typedef enum
  {
      UART_IF = 0,
      SPI_IF
  }if_TypeDef;
#endif


  typedef enum
  {
    LED1 = 0, // PHY link status
    LED2 = 1, // TCP connection status
    LED3 = 2  // blink
  } Led_TypeDef;

  extern volatile uint16_t phylink_check_time_msec;
  extern uint8_t flag_check_phylink;
   
  void RP2040_Board_Init(void);
  void init_hw_trig_pin(void);
  uint8_t get_hw_trig_pin(void);


  void init_uart_spi_if_sel_pin(void);
  uint8_t get_uart_spi_if(void);

  void init_uart_if_sel_pin(void);
  uint8_t get_uart_if_sel_pin(void);
  void init_factory_reset_pin(void);
  
#ifdef __USE_BOOT_ENTRY__
  void init_boot_entry_pin(void);
  uint8_t get_boot_entry_pin(void);
#endif
  
  void LED_Init(Led_TypeDef Led);
  void LED_On(Led_TypeDef Led);
  void LED_Off(Led_TypeDef Led);
  void LED_Toggle(Led_TypeDef Led);
  uint8_t get_LED_Status(Led_TypeDef Led);
  
#endif
