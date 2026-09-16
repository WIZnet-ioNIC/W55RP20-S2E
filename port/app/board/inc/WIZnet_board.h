/*

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
#define IP20        3
#define PLATYPUS_S2E 4

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#if ((DEVICE_BOARD_NAME == WIZ5XXSR_RP) || DEVICE_BOARD_NAME == W55RP20_S2E || DEVICE_BOARD_NAME == W232N || DEVICE_BOARD_NAME == IP20 || DEVICE_BOARD_NAME == PLATYPUS_S2E) // Chip product
//#define __USE_DHCP_INFINITE_LOOP__          // When this option is enabled, if DHCP IP allocation failed, process_dhcp() function will try to DHCP steps again.
#define __USE_DNS_INFINITE_LOOP__           // When this option is enabled, if DNS query failed, process_dns() function will try to DNS steps again.
// The WIZ145SR board has no factory reset pin; GP18 carries the debug UART.
#if (DEVICE_BOARD_NAME != W55RP20_S2E)
#define __USE_HW_FACTORY_RESET__            // Use Factory reset pin
#endif
#define __USE_SAFE_SAVE__                   // When this option is enabled, data verify is additionally performed in the flash save of config-data.
#define __USE_WATCHDOG__                  // WDT timeout 30 Second
// WIZ145SR offers neither an SSL/TLS nor an MQTT mode, so both are left out here.
// Their settings are what pushed the configuration structure past the single flash
// sector it is stored in, and the MQTT receive buffers cost 8 kB of RAM besides.
#if (DEVICE_BOARD_NAME != W55RP20_S2E)
#define __USE_S2E_OVER_TLS__                // Use S2E TCP client over SSL/TLS mode
#define __USE_MQTT__                        // Use S2E MQTT / MQTTS client mode
#endif
#define __USE_UART_485_422__
// WIZ145SR carries a TTL-only serial port with None / XON-XOFF / RTS-CTS flow
// control. The RTS-only modes and DTR/DSR drive an RS-422/485 transceiver this
// board does not have, so they stay out of the selectable range.
#if (DEVICE_BOARD_NAME == W55RP20_S2E)
#define SERIAL_FLOW_CONTROL_MAX             flow_rts_cts
#define SERIAL_UART_INTERFACE_MAX           UART_IF_RS232_TTL
#else
#define SERIAL_FLOW_CONTROL_MAX             flow_dtr_dsr
#define SERIAL_UART_INTERFACE_MAX           UART_IF_RS485_REVERSE
#endif
//#define __USE_USERS_GPIO__
#if (DEVICE_BOARD_NAME == WIZ5XXSR_RP)
#define DEVICE_ID_DEFAULT                   "WIZ5XXSR-RP"//"S2E_SSL-MB" // Device name
#define __USE_HW_TRIG_MODE_SWITCH__         // HW pin AT-mode entry
#elif (DEVICE_BOARD_NAME == W55RP20_S2E)
// WIZ145SR: interface selected by configuration (no IF_SEL pin); serial command mode is
// entered with the HW trigger pin or the '+++' trigger.
#define DEVICE_ID_DEFAULT                   "WIZ145SR" // Device name
#define __USE_HW_TRIG_MODE_SWITCH__         // HW pin serial command mode entry
#define __STATUS_IO_ACTIVE_LOW__            // Status pins read Low when connected / link up
#elif (DEVICE_BOARD_NAME == PLATYPUS_S2E)
#define __USE_UART_IF_SELECTOR__            // Use Serial interface port selector pin
#define __USE_HW_TRIG_MODE_SWITCH__
#define DEVICE_ID_DEFAULT                   "W55RP20-S2E-2CH"//"S2E_SSL-MB" // Device name
#elif (DEVICE_BOARD_NAME == W232N)
#define DEVICE_ID_DEFAULT                   "W232N"//"S2E_SSL-MB" // Device name
#define __USE_HW_TRIG_MODE_SWITCH__
#elif (DEVICE_BOARD_NAME == IP20)
#define DEVICE_ID_DEFAULT                   "IP20"//"S2E_SSL-MB" // Device name
#define __USE_HW_TRIG_MODE_SWITCH__
#endif
#define DEVICE_CLOCK_SELECT                 CLOCK_SOURCE_EXTERNAL // or CLOCK_SOURCE_INTERNAL
#if (DEVICE_BOARD_NAME == W55RP20_S2E)
#define DEVICE_UART_CNT                     (4)   // 2 HW UART (DATA0/1) + 2 PIO UART (DATA2/3)
#else
#define DEVICE_UART_CNT                     (2)
#endif
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

#elif ((DEVICE_BOARD_NAME == W55RP20_S2E) || (DEVICE_BOARD_NAME == W232N) || (DEVICE_BOARD_NAME == IP20) || (DEVICE_BOARD_NAME == PLATYPUS_S2E))

#if (DEVICE_BOARD_NAME == W55RP20_S2E)
// WIZ145SR pin map; the four channels run in GPIO order.
//
// Original (4-port): DATA2 TX/RX/CTS/RTS on GP13/14/8/15, DATA3 on GP12/27/9/28,
//     statuses on GP11/26/10/19, PHY link on GP29, no HW trigger or debug pin.
// Per channel: TX(out), RX(in), CTS(in), RTS(out), STATUS(out).
// CTS and DSR share the one input pin; RTS and DTR share the one output pin (function by config).
#define STATUS_PHYLINK_PIN           19   // PHY link status (not on EVB header; onboard LD2 red LED net)
#define HW_TRIG_PIN                  16   // Serial command mode entry, active low, read once at boot
#define DEBUG_UART_TX_PIN            18   // Debug message output over PIO (not on EVB header)

// DATA0 (HW uart1)
#define DATA0_UART_TX_PIN            4
#define DATA0_UART_RX_PIN            5
#define DATA0_UART_CTS_PIN           6
#define DATA0_UART_RTS_PIN           7
#define DATA0_STATUS_TCPCONNECT_PIN  26

// DATA1 (HW uart0)
#define DATA1_UART_TX_PIN            0
#define DATA1_UART_RX_PIN            1
#define DATA1_UART_CTS_PIN           2
#define DATA1_UART_RTS_PIN           3
#define DATA1_STATUS_TCPCONNECT_PIN  27

// DATA2 (PIO)
#define DATA2_UART_TX_PIN            8
#define DATA2_UART_RX_PIN            9
#define DATA2_UART_CTS_PIN           10
#define DATA2_UART_RTS_PIN           11
#define DATA2_STATUS_TCPCONNECT_PIN  28

// DATA3 (PIO)
#define DATA3_UART_TX_PIN            12
#define DATA3_UART_RX_PIN            13
#define DATA3_UART_CTS_PIN           14
#define DATA3_UART_RTS_PIN           15
#define DATA3_STATUS_TCPCONNECT_PIN  29   // not on EVB header (VSYS-sense net)

// DTR/DSR share the RTS/CTS pins (config selects RTS-CTS vs DTR-DSR); aliases keep the
// existing DTR/DSR GPIO helpers pointing at the correct shared pins.
#define DATA0_UART_DTR_PIN           DATA0_UART_RTS_PIN
#define DATA0_UART_DSR_PIN           DATA0_UART_CTS_PIN
#define DATA1_UART_DTR_PIN           DATA1_UART_RTS_PIN
#define DATA1_UART_DSR_PIN           DATA1_UART_CTS_PIN
#define DATA2_UART_DTR_PIN           DATA2_UART_RTS_PIN
#define DATA2_UART_DSR_PIN           DATA2_UART_CTS_PIN
#define DATA3_UART_DTR_PIN           DATA3_UART_RTS_PIN
#define DATA3_UART_DSR_PIN           DATA3_UART_CTS_PIN

// Removed: BOOT_MODE and UART_IF_SEL (chosen by configuration), and the factory reset pin.
// HW_TRIG now sits on GP16 and the debug UART on GP18; GP29 carries the DATA3 status.

#define LED1_PIN      STATUS_PHYLINK_PIN             // PHY link
#define LED2_PIN      DATA0_STATUS_TCPCONNECT_PIN    // DATA0 TCP status
#define LED3_PIN      DATA3_STATUS_TCPCONNECT_PIN    // No spare pin; heartbeat blink removed
#define LEDn          3

#else   // ---- W232N / IP20 / PLATYPUS_S2E : original 2-port pin map ----
#if (DEVICE_BOARD_NAME == PLATYPUS_S2E)
#define STATUS_PHYLINK_PIN      11
#else
#define STATUS_PHYLINK_PIN      10
#endif

// DATA0 - UART1
#define DATA0_UART_TX_PIN            4
#define DATA0_UART_RX_PIN            5
#define DATA0_UART_CTS_PIN           6
#define DATA0_UART_RTS_PIN           7
#define DATA0_UART_DTR_PIN           8
#define DATA0_UART_DSR_PIN           9
#define DATA0_UART_IF_SEL_PIN        12   //High : 485/422, Low or NC : TTL/232

#if (DEVICE_BOARD_NAME == PLATYPUS_S2E)
#define DATA0_STATUS_TCPCONNECT_PIN  10
#else
#define DATA0_STATUS_TCPCONNECT_PIN  11
#endif

// DATA1 - UART0
#define DATA1_UART_TX_PIN            0
#define DATA1_UART_RX_PIN            1
#define DATA1_UART_CTS_PIN           2
#define DATA1_UART_RTS_PIN           3
#define DATA1_UART_DTR_PIN           27
#define DATA1_UART_DSR_PIN           28
#define DATA1_UART_IF_SEL_PIN        16   //High : 485/422, Low or NC : TTL/232
#define DATA1_STATUS_TCPCONNECT_PIN  26

#define BOOT_MODE_PIN          15    //When this pin is Low during a device reset, it enters AT Command Mode
#define HW_TRIG_PIN            14    //When this pin is Low during a device reset, it enters AT Command Mode

#ifdef UART_PIO_DEBUG
#define DEBUG_UART_TX_PIN      29
#endif
#define LED1_PIN      STATUS_PHYLINK_PIN        //STATUS_PHYLINK
#define LED2_PIN      DATA0_STATUS_TCPCONNECT_PIN    //STATUS_TCP_PIN
#define LED3_PIN      19    //Blink
#define LEDn          3
#endif  // board-specific pin map

// ---- Common to this board group ----
#define WIZCHIP_PIN_SCK        21
#define WIZCHIP_PIN_MOSI       23
#define WIZCHIP_PIN_MISO       22
#define WIZCHIP_PIN_CS         20
#define WIZCHIP_PIN_RST        25
#define WIZCHIP_PIN_IRQ        24

#if (DEVICE_BOARD_NAME != W55RP20_S2E)
#define FAC_RSTn_PIN           18    //Holding Low for more than 5 seconds triggers a factory reset
#endif
#define DATA0_UART_PORTNUM          (1)
#endif

#ifdef __USE_UART_SPI_IF_SELECTOR__
typedef enum {
    UART_IF = 0,
    SPI_IF
} if_TypeDef;
#endif


typedef enum {
    LED1 = 0, // PHY link status
    LED2 = 1, // TCP connection status
    LED3 = 2  // blink
} Led_TypeDef;

extern volatile uint16_t phylink_check_time_msec;
extern uint8_t flag_check_phylink;

void RP2040_Board_Init(void);
void init_hw_trig_pin(void);
uint8_t get_hw_trig_pin(void);

void init_uart_if_sel_pin(void);
uint8_t get_uart_if_sel_pin(int channel);
void init_factory_reset_pin(void);
uint8_t get_phylink(void);
uint8_t get_factory_reset_pin(void);

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
