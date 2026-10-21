
# W55RP20-S2E README

> The W55RP20 is a System-in-Package (SiP) developed by WIZnet, integrating Raspberry Pi's RP2040 microcontroller, WIZnet's W5500 Ethernet controller, and 2MB of Flash memory into a single chip. 
 This repository contains firmware that implements 4-port Serial to Ethernet using the W55RP20.
 DATA0 and DATA1 run on the RP2040 hardware UARTs, DATA2 and DATA3 on PIO UARTs, and each channel has its own data socket.

These sections will provide how to configure development environment to develop and modify W55RP20-S2E.

- [W55RP20-S2E README](#w55rp20-s2e-readme)
  - [Development environment configuration](#development-environment-configuration)
    - [Required Build Environment](#required-build-environment)
- [Hardware requirements](#hardware-requirements)



<a name="development_environment_configuration"></a>
## Development environment configuration

To develop and modify W55RP20-S2E, the development environment must be configured so that RP2040 can be used.

### Required Build Environment

We recommend the following versions for successful build and development:

- **pico-sdk**: `2.2.0`  
- **ARM GCC Toolchain**: `14.2.Rel1`

> Using other versions of the toolchain may result in build errors.  

W55RP20-S2E was developed by configuring the development environment for **Windows**, When configuring the development environment, refer to the '**9.2. Building on MS Windows**' section of '**Getting started with Raspberry Pi Pico**' document below to configure the development environment.

- [**Getting started with Raspberry Pi Pico**][link-getting_started_with_raspberry_pi_pico]

If you want development environments other than the development environment for Windows, note that you can find other ways to configure development environment in **'Chapter 9. Building on other platforms'**  section of the document above.



<a name="hardware_requirements"></a>

# Hardware requirements

| Image                                                        | Name                                                      | Etc                                                          |
| ------------------------------------------------------------ | --------------------------------------------------------- | ------------------------------------------------------------ |
| <image src= "https://docs.wiznet.io/assets/images/w55rp20-evb-pico-docs-8e041fe8924bed1c8d567c1c8b87628d.png" width="200px" height="150px"> | [**W55RP20-EVB-PICO**](https://docs.wiznet.io/Product/ioNIC/W55RP20/w55rp20-evb-pico)           | [W55RP20 Document](https://docs.wiznet.io/Product/ioNIC/W55RP20/documents_md) |

> ### Pin Diagram

The W55RP20 has internal connections between the RP2040 and W5500 via GPIO pins. The connection table is as follows:

| I/O  | Pin Name | Description                                    |
| :--- | -------- | ---------------------------------------------- |
| O    | GPIO20   | Connected to **CSn** on W5500                  |
| O    | GPIO21   | Connected to **SCLK** on W5500                 |
| I    | GPIO22   | Connected to **MISO** on W5500                 |
| O    | GPIO23   | Connected to **MOSI** on W5500                 |
| I    | GPIO24   | Connected to **INTn** on W5500                 |
| O    | GPIO25   | Connected to **RSTn** on W5500                 |
<BR>

The function pins are as follows :
| Function               | Type | Pin Num | GPIO Num | Description                                           |
|------------------------|------|---------|----------|-------------------------------------------------------|
| DEBUG_UART_TX_PIN       | O    | 40      | 18       | TX pin for debug message output (PIO, 115200-8-N-1, TX only). Output is enabled by the debug setting (DG, factory default: enabled), which is read at boot |
| DATA0_UART_TX_PIN       | O    | 9       | 4        | TX pin for DATA0 UART transmission (HW UART1)          |
| DATA0_UART_RX_PIN       | I    | 10      | 5        | RX pin for DATA0 UART reception (HW UART1)             |
| DATA0_UART_CTS_PIN      | I    | 11      | 6        | CTS input for RTS/CTS flow control, or DSR input for DTR/DSR flow control, as set by FL (HW UART1) |
| DATA0_UART_RTS_PIN      | O    | 12      | 7        | RTS output for RTS/CTS flow control, or DTR output for DTR/DSR flow control, as set by FL. Drives the transceiver direction when RS-422/485 is selected by UI (HW UART1) |
| DATA1_UART_TX_PIN       | O    | 65      | 0        | TX pin for DATA1 UART transmission (HW UART0)          |
| DATA1_UART_RX_PIN       | I    | 66      | 1        | RX pin for DATA1 UART reception (HW UART0)             |
| DATA1_UART_CTS_PIN      | I    | 67      | 2        | CTS input for RTS/CTS flow control, or DSR input for DTR/DSR flow control, as set by EF (HW UART0) |
| DATA1_UART_RTS_PIN      | O    | 68      | 3        | RTS output for RTS/CTS flow control, or DTR output for DTR/DSR flow control, as set by EF. Drives the transceiver direction when RS-422/485 is selected by EI (HW UART0) |
| DATA2_UART_TX_PIN       | O    | 14      | 8        | TX pin for DATA2 UART transmission (PIO UART2)         |
| DATA2_UART_RX_PIN       | I    | 15      | 9        | RX pin for DATA2 UART reception (PIO UART2)            |
| DATA2_UART_CTS_PIN      | I    | 16      | 10       | CTS input for RTS/CTS flow control, or DSR input for DTR/DSR flow control, as set by WF (PIO UART2) |
| DATA2_UART_RTS_PIN      | O    | 17      | 11       | RTS output for RTS/CTS flow control, or DTR output for DTR/DSR flow control, as set by WF. Drives the transceiver direction when RS-422/485 is selected by WI (PIO UART2) |
| DATA3_UART_TX_PIN       | O    | 18      | 12       | TX pin for DATA3 UART transmission (PIO UART3)         |
| DATA3_UART_RX_PIN       | I    | 19      | 13       | RX pin for DATA3 UART reception (PIO UART3)            |
| DATA3_UART_CTS_PIN      | I    | 20      | 14       | CTS input for RTS/CTS flow control, or DSR input for DTR/DSR flow control, as set by YF (PIO UART3) |
| DATA3_UART_RTS_PIN      | O    | 21      | 15       | RTS output for RTS/CTS flow control, or DTR output for DTR/DSR flow control, as set by YF. Drives the transceiver direction when RS-422/485 is selected by YI (PIO UART3) |
| HW_TRIG_PIN             | I    | 30      | 16       | GPIO input (internal pull-up). If Low at boot (sampled 5 times, 5 ms apart), the device enters AT Command Mode on DATA0 |
| MCU_STATUS_PIN          | O    | 41      | 19       | GPIO output. Starts Low at boot, then toggles every second (1 s high, 1 s low) while the firmware is running. A pin that stops toggling means the MCU has stopped |
| DATA0_STATUS_TCPCONNECT_PIN | O | 46     | 26       | GPIO output. Low while the DATA0 TCP connection is established, High otherwise (including UDP mode) |
| DATA1_STATUS_TCPCONNECT_PIN | O | 47     | 27       | GPIO output. Low while the DATA1 TCP connection is established, High otherwise (including UDP mode) |
| DATA2_STATUS_TCPCONNECT_PIN | O | 48     | 28       | GPIO output. Low while the DATA2 TCP connection is established, High otherwise (including UDP mode) |
| DATA3_STATUS_TCPCONNECT_PIN | O | 49     | 29       | GPIO output. Low while the DATA3 TCP connection is established, High otherwise (including UDP mode) |
| UART_IF_SEL_PIN         | -    | -       | -        | Removed. The interface is chosen per channel by configuration (UI/EI/WI/YI: 0 RS-232/TTL, 1 RS-422, 2 RS-485, 3 RS-485 Reverse). In RS-422/485 mode the channel's RTS pin is the direction control: held High for RS-422, High while transmitting for RS-485, Low while transmitting for RS-485 Reverse |
| FAC_RSTn_PIN            | -    | -       | -        | Removed. Factory reset is issued with the FR command   |
| STATUS_PHYLINK_PIN      | -    | -       | -        | Removed. GPIO19 is used as MCU_STATUS_PIN. The PHY link is shown by the W5500 LINK LED output |

- Flow control values (FL/EF/WF/YF): 0 None, 1 XON/XOFF, 2 RTS/CTS, 3 RTS only, 4 Reverse RTS, 5 DTR/DSR.
- GPIO16, 18, 19 and 29 are not on the W55RP20-EVB-PICO header. On the EVB, GPIO19 is the onboard red LED.
- The BOOT_MODE pin of the 2-port firmware is not used.
- Factory default for every channel: RS-232/TTL, 115200-8-N-1, no flow control.

<BR>

<!--
Link
-->

[link-getting_started_with_raspberry_pi_pico]: https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
[link-rp2040]: https://www.raspberrypi.org/products/rp2040/
[link-w5100s]: https://docs.wiznet.io/Product/iEthernet/W5100S/overview
[link-wiz500sr-rp]: https://docs.wiznet.io/Product/S2E-Module/WIZ5xxSR-RP-Series/WIZ500SR-RP/overview
[link-wiz505sr-rp]: https://docs.wiznet.io/Product/S2E-Module/WIZ5xxSR-RP-Series/WIZ505SR-RP/overview
[link-wiz510sr-rp]: https://docs.wiznet.io/Product/S2E-Module/WIZ5xxSR-RP-Series/WIZ510SR-RP/overview
[link-wiz500sr-rp_main]: https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz500sr-rp_main.png
[link-wiz505sr-rp_main]: https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz505sr-rp_main.png
[link-wiz510sr-rp_main]: https://github.com/Wiznet/W5XXSR-RP-C/blob/main/static/images/getting_started/wiz510sr-rp_main.png
