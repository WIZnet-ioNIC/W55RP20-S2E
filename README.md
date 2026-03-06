
# W55RP20-S2E README

> The W55RP20 is a System-in-Package (SiP) developed by WIZnet, integrating Raspberry Pi's RP2040 microcontroller, WIZnet's W5500 Ethernet controller, and 2MB of Flash memory into a single chip. 
 This repository contains firmware that implements Serial to Ethernet using the W55RP20.

These sections will provide how to configure development environment to develop and modify W55RP20-S2E.

- [W55RP20-S2E README](#w55rp20-s2e-readme)
  - [Development environment configuration](#development-environment-configuration)
    - [Required Build Environment](#required-build-environment)
    - [Applying Patches](#applying-patches)
  - [Build](#build)
  - [Build Options (ProjectOptions.cmake)](#build-options-projectoptionscmake)
    - [Board Selection](#board-selection)
    - [Operation Mode](#operation-mode)
    - [Feature Flags](#feature-flags)
    - [Serial Protocol](#serial-protocol)
  - [Features & Architecture](#features--architecture)
- [Hardware requirements](#hardware-requirements)
- [W232N](#w232n)



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

<a name="applying_patches"></a>
### Applying Patches

Some features of the W55RP20-S2E firmware require patches to be applied to the **pico-sdk**.  
To apply the provided patch files, run the following commands from the repository root:

```bash
cd .\libraries\pico-sdk\
git apply ..\..\patches\001_pico-sdk_watchdog.patch
```

<a name="build"></a>

## Build

From the project root, run the following commands to configure and build:

```bash
# 1. Configure (reads ProjectOptions.cmake automatically)
cmake -S . -B build

# 2. Build
cmake --build build
```

> All build options are managed in `ProjectOptions.cmake`. You do **not** need to pass `-D` flags on the command line — simply edit the file and re-run cmake.

The build produces the following output files under `build/`:

| File | Description |
|------|-------------|
| `main/Boot/Boot.uf2` | Bootloader image |
| `main/App/App_linker.uf2` | Application image |
| `W55RP20_S2E_merged.uf2` | Merged UF2 (Boot + App) — flash this to the device |

<a name="build-options-projectoptionscmake"></a>

## Build Options (ProjectOptions.cmake)

All user-configurable options are in [`ProjectOptions.cmake`](ProjectOptions.cmake).
Edit the file, then re-run cmake — changes are applied automatically without a clean build.

### Board Selection

Uncomment the target board (only one at a time):

```cmake
if(NOT DEFINED BOARD_NAME)
    #set(BOARD_NAME WIZ5XXSR_RP)
    set(BOARD_NAME W55RP20_S2E)    # <-- active
    #set(BOARD_NAME W232N)
    #set(BOARD_NAME IP20)
    #set(BOARD_NAME PLATYPUS_S2E)
endif()
```

| Value | Target hardware |
|-------|----------------|
| `W55RP20_S2E` | W55RP20-EVB-PICO (default) |
| `W232N` | WIZnet W232N industrial module |
| `IP20` | IP20 module |
| `WIZ5XXSR_RP` | WIZ5xxSR-RP series |
| `PLATYPUS_S2E` | Platypus S2E board |

### Operation Mode

Uncomment **one** line to select the SEG operation mode compiled into the firmware:

```cmake
# set(OPMODE "TCP_CLIENT_MODE"      CACHE STRING "SEG operation mode" FORCE)
set(OPMODE "TCP_SERVER_MODE"     CACHE STRING "SEG operation mode" FORCE)  # <-- active
# set(OPMODE "TCP_MIXED_MODE"      CACHE STRING "SEG operation mode" FORCE)
# set(OPMODE "UDP_MODE"            CACHE STRING "SEG operation mode" FORCE)
# set(OPMODE "SSL_TCP_CLIENT_MODE" CACHE STRING "SEG operation mode" FORCE)
# set(OPMODE "MQTT_CLIENT_MODE"    CACHE STRING "SEG operation mode" FORCE)
# set(OPMODE "MQTTS_CLIENT_MODE"   CACHE STRING "SEG operation mode" FORCE)
```

| Value | Description |
|-------|-------------|
| `TCP_CLIENT_MODE` | Device connects to a remote TCP server |
| `TCP_SERVER_MODE` | Device listens and accepts incoming TCP connections |
| `TCP_MIXED_MODE` | Switches between client and server roles |
| `UDP_MODE` | UDP communication |
| `SSL_TCP_CLIENT_MODE` | TCP client with TLS/SSL |
| `MQTT_CLIENT_MODE` | MQTT over TCP |
| `MQTTS_CLIENT_MODE` | MQTT over TLS |

### Feature Flags

Enable or disable optional subsystems:

```cmake
set(ENABLE_SEGCP ON  CACHE BOOL "Enable config tool tasks (UDP/TCP/Serial)" FORCE)
# set(ENABLE_SEGCP OFF CACHE BOOL "Enable config tool tasks (UDP/TCP/Serial)" FORCE)
```

| Flag | Description |
|------|-------------|
| `ENABLE_SEGCP` | Enables the WIZnet Config Tool interface (UDP/TCP/Serial AT commands). When `OFF`, the device runs in data-forwarding mode only with no remote configuration support. |

### Serial Protocol

Uncomment **one** line to select the serial framing protocol:

```cmake
set(SERIAL_MODE "SEG_SERIAL_PROTOCOL_NONE" CACHE STRING "..." FORCE)  # Raw (default)
# set(SERIAL_MODE "SEG_SERIAL_MODBUS_RTU"   CACHE STRING "..." FORCE)
# set(SERIAL_MODE "SEG_SERIAL_MODBUS_ASCII" CACHE STRING "..." FORCE)
```

| Value | Description |
|-------|-------------|
| `SEG_SERIAL_PROTOCOL_NONE` | Raw serial passthrough (default) |
| `SEG_SERIAL_MODBUS_RTU` | Modbus RTU framing |
| `SEG_SERIAL_MODBUS_ASCII` | Modbus ASCII framing |

<a name="features--architecture"></a>

## Features & Architecture

### Dual-bank Boot/App Structure

The firmware is split into two independently linked images:

| Image | Role |
|-------|------|
| **Boot** | Handles firmware update (OTA via network), factory reset, and SEGCP in bootloader mode |
| **App** | Main S2E application — runs after Boot hands off control |

Both images are merged into a single `_merged.uf2` file for flashing.

### FreeRTOS Task Layout (App)

| Task | Role |
|------|------|
| `net_status_task` | Monitors PHY link, runs DHCP/DNS, signals other tasks when network is ready |
| `seg_task` | Main S2E loop — drives the TCP/UDP/MQTT state machine |
| `seg_u2e_task` | Forwards data from UART ring buffer to Ethernet socket |
| `seg_recv_task` | Receives data from Ethernet socket and writes to UART |
| `seg_timer_task` | Handles inactivity, keepalive, and connection-password timers |
| `eth_interrupt_task` | Processes W5500 socket interrupt (receive event) |
| `segcp_udp/tcp/uart_task` | Config Tool protocol handlers *(only when `ENABLE_SEGCP=ON`)* |
| `http_webserver_task` | Embedded web UI for device configuration |
| `spi_data_transfer_task` | SPI slave interface for AT command input |

### TCP Keepalive & Timeout Notes

- **Keepalive** is enabled by default (`keepalive_en = ENABLE`).
  After connection, a keepalive probe is sent after `keepalive_wait_time` (default 7 s).
  If the peer does not respond within the W5500 retransmission window (`RCR × RTR` = 8 × 200 ms = 1.6 s), the socket is closed.
  → Disable keepalive via the Config Tool if the peer does not support it.

- **Inactivity timer**: set to `0` by default (disabled). When non-zero, the connection is closed after the specified number of seconds of no data.

- **W5500 TCP retransmission**: configured via `tcp_rcr_val` (default 8 retries, 200 ms interval = 1.6 s total timeout).

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
| Debug_UART_Tx           | O    | 65      | 0        | Output Debug Messages                                  |
| Debug_UART_Rx           | I    | 66      | 1        |                                                       |
| DATA_UART_TX_PIN        | O    | 9       | 4        | TX pin for Data UART transmission                      |
| DATA_UART_RX_PIN        | I    | 10      | 5        | RX pin for Data UART reception                         |
| DATA_UART_CTS_PIN       | I    | 11      | 6        | CTS pin for Data UART flow control                     |
| DATA_UART_RTS_PIN       | O    | 12      | 7        | RTS pin for Data UART flow control <br> When 485/422 selected by UART_IF_SEL pin, this pin act as 485/422 select pin. <br> NC : 485 <br> Low : 422                     |
| DATA_DTR_PIN            | O    | 14      | 8        | DTR pin for Data UART control                          |
| DATA_DSR_PIN            | I    | 15      | 9        | DSR pin for Data UART control                          |
| STATUS_PHYLINK_PIN      | O    | 16      | 10       | Output High when the PHY link is established           |
| STATUS_TCPCONNECT_PIN   | O    | 17      | 11       | Output High when TCP connection is active              |
| UART_IF_SEL_PIN         | I    | 18      | 12       | UART Interface select Input High : RS485/422, Low or Floating : RS232 |
| HW_TRIG_PIN             | I    | 20      | 14       | When this pin is Low during a device reset, it enters AT Command Mode |
| BOOT_MODE_PIN           | I    | 21      | 15       | When this pin is Low during a device reset, it enters Boot Mode        |
| FAC_RSTn_PIN            | I    | 40      | 18       | Holding Low for more than 5 seconds triggers a factory reset           |  

<BR>
<a name="W232N"></a>

# W232N  

The **W232N** is an industrial module from WIZnet that applies the W55RP20-S2E firmware. For more detailed information, please refer to the documentation for this product.

| Image                                                        | Name                                                      | Etc                                                          |
| ------------------------------------------------------------ | --------------------------------------------------------- | ------------------------------------------------------------ |
| <image src= "https://docs.wiznet.io/img/products/w232n/W232_Rail_mount.png" width="200px" height="150px"> | [**W232N**](https://docs.wiznet.io/Product/S2E-Module/Industrial/W232N-datasheet-kr)           | [W232N Document](https://docs.wiznet.io/Product/S2E-Module/Industrial/Config-tool-Guide-kr) |

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
