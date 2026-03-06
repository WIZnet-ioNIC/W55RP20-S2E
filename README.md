
# W55RP20-S2E README

> The W55RP20 is a System-in-Package (SiP) developed by WIZnet, integrating Raspberry Pi's RP2040 microcontroller, WIZnet's W5500 Ethernet controller, and 2MB of Flash memory into a single chip.
 This repository contains firmware that implements Serial to Ethernet using the W55RP20.

> [!WARNING]
> **This branch is currently under active development and debugging.**
> Features and build options are subject to change. Use with caution in production environments.

## Overview of Changes from the Original Firmware

The original firmware was built as a single monolithic image.
All operational settings (network mode, serial protocol, etc.) were configured at **runtime** through the WIZnet Config Tool, and the firmware loaded and applied those settings on boot.

This repository introduces a **build-time feature selection system** as an alternative approach.
Rather than relying solely on runtime configuration, developers can now select only the features and operation modes they need at compile time via [`ProjectOptions.cmake`](ProjectOptions.cmake).

**Motivation:**

- Reduce firmware footprint by excluding unused subsystems
- Simplify deployment for fixed-function devices that do not require the full Config Tool interface
- Provide a clear and maintainable customization point for future feature additions
- Enable response to specific or specialized customer requirements without modifying the core codebase

> The runtime Config Tool interface (`ENABLE_SEGCP`) remains available and can be re-enabled at any time.
> Both approaches — runtime configuration and build-time selection — can coexist.

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
- [Hardware & Product Information](#hardware--product-information)



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

<a name="hardware--product-information"></a>

# Hardware & Product Information

For detailed hardware specifications, pin diagrams, product datasheets, and Config Tool usage guides, please refer to the **`main` branch**:

> [!NOTE]
> **[→ See the `main` branch README for full hardware and product documentation](https://github.com/Wiznet/W55RP20-S2E/blob/main/README.md)**

This includes:
- Hardware requirements and supported board list
- W55RP20 internal GPIO connections (RP2040 ↔ W5500)
- Function pin diagram
- W232N industrial module documentation
- WIZnet Config Tool usage guide

<!--
Link
-->

[link-getting_started_with_raspberry_pi_pico]: https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
