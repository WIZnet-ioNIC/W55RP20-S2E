
# W55RP20-S2E README

> The W55RP20 is a System-in-Package (SiP) developed by WIZnet, integrating Raspberry Pi's RP2040 microcontroller, WIZnet's W5500 Ethernet controller, and 2MB of Flash memory into a single chip. 
 This repository contains firmware that implements Serial to Ethernet using the W55RP20.

These sections will provide how to configure development environment to develop and modify W55RP20-S2E.

- [W55RP20-S2E README](#w55rp20-s2e-readme)
  - [Development environment configuration](#development-environment-configuration)
    - [Required Build Environment](#required-build-environment)
- [Hardware requirements](#hardware-requirements)
- [W232N](#w232n)



<a name="development_environment_configuration"></a>
## Development environment configuration

To develop and modify W55RP20-S2E, the development environment must be configured so that RP2040 can be used.

### Required Build Environment

We recommend the following versions for successful build and development.
Click each name to open the official download page:

- [**pico-sdk**](https://github.com/raspberrypi/pico-sdk/releases/tag/2.2.0): `2.2.0`
- [**ARM GCC Toolchain**](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads): `14.2.Rel1`
- [**CMake**](https://cmake.org/download/): `3.13` or later
- [**Ninja**](https://github.com/ninja-build/ninja/releases): `1.12.x` (build system used by the tasks under `build/`)
- [**Python**](https://www.python.org/downloads/): `3.8` or later (required by `pico-sdk` and the scripts in [tools/](tools))
- [**Git**](https://git-scm.com/download/win): latest (needed to clone `pico-sdk` and submodules)
- [**picotool**](https://github.com/raspberrypi/picotool): `2.2.0` (used by the *Run Project* task to load firmware via USB)
- [**OpenOCD**](https://github.com/raspberrypi/openocd): `0.12.0+dev` (used by the *Flash* / *Rescue Reset* tasks via CMSIS-DAP)
- [**SRecord**](https://srecord.sourceforge.net/download.html): provides `srec_cat` used by [tools/merge_hex.py](tools/merge_hex.py)
- [**Artistic Style (astyle)**](https://astyle.sourceforge.net/): used by [style/restyle.py](style/restyle.py)

> Using other versions of the toolchain may result in build errors.

W55RP20-S2E was developed by configuring the development environment for **Windows**, When configuring the development environment, refer to the '**9.2. Building on MS Windows**' section of '**Getting started with Raspberry Pi Pico**' document below to configure the development environment.

- [**Getting started with Raspberry Pi Pico**][link-getting_started_with_raspberry_pi_pico]

If you want development environments other than the development environment for Windows, note that you can find other ways to configure development environment in **'Chapter 9. Building on other platforms'**  section of the document above.

### Additional Tools Required by the `tools/` Scripts

The Python helper scripts under [tools/](tools) drive the post-build steps
(generating `.bin` / `.hex` / `.uf2` artifacts and the embedded Web page header).
The table below lists what each script needs:

| Script | Purpose | External programs used |
| ------ | ------- | ---------------------- |
| [tools/merge_hex.py](tools/merge_hex.py) | Convert `Boot.elf` / `App_linker.elf` to HEX/BIN and merge them | `arm-none-eabi-objcopy`, `srec_cat` |
| [tools/hex_to_uf2_converter.py](tools/hex_to_uf2_converter.py) | Convert the merged HEX to BIN and then to UF2 | `arm-none-eabi-objcopy`, `python` + `tools/uf2conv.py` |
| [tools/uf2conv.py](tools/uf2conv.py) | UF2 packing / unpacking helper | Python 3 only (stdlib) |
| [tools/html_to_c_header.py](tools/html_to_c_header.py) | Embed `port/app/html_file/Web_page.html` into a C header | Python 3 only (stdlib) |
| [style/restyle.py](style/restyle.py) | Apply project coding style to C/C++ sources under `main/` and `port/` | `astyle` (Artistic Style) |

> `arm-none-eabi-objcopy` is shipped with the ARM GCC Toolchain listed above.
> `srec_cat` is provided by the **SRecord** package and must be installed
> separately on Windows.
> `astyle` must be installed and available on `PATH` for [style/restyle.py](style/restyle.py)
> to find it (the script searches for `astyle.exe` / `astyle`).

#### Install / Download CLI commands (Windows)

The easiest way to install the required programs on Windows is via
[winget](https://learn.microsoft.com/windows/package-manager/winget/) or
[Chocolatey](https://chocolatey.org/). Run the commands below from an
elevated PowerShell prompt.

```powershell
# Python 3 (>= 3.8)
winget install --id Python.Python.3.12 -e

# CMake
winget install --id Kitware.CMake -e

# Ninja build system
winget install --id Ninja-build.Ninja -e

# Git (needed to clone pico-sdk and submodules)
winget install --id Git.Git -e

# ARM GCC Toolchain 14.2.Rel1
winget install --id Arm.GnuArmEmbeddedToolchain -e
# Alternative: download the installer manually from
#   https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads

# SRecord (provides srec_cat used by tools/merge_hex.py)
choco install srecord -y
# Alternative: download the Windows binary from
#   https://srecord.sourceforge.net/download.html
# and add the extracted folder to your PATH.

# Artistic Style (provides astyle used by style/restyle.py)
choco install astyle -y
# Alternative: download the Windows binary from
#   https://astyle.sourceforge.net/
# and add the extracted folder to your PATH.
```

The Raspberry Pi Pico specific tools (pico-sdk, picotool, OpenOCD) are most
easily installed through the **Raspberry Pi Pico** VS Code extension, which
downloads matching versions into `%USERPROFILE%\.pico-sdk\` (the tasks in this
workspace already point at that location):

```powershell
# Install the VS Code extension (one-time)
code --install-extension raspberry-pi.raspberry-pi-pico
```

If you prefer to install them manually:

```powershell
# pico-sdk 2.2.0
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
# Then set PICO_SDK_PATH to the cloned folder

# picotool 2.2.0
git clone -b 2.2.0 https://github.com/raspberrypi/picotool.git
# Build instructions: https://github.com/raspberrypi/picotool#building

# OpenOCD for Raspberry Pi (with RP2040/RP2350 support)
git clone https://github.com/raspberrypi/openocd.git
# Build instructions: https://github.com/raspberrypi/openocd#readme
```

After installation, verify that every CLI is on your `PATH`:

```powershell
python --version
cmake --version
ninja --version
arm-none-eabi-gcc --version
arm-none-eabi-objcopy --version
srec_cat --version
astyle --version
picotool version
openocd --version
```



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
