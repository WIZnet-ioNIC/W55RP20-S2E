# CMake minimum required version
cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
endif()
include(pico_sdk_import.cmake)

# Pull in FreeRTOS (must be before project)
if(NOT DEFINED FREERTOS_KERNEL_PATH)
set(FREERTOS_KERNEL_PATH ${CMAKE_SOURCE_DIR}/libraries/FreeRTOS-Kernel)
    message(STATUS "FREERTOS_KERNEL_PATH = ${FREERTOS_KERNEL_PATH}")
endif()
include(FreeRTOS_Kernel_import.cmake)

# Set project name
set(PROJECT_NAME RP2040-HAT-FREERTOS-C)

# Set project informations
project(${PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

# All user-configurable options (board, opmode, features)
include(ProjectOptions.cmake)

message(STATUS "========== ProjectOptions summary ==========")
message(STATUS "  BOARD_NAME   = ${BOARD_NAME}")
message(STATUS "  WIZNET_CHIP  = ${WIZNET_CHIP}")
message(STATUS "  OPMODE       = ${OPMODE}")
message(STATUS "  ENABLE_SEGCP = ${ENABLE_SEGCP}")
message(STATUS "  SERIAL_MODE  = ${SERIAL_MODE}")
message(STATUS "============================================")

if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED AWS_SDK_DIR)
    set(AWS_SDK_DIR ${CMAKE_SOURCE_DIR}/libraries/aws-iot-device-sdk-embedded-C)
    message(STATUS "AWS_SDK_DIR = ${AWS_SDK_DIR}")
endif()

if(NOT DEFINED MBEDTLS_DIR)
    set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/libraries/mbedtls)
    message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
endif()

if(NOT DEFINED BOOT_PORT_DIR)
    set(BOOT_PORT_DIR ${CMAKE_SOURCE_DIR}/port/boot)
    message(STATUS "BOOT_PORT_DIR = ${BOOT_PORT_DIR}")
endif()

if(NOT DEFINED APP_PORT_DIR)
    set(APP_PORT_DIR ${CMAKE_SOURCE_DIR}/port/app)
    message(STATUS "APP_PORT_DIR = ${APP_PORT_DIR}")
endif()

# Turn off mbedtls test mode
set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
add_definitions(-DMBEDTLS_CONFIG_FILE="${APP_PORT_DIR}/mbedtls/inc/SSLConfig.h")
add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)
add_definitions(-DPICO_USE_FASTEST_SUPPORTED_CLOCK=1)
add_definitions(-DPICO_FLASH_SPI_CLKDIV=4)

# UART PIO Debug option - uncomment to enable
#set(UART_PIO_DEBUG ON)
if(UART_PIO_DEBUG)
    add_definitions(-DUART_PIO_DEBUG)
endif()

if(${BOARD_NAME} STREQUAL PLATYPUS_S2E)
    add_definitions(-DPICO_DEFAULT_UART_BAUD_RATE=115200)
else() 
    add_definitions(-DPICO_DEFAULT_UART_BAUD_RATE=921600)
endif()

# Set compile options
add_compile_options(-ffunction-sections -fdata-sections)
add_link_options(-Wl,--gc-sections)
#add_compile_options(
#        -Wall
#        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
#        -Wno-unused-function # we have some for the docs that aren't called
#        -Wno-maybe-uninitialized
#        -O3
#        )

add_subdirectory(main)
# Add libraries in subdirectories
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
add_subdirectory(${MBEDTLS_DIR})
add_subdirectory(${BOOT_PORT_DIR})
add_subdirectory(${APP_PORT_DIR})

## Automatically run html_to_c_header.py before build
add_custom_target(
    html_to_c_header
    COMMAND python ${CMAKE_SOURCE_DIR}/tools/html_to_c_header.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Run html_to_c_header.py before build"
)

# Ensure Boot and App_linker depend on html_to_c_header
add_dependencies(Boot html_to_c_header)
add_dependencies(App_linker html_to_c_header)

## Automatically run merge_hex.py after build
add_custom_target(
    merge_hex ALL
    COMMAND python ${CMAKE_SOURCE_DIR}/tools/merge_hex.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Run merge_hex.py after build"
)

# Ensure merge_hex runs after Boot and App_linker are built
add_dependencies(merge_hex Boot App_linker)

## Automatically run style/restyle.py after build
add_custom_target(
    restyle ALL
    COMMAND python ${CMAKE_SOURCE_DIR}/style/restyle.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Run style/restyle.py after build"
)

# Ensure restyle runs after Boot and App_linker are built
add_dependencies(restyle Boot App_linker)

## Automatically run hex_to_uf2_converter.py after merge_hex
add_custom_target(
    hex_to_uf2_converter ALL
    COMMAND python ${CMAKE_SOURCE_DIR}/tools/hex_to_uf2_converter.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Run hex_to_uf2_converter.py to create merged UF2 file"
)

# Ensure hex_to_uf2_converter runs after merge_hex is completed
add_dependencies(hex_to_uf2_converter merge_hex)
