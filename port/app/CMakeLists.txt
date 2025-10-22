# MCU dependency files
add_library(APP_MCU_FILES STATIC)

target_include_directories(APP_MCU_FILES PUBLIC
        ${APP_PORT_DIR}/FreeRTOS-Kernel/inc
        ${APP_PORT_DIR}
)

target_link_libraries(APP_MCU_FILES PUBLIC
        cmsis_core
        pico_flash
        pico_multicore
        pico_stdlib
        pico_stdlib_headers
        pico_rand
        hardware_clocks
        hardware_dma
        hardware_flash
        hardware_gpio
        hardware_irq
        hardware_pio
        hardware_resets
        hardware_spi
        hardware_structs
        hardware_uart
        hardware_watchdog
        FreeRTOS-Kernel-Heap4
        tinyusb_device_unmarked
	tinyusb_board
        )

target_include_directories(AWS_SDK_FILES PUBLIC
        ${APP_PORT_DIR}/aws-iot-device-sdk-embedded-C/inc
)

# ioLibrary_Driver
add_library(APP_IOLIBRARY_FILES STATIC)

target_sources(APP_IOLIBRARY_FILES PUBLIC
        ${APP_PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        ${APP_PORT_DIR}/ioLibrary_Driver/src/w5x00_gpio_irq.c
        )

if(${BOARD_NAME} STREQUAL W55RP20_S2E OR ${BOARD_NAME} STREQUAL W232N OR ${BOARD_NAME} STREQUAL IP20)
pico_generate_pio_header(APP_IOLIBRARY_FILES ${APP_PORT_DIR}/ioLibrary_Driver/src/wiznet_spi_pio.pio)

target_sources(APP_IOLIBRARY_FILES PUBLIC 
        ${APP_PORT_DIR}/ioLibrary_Driver/src/wiznet_spi_pio.c
        )
endif()

target_include_directories(APP_IOLIBRARY_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet
        ${APP_PORT_DIR}/ioLibrary_Driver/inc
        ${APP_PORT_DIR}
        )

target_link_libraries(APP_IOLIBRARY_FILES PUBLIC
        APP_MCU_FILES
        APP_BOARD_FILES
        DHCP_FILES
        DNS_FILES
        ETHERNET_FILES
        )

# timer
add_library(APP_TIMER_FILES STATIC)

target_sources(APP_TIMER_FILES PUBLIC
        ${APP_PORT_DIR}/timer/src/timer.c
        )

target_include_directories(APP_TIMER_FILES PUBLIC
        ${APP_PORT_DIR}/timer/inc
        )

target_link_libraries(APP_TIMER_FILES PUBLIC
        APP_MCU_FILES      
        )
        
add_library(APP_MBEDTLS_FILES STATIC)

target_sources(APP_MBEDTLS_FILES PUBLIC
        ${APP_PORT_DIR}/mbedtls/src/SSLInterface.c
        ${APP_PORT_DIR}/mbedtls/src/SSL_Random.c
        )

target_include_directories(APP_MBEDTLS_FILES PUBLIC
        ${MBEDTLS_DIR}/include
        ${APP_PORT_DIR}/mbedtls/inc
        ${APP_PORT_DIR}
        )

target_link_libraries(APP_MBEDTLS_FILES PUBLIC
        APP_MCU_FILES
        APP_IOLIBRARY_FILES
        APP_CONFIG_FILES
        mbedcrypto
        mbedx509
        mbedtls
        )

add_library(APP_BOARD_FILES STATIC)

target_sources(APP_BOARD_FILES PUBLIC
        ${APP_PORT_DIR}/board/src/WIZnet_board.c
        )

target_include_directories(APP_BOARD_FILES PUBLIC
        ${APP_PORT_DIR}/board/inc
        ${APP_PORT_DIR}
        )

target_link_libraries(APP_BOARD_FILES PUBLIC
        APP_MCU_FILES
        APP_CONFIG_FILES
        APP_IOLIBRARY_FILES
        APP_PLATFORM_FILES
        )
        
add_library(APP_CONFIG_FILES STATIC)

target_sources(APP_CONFIG_FILES PUBLIC
        ${APP_PORT_DIR}/configuration/src/ConfigData.c
        ${APP_PORT_DIR}/configuration/src/segcp.c
        ${APP_PORT_DIR}/configuration/src/util.c
        )

target_include_directories(APP_CONFIG_FILES PUBLIC
        ${APP_PORT_DIR}/configuration/inc
        ${APP_PORT_DIR}
        )
        
target_link_libraries(APP_CONFIG_FILES PUBLIC
        APP_MCU_FILES
        APP_BOARD_FILES
        APP_IOLIBRARY_FILES
        APP_S2E_FILES
        APP_PLATFORM_FILES
        )

add_library(APP_HTTPSERVER_FILES STATIC)

target_sources(APP_CONFIG_FILES PUBLIC
        ${APP_PORT_DIR}/http_server/src/httpParser.c
        ${APP_PORT_DIR}/http_server/src/httpServer.c
        ${APP_PORT_DIR}/http_server/src/httpUtil.c
        ${APP_PORT_DIR}/configuration/src/util.c
        )

target_include_directories(APP_HTTPSERVER_FILES PUBLIC
        ${APP_PORT_DIR}/html_file
        ${APP_PORT_DIR}/http_server/inc
        ${APP_PORT_DIR}
        )
        
target_link_libraries(APP_HTTPSERVER_FILES PUBLIC
        APP_MCU_FILES
        APP_IOLIBRARY_FILES
        )

add_library(APP_S2E_FILES STATIC)

target_sources(APP_S2E_FILES PUBLIC
        ${APP_PORT_DIR}/serial_to_ethernet/src/seg.c
        )

target_include_directories(APP_S2E_FILES PUBLIC
        ${APP_PORT_DIR}/serial_to_ethernet/inc
        ${APP_PORT_DIR}
        )

target_link_libraries(APP_S2E_FILES PUBLIC
        APP_MCU_FILES
        APP_BOARD_FILES
        APP_CONFIG_FILES
        APP_IOLIBRARY_FILES
        APP_PLATFORM_FILES
        APP_AWS_SDK_PORT_FILES
        )

add_library(APP_PLATFORM_FILES STATIC)

target_sources(APP_PLATFORM_FILES PUBLIC
        ${APP_PORT_DIR}/platform_handler/src/dhcp_cb.c
        ${APP_PORT_DIR}/platform_handler/src/deviceHandler.c
        ${APP_PORT_DIR}/platform_handler/src/dnsHandler.c
        ${APP_PORT_DIR}/platform_handler/src/flashHandler.c
        ${APP_PORT_DIR}/platform_handler/src/gpioHandler.c
        ${APP_PORT_DIR}/platform_handler/src/httpHandler.c
        ${APP_PORT_DIR}/platform_handler/src/netHandler.c
        ${APP_PORT_DIR}/platform_handler/src/storageHandler.c
        ${APP_PORT_DIR}/platform_handler/src/timerHandler.c
        ${APP_PORT_DIR}/platform_handler/src/uartHandler.c
)

target_include_directories(APP_PLATFORM_FILES PUBLIC
        ${APP_PORT_DIR}/platform_handler/inc
        ${APP_PORT_DIR}
)

target_link_libraries(APP_PLATFORM_FILES PUBLIC
        APP_MCU_FILES
        APP_BOARD_FILES
        APP_CONFIG_FILES
        APP_HTTPSERVER_FILES
        APP_IOLIBRARY_FILES
        APP_MBEDTLS_FILES
        APP_MODBUS_FILES
)

add_library(APP_MODBUS_FILES STATIC)

target_sources(APP_MODBUS_FILES PUBLIC
        ${APP_PORT_DIR}/modbus/src/mb.c
        ${APP_PORT_DIR}/modbus/src/mbascii.c
        ${APP_PORT_DIR}/modbus/src/mbcrc.c
        ${APP_PORT_DIR}/modbus/src/mbrtu.c
        ${APP_PORT_DIR}/modbus/src/mbtcp.c
        ${APP_PORT_DIR}/modbus/src/mbtimer.c
        ${APP_PORT_DIR}/modbus/src/mbserial.c
)

target_include_directories(APP_MODBUS_FILES PUBLIC
        ${APP_PORT_DIR}/modbus/inc
        ${APP_PORT_DIR}
)

target_link_libraries(APP_MODBUS_FILES PUBLIC
        APP_MCU_FILES
        APP_BOARD_FILES
)

add_library(APP_AWS_SDK_PORT_FILES STATIC)

target_sources(APP_AWS_SDK_PORT_FILES PUBLIC
        #${APP_PORT_DIR}/aws-iot-device-sdk-embedded-C/src/http_transport_interface.c
        ${APP_PORT_DIR}/aws-iot-device-sdk-embedded-C/src/mqtt_transport_interface.c
        #${APP_PORT_DIR}/aws-iot-device-sdk-embedded-C/src/util.c
        )

target_include_directories(APP_AWS_SDK_PORT_FILES PUBLIC
        ${APP_PORT_DIR}/aws-iot-device-sdk-embedded-C/inc
        )

target_link_libraries(APP_AWS_SDK_PORT_FILES PUBLIC
        APP_MCU_FILES
        APP_BOARD_FILES
        APP_CONFIG_FILES
        APP_IOLIBRARY_FILES
        APP_MBEDTLS_FILES
        AWS_SDK_FILES
        )