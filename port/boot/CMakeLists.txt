
add_library(BOOT_MCU_FILES STATIC)

target_include_directories(BOOT_MCU_FILES PUBLIC
        ${BOOT_PORT_DIR}
)

target_link_libraries(BOOT_MCU_FILES PUBLIC 
        cmsis_core
        pico_stdlib
        pico_stdlib_headers
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
        tinyusb_device_unmarked
	tinyusb_board
        )

add_library(BOOT_IOLIBRARY_FILES STATIC)

target_sources(BOOT_IOLIBRARY_FILES PUBLIC 
        ${BOOT_PORT_DIR}/ioLibrary_Driver/src/w5x00_spi.c
        )

if(${BOARD_NAME} STREQUAL W55RP20_S2E OR ${BOARD_NAME} STREQUAL W232N OR ${BOARD_NAME} STREQUAL IP20 OR ${BOARD_NAME} STREQUAL PLATYPUS_S2E)
pico_generate_pio_header(BOOT_IOLIBRARY_FILES ${BOOT_PORT_DIR}/ioLibrary_Driver/src/wiznet_spi_pio.pio)

target_sources(BOOT_IOLIBRARY_FILES PUBLIC 
        ${BOOT_PORT_DIR}/ioLibrary_Driver/src/wiznet_spi_pio.c
        )
endif()

target_include_directories(BOOT_IOLIBRARY_FILES PUBLIC 
        ${WIZNET_DIR}/Ethernet
        ${BOOT_PORT_DIR}/ioLibrary_Driver/inc
        ${BOOT_PORT_DIR}
        )

target_link_libraries(BOOT_IOLIBRARY_FILES PUBLIC 
        BOOT_MCU_FILES
        BOOT_BOARD_FILES
        DHCP_FILES
        ETHERNET_FILES
        )

# timer
add_library(BOOT_TIMER_FILES STATIC)

target_sources(BOOT_TIMER_FILES PUBLIC
        ${BOOT_PORT_DIR}/timer/src/timer.c
        )

target_include_directories(BOOT_TIMER_FILES PUBLIC
        ${BOOT_PORT_DIR}/timer/inc
        )

target_link_libraries(BOOT_TIMER_FILES PRIVATE
        BOOT_MCU_FILES      
        )

add_library(BOOT_BOARD_FILES STATIC)

target_sources(BOOT_BOARD_FILES PUBLIC
        ${BOOT_PORT_DIR}/board/src/WIZnet_board.c
        )

target_include_directories(BOOT_BOARD_FILES PUBLIC
        ${BOOT_PORT_DIR}/board/inc
        ${BOOT_PORT_DIR}
        )

target_link_libraries(BOOT_BOARD_FILES PUBLIC
        BOOT_MCU_FILES
        BOOT_CONFIG_FILES
        BOOT_IOLIBRARY_FILES
        BOOT_PLATFORM_FILES
        )
        
add_library(BOOT_CONFIG_FILES STATIC)

target_sources(BOOT_CONFIG_FILES PUBLIC
        ${BOOT_PORT_DIR}/configuration/src/ConfigData.c
        ${BOOT_PORT_DIR}/configuration/src/segcp.c
        ${BOOT_PORT_DIR}/configuration/src/util.c
        )

target_include_directories(BOOT_CONFIG_FILES PUBLIC
        ${BOOT_PORT_DIR}/configuration/inc
        ${BOOT_PORT_DIR}
        )
        
target_link_libraries(BOOT_CONFIG_FILES PUBLIC
        BOOT_MCU_FILES
        BOOT_BOARD_FILES
        BOOT_IOLIBRARY_FILES
        BOOT_PLATFORM_FILES
        BOOT_S2E_FILES
        )

add_library(BOOT_S2E_FILES STATIC)

target_sources(BOOT_S2E_FILES PUBLIC
        ${BOOT_PORT_DIR}/serial_to_ethernet/src/seg.c
        )

target_include_directories(BOOT_S2E_FILES PUBLIC
        ${BOOT_PORT_DIR}/serial_to_ethernet/inc
        ${BOOT_PORT_DIR}
        )

target_link_libraries(BOOT_S2E_FILES PUBLIC
        BOOT_MCU_FILES
        BOOT_BOARD_FILES
        BOOT_CONFIG_FILES
        BOOT_IOLIBRARY_FILES
        BOOT_PLATFORM_FILES
        )

add_library(BOOT_PLATFORM_FILES STATIC)

if(${BOARD_NAME} STREQUAL W55RP20_S2E OR ${BOARD_NAME} STREQUAL W232N OR ${BOARD_NAME} STREQUAL IP20 OR ${BOARD_NAME} STREQUAL PLATYPUS_S2E)
pico_generate_pio_header(BOOT_PLATFORM_FILES ${BOOT_PORT_DIR}/platform_handler/src/uart_tx.pio)
endif()

target_sources(BOOT_PLATFORM_FILES PUBLIC
        ${BOOT_PORT_DIR}/platform_handler/src/dhcp_cb.c
        ${BOOT_PORT_DIR}/platform_handler/src/deviceHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/flashHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/gpioHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/storageHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/timerHandler.c
        ${BOOT_PORT_DIR}/platform_handler/src/uartHandler.c
)

target_include_directories(BOOT_PLATFORM_FILES PUBLIC
        ${BOOT_PORT_DIR}/platform_handler/inc
        ${BOOT_PORT_DIR}
)

target_link_libraries(BOOT_PLATFORM_FILES PUBLIC
        BOOT_MCU_FILES
        BOOT_BOARD_FILES
        BOOT_CONFIG_FILES
        BOOT_IOLIBRARY_FILES
        BOOT_S2E_FILES
)