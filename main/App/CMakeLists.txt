set(TARGET_NAME App)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        APP_MCU_FILES
        APP_IOLIBRARY_FILES
        APP_BOARD_FILES
        APP_CONFIG_FILES
        APP_MBEDTLS_FILES
        APP_S2E_FILES
        APP_TIMER_FILES
        APP_PLATFORM_FILES
        APP_MODBUS_FILES
        APP_AWS_SDK_PORT_FILES
        )

set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "-Wall")
pico_enable_stdio_usb(${TARGET_NAME} 0)
pico_enable_stdio_uart(${TARGET_NAME} 1)
pico_add_extra_outputs(${TARGET_NAME})

set(TARGET_NAME_LINKER ${TARGET_NAME}_linker)
add_executable(${TARGET_NAME_LINKER}
        ${TARGET_NAME}.c
        )
target_link_libraries(${TARGET_NAME_LINKER} PRIVATE
        APP_MCU_FILES
        APP_IOLIBRARY_FILES
        APP_BOARD_FILES
        APP_CONFIG_FILES
        APP_MBEDTLS_FILES
        APP_S2E_FILES
        APP_TIMER_FILES
        APP_PLATFORM_FILES
        )
set_target_properties(${TARGET_NAME_LINKER} PROPERTIES COMPILE_FLAGS "-Wall")
pico_set_linker_script(${TARGET_NAME_LINKER} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME_LINKER}.ld)
pico_enable_stdio_usb(${TARGET_NAME_LINKER} 0)
pico_enable_stdio_uart(${TARGET_NAME_LINKER} 1)
pico_add_extra_outputs(${TARGET_NAME_LINKER})
