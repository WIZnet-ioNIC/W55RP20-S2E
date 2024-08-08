set(TARGET_NAME Boot)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        BOOT_MCU_FILES
        BOOT_IOLIBRARY_FILES
        BOOT_BOARD_FILES
        BOOT_CONFIG_FILES
        BOOT_TIMER_FILES
        BOOT_PLATFORM_FILES
)

function(target_cl_options option)
    target_compile_options(${TARGET_NAME} PRIVATE ${option})
    target_link_options(${TARGET_NAME} PRIVATE ${option})
endfunction()

target_cl_options("-ffunction-sections")
target_cl_options("-fdata-sections")
target_link_options(${TARGET_NAME} PRIVATE "LINKER:--gc-sections")

pico_enable_stdio_usb(${TARGET_NAME} 0)
pico_enable_stdio_uart(${TARGET_NAME} 1)

pico_add_extra_outputs(${TARGET_NAME})
pico_set_binary_type(${TARGET_NAME} copy_to_ram)
set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "-Wall")

pico_set_linker_script(${TARGET_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME}.ld)
