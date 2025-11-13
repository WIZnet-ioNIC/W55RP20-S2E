set(TARGET_NAME SPI_Mode_Master)

add_executable(${TARGET_NAME}
        ${TARGET_NAME}.c
        )

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_stdlib 
        hardware_spi 
        hardware_clocks
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

set_target_properties(${TARGET_NAME} PROPERTIES COMPILE_FLAGS "-Wall")
pico_add_extra_outputs(${TARGET_NAME})