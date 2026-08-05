cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(microcontroller_firmware)

pico_sdk_init()

set(FIRMWARE_EXE_NAME firmware)

add_executable(${FIRMWARE_EXE_NAME}
    src/main.c
    src/runtime_config.c
    src/usb_comm.c
    src/log.c
    src/esc_firmware/update.c
    src/am32/bootloader.c
    src/pwm/pwm.c
    src/pwm/control.c
    src/dshot/dshot.c
    src/dshot/control.c
    src/dshot/telemetry_usb.c
)

target_include_directories(${FIRMWARE_EXE_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

pico_generate_pio_header(${FIRMWARE_EXE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/dshot/dshot.pio)

target_link_libraries(${FIRMWARE_EXE_NAME}
    pico_stdlib
    pico_stdio_usb
    hardware_clocks
    hardware_gpio
    hardware_pwm
    hardware_pio
    hardware_sync
)

pico_add_extra_outputs(${FIRMWARE_EXE_NAME})
