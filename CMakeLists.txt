cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(microcontroller_firmware)

pico_sdk_init()

set(FIRMWARE_EXE_NAME firmware)
set(MANAFISH_RELEASE_VERSION "" CACHE STRING "Exact Manafish MCU release version")
execute_process(COMMAND git rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE CONTROL_GIT_ID OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT CONTROL_GIT_ID)
    set(CONTROL_GIT_ID "source-export")
endif()
execute_process(COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE CONTROL_GIT_DIRTY OUTPUT_STRIP_TRAILING_WHITESPACE)
if(CONTROL_GIT_DIRTY)
    string(APPEND CONTROL_GIT_ID "-dirty")
endif()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/release_version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/release_version.h
    @ONLY
)

add_executable(${FIRMWARE_EXE_NAME}
    src/main.c
    src/control/controller.c
    src/control/protocol.c
    src/control/runtime.c
    src/imu/bmi270_sensor.c
    src/usb_tx.c
    src/usb_rx.c
    third_party/bmi270/bmi2.c
    third_party/bmi270/bmi270.c
    src/runtime_config.c
    src/usb_comm.c
    src/log.c
    src/esc_firmware/update.c
    src/am32/bootloader.c
    src/pwm/pwm.c
    src/pwm/control.c
    src/dshot/dshot.c
    src/dshot/control.c
    src/dshot/current_sensing.c
    src/dshot/telemetry_usb.c
)

target_include_directories(
    ${FIRMWARE_EXE_NAME}
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}/generated
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(${FIRMWARE_EXE_NAME} SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/bmi270)

target_compile_definitions(
    ${FIRMWARE_EXE_NAME}
    PRIVATE
        CONTROL_BUILD_IDENTITY="pico-control-dev:${CONTROL_GIT_ID}"
        PICO_STACK_SIZE=0x4000
        PICO_CORE1_STACK_SIZE=0x800
        CFG_TUD_CDC_RX_BUFSIZE=2048
        CFG_TUD_CDC_TX_BUFSIZE=512
        PICO_STDIO_USB_DEFAULT_CRLF=0
        PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
)

# Keep compiler stack-usage evidence alongside the linker map for RP2040 budget review.
target_compile_options(${FIRMWARE_EXE_NAME} PRIVATE $<$<COMPILE_LANGUAGE:C>:-fstack-usage>)
pico_generate_pio_header(${FIRMWARE_EXE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/dshot/dshot.pio)

target_link_libraries(${FIRMWARE_EXE_NAME}
    pico_stdlib
    pico_stdio_usb
    pico_multicore
    hardware_spi
    hardware_watchdog
    hardware_clocks
    hardware_gpio
    hardware_pwm
    hardware_pio
    hardware_sync
)

# The SDK normally places core0's stack in a 4 KiB scratch bank. The bounded
# frame codec needs a real 16 KiB reservation, not a stack-size symbol whose
# range silently overlaps the heap. Core1 uses its explicit 8 KiB BSS stack.
set(CONTROL_MEMORY_PLATFORM rp2040)
if(PICO_PLATFORM MATCHES "^rp2350")
    set(CONTROL_MEMORY_PLATFORM rp2350)
endif()
file(READ "${PICO_SDK_PATH}/src/rp2_common/pico_crt0/${CONTROL_MEMORY_PLATFORM}/memmap_default.ld"
    CONTROL_LINKER_SCRIPT)
string(REPLACE ".stack_dummy (NOLOAD):" ".stack_dummy ORIGIN(RAM) + LENGTH(RAM) - 0x4000 (NOLOAD):"
    CONTROL_LINKER_SCRIPT "${CONTROL_LINKER_SCRIPT}")
string(REPLACE "KEEP(*(.stack*))\n    } > SCRATCH_Y" "KEEP(*(.stack*))\n    } > RAM"
    CONTROL_LINKER_SCRIPT "${CONTROL_LINKER_SCRIPT}")
string(REPLACE "__StackTop = ORIGIN(SCRATCH_Y) + LENGTH(SCRATCH_Y);"
    "__StackTop = ORIGIN(RAM) + LENGTH(RAM);" CONTROL_LINKER_SCRIPT "${CONTROL_LINKER_SCRIPT}")
string(REPLACE "__HeapLimit = ORIGIN(RAM) + LENGTH(RAM);"
    "__HeapLimit = ORIGIN(RAM) + LENGTH(RAM) - 0x4000;" CONTROL_LINKER_SCRIPT "${CONTROL_LINKER_SCRIPT}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/control_memmap.ld" "${CONTROL_LINKER_SCRIPT}")
pico_set_linker_script(${FIRMWARE_EXE_NAME} "${CMAKE_CURRENT_BINARY_DIR}/control_memmap.ld")
pico_add_extra_outputs(${FIRMWARE_EXE_NAME})
