cmake_minimum_required(VERSION 3.13)

# Separate project: never include the controller's CMakeLists or source list.
set(PICO_BOARD pico CACHE STRING "Original RP2040 Pico only")
set(PICO_SDK_FETCH_FROM_GIT OFF CACHE BOOL "Use the existing Nix SDK" FORCE)
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)
project(manafish_spi_probe C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
pico_sdk_init()
if(NOT PICO_BOARD STREQUAL "pico" OR NOT PICO_PLATFORM STREQUAL "rp2040")
    message(FATAL_ERROR "This diagnostic is restricted to the original RP2040 Pico")
endif()
if(NOT PICO_SDK_VERSION_STRING VERSION_EQUAL "2.1.1")
    message(FATAL_ERROR "Use the repository's locked Pico SDK 2.1.1 via nix develop")
endif()
option(SPI_PROBE_COMPARE_PULLS "Append separately labelled weak-MISO-pull comparisons" OFF)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS probe.c)
file(SHA256 ${CMAKE_CURRENT_LIST_DIR}/probe.c SPI_PROBE_SOURCE_SHA256)
configure_file(probe_build.h.in generated/probe_build.h @ONLY)
add_executable(spi_probe probe.c)
target_include_directories(spi_probe PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_compile_options(spi_probe PRIVATE -Wall -Wextra -Werror -fstack-usage)
target_compile_definitions(spi_probe PRIVATE
    SPI_PROBE_COMPARE_PULLS=$<BOOL:${SPI_PROBE_COMPARE_PULLS}>
    PICO_STACK_SIZE=0x1000
    PICO_CORE1_STACK_SIZE=0x800
    USBD_PRODUCT="Manafish BMI270 read-only SPI probe"
    USBD_DESC_STR_MAX=36 # 35 product characters plus the descriptor header word.
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
    PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0
    PICO_STDIO_USB_RESET_BOOTSEL_FIXED_ACTIVITY_LED=1
    PICO_STDIO_USB_RESET_BOOTSEL_ACTIVITY_LED=-1
    PICO_STDIO_USB_DEFAULT_CRLF=0)
target_link_libraries(spi_probe PRIVATE pico_stdlib pico_stdio_usb pico_multicore
    hardware_spi hardware_sync hardware_watchdog)
pico_enable_stdio_uart(spi_probe 0)
pico_enable_stdio_usb(spi_probe 1)
pico_set_program_name(spi_probe "manafish-bmi270-read-only-spi-probe")
pico_set_program_version(spi_probe "diagnostic-1")
pico_add_extra_outputs(spi_probe)
