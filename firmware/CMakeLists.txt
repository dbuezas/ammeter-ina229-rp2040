cmake_minimum_required(VERSION 3.13)

# The Pico SDK is not included. Set PICO_SDK_PATH to a checkout (default ~/pico-sdk).
if(NOT DEFINED ENV{PICO_SDK_PATH})
    set(ENV{PICO_SDK_PATH} "$ENV{HOME}/pico-sdk")
endif()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ina229_ammeter C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(ina229_ammeter main.c)
target_link_libraries(ina229_ammeter pico_stdlib hardware_spi)

# The USB send buffer is 64 bytes by default, but a CSV row is about 85. Rows that do not
# fit get cut or dropped with no error, so make room for several rows.
#
# When the buffer is full, stdio_usb waits this long for the host and then drops the rest.
# A long wait is fine: the totals are kept in the INA229's accumulators anyway.
target_compile_definitions(ina229_ammeter PRIVATE
    CFG_TUD_CDC_TX_BUFSIZE=1024
    CFG_TUD_CDC_RX_BUFSIZE=256
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000000
)

# USB only. GP0/GP1 stay free, e.g. for a logic analyser.
pico_enable_stdio_usb(ina229_ammeter 1)
pico_enable_stdio_uart(ina229_ammeter 0)

pico_add_extra_outputs(ina229_ammeter)
