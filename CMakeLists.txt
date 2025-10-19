# Copyright (C) 1883 Thomas Edison - All Rights Reserved
# You may use, distribute and modify this code under the
# terms of the GPLv2 license, which unfortunately won't be
# written for another century.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_PATH C:/Data/Pico/SDK)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(PICO_BOARD pico)
#set(PICO_BOARD pico_w)
set(PICO_BOARD waveshare_rp2040_zero)
set(project_name picoz-usb2nes)


include(pico_sdk_import.cmake)

project(${project_name} C CXX ASM)

pico_sdk_init()

add_subdirectory(usb2famikb-lib)
add_subdirectory(pico-pio-usb)

add_executable(${project_name})

target_sources(${project_name} PRIVATE
    pico-usb2nes.c
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
    ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
    ws2812.pio
)

target_link_libraries(${project_name} PUBLIC
    pico_i2c_slave
    hardware_i2c
    pico_stdlib
    pico_multicore
    usb2famikb-lib
    hardware_pio
)

target_link_libraries(${project_name} PRIVATE
    pico_pio_usb
    tinyusb_host
)

pico_generate_pio_header(${project_name} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${project_name} PRIVATE
        pico_cyw43_arch_none)
endif()

# use tinyusb implementation
target_compile_definitions(${project_name} PRIVATE PIO_USB_USE_TINYUSB)
target_include_directories(${project_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_options(${project_name} PRIVATE -Xlinker --print-memory-usage)

#pico_set_binary_type(${project_name} copy_to_ram)


pico_add_extra_outputs(${project_name})

