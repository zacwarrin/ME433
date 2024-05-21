cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(hunter_pico_examples C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(HUNTER_PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# # Initialize the SDK
pico_sdk_init()

# add_compile_options(-Wall
#         -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
#         -Wno-unused-function # we have some for the docs that aren't called
#         -Wno-maybe-uninitialized
#         )

# add_executable(accel )

# target_sources(accel PRIVATE accel.c)


# pico_enable_stdio_usb(accel 1) 
# pico_enable_stdio_uart(accel 0)

# # Pull in our pico_stdlib which pulls in commonly used features
# target_link_libraries(accel pico_stdlib hardware_i2c hardware_adc)

# # create map/bin/hex file etc.
# pico_add_extra_outputs(accel)

add_executable(dev_hid_composite)

target_sources(dev_hid_composite PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
        )

# Make sure TinyUSB can find tusb_config.h
target_include_directories(dev_hid_composite PUBLIC
        ${CMAKE_CURRENT_LIST_DIR})

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(dev_hid_composite PUBLIC pico_stdlib pico_unique_id tinyusb_device tinyusb_board hardware_i2c)

# Uncomment this line to enable fix for Errata RP2040-E5 (the fix requires use of GPIO 15)
#target_compile_definitions(dev_hid_composite PUBLIC PICO_RP2040_USB_DEVICE_ENUMERATION_FIX=1)

pico_add_extra_outputs(dev_hid_composite)

# add url via pico_set_program_url
# example_auto_set_url(dev_hid_composite)
