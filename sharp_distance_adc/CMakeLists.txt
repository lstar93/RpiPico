cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(sharp_distance C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(sharp_distance
        main.cpp
        )

pico_enable_stdio_usb(sharp_distance 1)
pico_enable_stdio_uart(sharp_distance 1)

include_directories(../)

# pull in common dependencies and additional i2c hardware support, adc for potentiometes
target_link_libraries(sharp_distance pico_stdlib hardware_adc hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(sharp_distance)

# add url via pico_set_program_url
# example_auto_set_url(sharp_distance)