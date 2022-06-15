cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pca9685 C CXX ASM)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(pca9685_i2c
        pca9685_i2c.cpp
        )

pico_enable_stdio_usb(pca9685_i2c 1)
pico_enable_stdio_uart(pca9685_i2c 1)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(pca9685_i2c pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(pca9685_i2c)

# add url via pico_set_program_url
# example_auto_set_url(pca9685_i2c)