cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(mpu6050 C CXX ASM)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(mpu6050_i2c
        mpu6050_i2c.c
        )

pico_enable_stdio_usb(mpu6050_i2c 1)
pico_enable_stdio_uart(mpu6050_i2c 1)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(mpu6050_i2c pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu6050_i2c)

# add url via pico_set_program_url
# example_auto_set_url(mpu6050_i2c)