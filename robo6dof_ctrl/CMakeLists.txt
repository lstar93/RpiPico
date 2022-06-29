cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(robo6dof C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(robo6dof
        main.cpp
        )

pico_enable_stdio_usb(robo6dof 1)
pico_enable_stdio_uart(robo6dof 1)

# pull in common dependencies and additional i2c hardware support, adc for potentiometes
target_link_libraries(robo6dof pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(robo6dof)

# add url via pico_set_program_url
# example_auto_set_url(robo6dof)