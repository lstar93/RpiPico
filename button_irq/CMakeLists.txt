cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(button_irq C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(button_irq
        main.cpp
        button_irq.h
        )

pico_enable_stdio_usb(button_irq 1)
pico_enable_stdio_uart(button_irq 1)

# pull in common dependencies and additional i2c hardware support, adc for potentiometes
target_link_libraries(button_irq pico_stdlib hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(button_irq)

# add url via pico_set_program_url
# example_auto_set_url(button_irq)