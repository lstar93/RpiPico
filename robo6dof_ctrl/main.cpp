/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../pca9685_i2c/pca9685_i2c.h"
#include "../button_irq/button_irq.h"
#include "hardware/gpio.h"
#include <map>
#include <functional>

enum GPIO {
    GPIO_10=10,
    GPIO_11,
    GPIO_12,
    GPIO_13,
    GPIO_14,
    GPIO_15,
    GPIO_25=25
};

// servo driver signal limits
enum towerpro_mg995_limits {
    min = 500,
    max = 2500
};

int main() {
    stdio_init_all();

    // led blink
    const uint LED_PIN = GPIO_25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // pca9685 on deafult gpio 4 and 5, i2c0, pins 6 and 7
    auto pca9685 = std::make_shared<PCA9685>();
    ServoDriver servo_drv(pca9685);

    // 50Hz for servo driving and set 1500 ms (typically middle position) init value for all servos
    // limit servo signal from 500 to 2500
    servo_drv.init_servo_driver(1500, towerpro_mg995_limits::min, towerpro_mg995_limits::max); 

    // Drive servos
    uint16_t servo_position = 1500;
    Button_IRQ::CreateOn(GPIO_13).set_callback(
        [&]() {
            if(servo_position > towerpro_mg995_limits::min) {
                servo_position -= 100;
            }
            printf("Choosed position %d ms\n", servo_position);
        }
    );
    Button_IRQ::CreateOn(GPIO_14).set_callback(
        [&]() {
            if(servo_position < towerpro_mg995_limits::max) {
                servo_position += 100;
            }
            printf("Choosed position %d ms\n", servo_position);
        }
    );

    // Choose servo to drive
    uint8_t servo_pin = 0;
    Button_IRQ::CreateOn(GPIO_10).set_callback(
        [&]() {
            if(servo_pin > 0) {
                servo_pin -= 1;
            }
            printf("Choosed servo %d\n", servo_pin);
        }
    );
    Button_IRQ::CreateOn(GPIO_11).set_callback(
        [&]() {
            if(servo_pin < 6) {
                servo_pin += 1;
            }
            printf("Choosed servo %d\n", servo_pin);
        }
    );

    // Set servo to choosed position
    Button_IRQ::CreateOn(GPIO_12).set_callback(
        [&]() {
            printf("Setting servo %d position to %d ms\n", servo_pin, servo_position);
            servo_drv.set_servo_position(servo_pin, servo_position);
        }
    );

    // Toggle servo edge position
    auto position = towerpro_mg995_limits::min;
    Button_IRQ::CreateOn(GPIO_15).set_callback(
        [&]() {
            printf("Setting servo %d position to %d ms\n", servo_pin, position);
            servo_drv.set_servo_position(servo_pin, position);
            position = (position == towerpro_mg995_limits::min ? towerpro_mg995_limits::max : towerpro_mg995_limits::min); // toggle
        }
    );

    while (1) {
        // led blink
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}