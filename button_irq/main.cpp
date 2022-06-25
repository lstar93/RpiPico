/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../pca9685_i2c/pca9685_i2c.h"
#include "button_irq.h"
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
    GPIO_25
};

void test_func() {
    printf("TEST\n");
}

int main() {
    stdio_init_all();

    // led blink
    const uint LED_PIN = GPIO_25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    auto button0 = Button_IRQ::Create(GPIO_13).set_callback(
        [&]() {
            printf("Test from lambda\n");
        }
    );

    auto button1 = Button_IRQ::Create(GPIO_14).set_callback(test_func);

    while (1) {
        // led blink
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}