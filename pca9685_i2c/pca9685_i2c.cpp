/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pca9685_i2c.h"

int main() {

    // led blink
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    const uint8_t SDA = 8;
    const uint8_t SCL = 9;
    PCA9685 pca9685(SDA, SCL);
    pca9685.set_frequency(50); // 50Hz for servo driving

    while (1) {

        pca9685.set_pwm(0, 0, 500);
        pca9685.set_pwm(1, 0, 1500);
        pca9685.set_pwm(2, 0, 2000);

        // led blink
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}