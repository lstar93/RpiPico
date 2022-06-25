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
    PCA9685 pca9685(i2c0, SDA, SCL);
    ServoDriver servo_drv(&pca9685);

    // 50Hz for servo driving and set 1500 ms (middle) init value for all servos
    servo_drv.init_servo_driver(1500); 

    while (1) {

        servo_drv.set_servo_position(0, 2000);
        servo_drv.set_servo_position(1, 1300);
        servo_drv.set_servo_position(2, 1700);
        servo_drv.set_servo_position(3, 1000);

        /*
        pca9685.set_pwm(0, 0, 300);
        pca9685.set_pwm(1, 0, 300);
        pca9685.set_pwm(2, 0, 400);
        pca9685.set_pwm(3, 0, 200);
        pca9685.set_pwm(4, 0, 300);
        */

        // led blink
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}