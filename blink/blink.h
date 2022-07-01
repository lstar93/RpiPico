#pragma once

#include "pico/stdlib.h"

class Blinker {
    const uint LED_PIN;
public:
    Blinker(const uint LED_PIN = 25): LED_PIN(LED_PIN) {
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
    }

    void blink(uint16_t delay) {
        gpio_put(LED_PIN, 1);
        sleep_ms(delay);
        gpio_put(LED_PIN, 0);
        sleep_ms(delay);
    }

    void blink_non_block() {
        static bool blink = true;
        gpio_put(LED_PIN, blink);
        blink = blink ? false : true;
    }
};