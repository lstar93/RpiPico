/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../pca9685_i2c/pca9685_i2c.h"
#include "hardware/gpio.h"
#include <map>
#include <functional>

/*enum gpio_irq_source {
    LEVEL_LOW,
    LEVEL_HIGH,
    EDGE_FALL,
    EDGE_RISE,
    UNKNOWN
};

std::map<uint8_t, std::string> gpio_irq_code_to_str = {
    {LEVEL_LOW, "LEVEL_LOW"},
    {LEVEL_HIGH, "LEVEL_HIGH"},
    {EDGE_FALL, "EDGE_FALL"},
    {EDGE_RISE, "EDGE_RISE"},
    {UNKNOWN, "UNKNOWN"}
};

gpio_irq_source gpio_get_event_id(uint32_t events) {
    for (uint i = 0; i < 4; i++) {
        uint mask = (1 << i);
        if (events & mask) {
            return static_cast<gpio_irq_source>(i);
        }
    }
    return UNKNOWN;
}*/

using button_callbacks_map_t = std::map<uint8_t, std::function<void()>>;
button_callbacks_map_t gpio_callback_wrapper;

class Button_IRQ {

    static void gpio_callback(uint gpio, uint32_t events) {
        // Check interrupt event id
        // auto evt_id = gpio_get_event_id(events);
        // printf("GPIO %d %d %s\n", gpio, evt_id, gpio_irq_code_to_str[evt_id].c_str());
        if(gpio_callback_wrapper.find(gpio) != gpio_callback_wrapper.end()) {
            gpio_callback_wrapper[gpio]();
        }
    }

    uint8_t gpio = 0;

public:
    Button_IRQ(uint8_t gpio, gpio_irq_level level, bool enabled): gpio(gpio){
        gpio_set_irq_enabled_with_callback(gpio, level, enabled, gpio_callback);
        gpio_callback_wrapper[gpio] = std::bind(&Button_IRQ::callback, *this);
    }

    void callback() {
        printf("Button %d\n", gpio);
    }
};

int main() {
    stdio_init_all();

    // led blink
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // pca9685 on deafult gpio 4 and 5 i2c0, pins 7 and 8
    /*PCA9685 pca9685;

    // 50Hz for servo driving and set 1500 ms (middle) init value for all servos
    pca9685.init_servo(1500); 

    // std::vector<uint16_t> servo_positions(6);*/

    // gpio_set_irq_enabled_with_callback(10, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    // gpio_set_irq_enabled_with_callback(11, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    // gpio_set_irq_enabled_with_callback(12, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    // gpio_set_irq_enabled_with_callback(13, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    // gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    //gpio_set_irq_enabled_with_callback(15, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    Button_IRQ button10(10, GPIO_IRQ_EDGE_RISE, true);
    Button_IRQ button11(11, GPIO_IRQ_EDGE_RISE, true);
    Button_IRQ button12(12, GPIO_IRQ_EDGE_RISE, true);
    Button_IRQ button13(13, GPIO_IRQ_EDGE_RISE, true);
    Button_IRQ button14(14, GPIO_IRQ_EDGE_RISE, true);
    Button_IRQ button15(15, GPIO_IRQ_EDGE_RISE, true);

    while (1) {
        // led blink
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
        // printf("ms: %d, \n", cnt);
    }

    return 0;
}