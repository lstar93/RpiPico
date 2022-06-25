#pragma once

#include "../pca9685_i2c/pca9685_i2c.h"
#include "hardware/gpio.h"
#include <map>
#include <functional>

enum gpio_irq_source {
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
}

class Button_IRQ {
    using button_callbacks_map_t = std::map<uint8_t, std::function<void()>>;
    static button_callbacks_map_t gpio_callback_wrapper;

    static void gpio_callback(uint gpio, uint32_t events) {
        // Debounce control
        static unsigned long time = to_ms_since_boot(get_absolute_time());
        static const int delayTime = 300; // Delay for every push button may vary
        if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
            // Recommend to not to change the position of this line
            time = to_ms_since_boot(get_absolute_time());
            
            if(gpio_callback_wrapper.find(gpio) != gpio_callback_wrapper.end()) {
                gpio_callback_wrapper[gpio]();
            }
        }
    }

    uint8_t gpio = 0;
    std::function<void()> cb = 
    [&]() {
        printf("GPIO %d\n", gpio);
    }; 

    Button_IRQ(uint8_t gpio, gpio_irq_level level): gpio(gpio) {
        gpio_set_irq_enabled_with_callback(gpio, level, true, gpio_callback); // create interrupt on gpio, always enabled
    }

public:

    // handle button state change on endge fall by default
    static Button_IRQ CreateOn(uint8_t gpio, gpio_irq_level level = GPIO_IRQ_EDGE_FALL) {
        return Button_IRQ(gpio, level);
    }

    Button_IRQ& set_callback(const std::function<void()>& _cb) {
        cb = _cb;
        gpio_callback_wrapper[gpio] = cb;
        return *this;
    }
};
Button_IRQ::button_callbacks_map_t Button_IRQ::gpio_callback_wrapper = {};