/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "../pca9685_i2c/pca9685_i2c.h"
#include "hardware/gpio.h"
#include <map>
#include <functional>

/*
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
}*/

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
    static Button_IRQ Create(uint8_t gpio, gpio_irq_level level = GPIO_IRQ_EDGE_FALL) {
        return Button_IRQ(gpio, level);
    }

    Button_IRQ& set_callback(const std::function<void()>& _cb) {
        cb = _cb;
        gpio_callback_wrapper[gpio] = cb;
        return *this;
    }
};
Button_IRQ::button_callbacks_map_t Button_IRQ::gpio_callback_wrapper = {};

enum GPIO {
    GPIO_10=10,
    GPIO_11,
    GPIO_12,
    GPIO_13,
    GPIO_14,
    GPIO_15,
    GPIO_25
};

int main() {
    stdio_init_all();

    // led blink
    const uint LED_PIN = GPIO_25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // pca9685 on deafult gpio 4 and 5, i2c0, pins 6 and 7
    PCA9685 pca9685;

    // 50Hz for servo driving and set 1500 ms (typically middle position) init value for all servos
    pca9685.init_servo_driver(1500); 

    // Drive servos
    uint16_t servo_position = 1500;

    auto button4 = Button_IRQ::Create(GPIO_13).set_callback(
        [&]() {
            if(servo_position > 0) {
                servo_position -= 100;
            }
            printf("Choosed position %d ms\n", servo_position);
        }
    );
    auto button5 = Button_IRQ::Create(GPIO_14).set_callback(
        [&]() {
            if(servo_position < 2000) {
                servo_position += 100;
            }
            printf("Choosed position %d ms\n", servo_position);
        }
    );

    uint8_t servo_pin = 0;
    auto button0 = Button_IRQ::Create(GPIO_10).set_callback(
        [&]() {
            if(servo_pin > 0) {
                servo_pin -= 1;
            }
            printf("Choosed servo %d\n", servo_pin);
        }
    );
    auto button1 = Button_IRQ::Create(GPIO_11).set_callback(
        [&]() {
            if(servo_pin < 6) {
                servo_pin += 1;
            }
            printf("Choosed servo %d\n", servo_pin);
        }
    );

    auto button2 = Button_IRQ::Create(GPIO_12).set_callback(
        [&]() {
            printf("Setting servo %d position to %d ms\n", servo_pin, servo_position);
            pca9685.set_servo_position(servo_pin, servo_position);
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