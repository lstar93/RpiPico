/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "blink/blink.h"
#include "button_irq/button_irq.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <map>
#include <functional>
#include <sstream>

class NoSleepDelayFunctionCaller {
    unsigned long time = to_ms_since_boot(get_absolute_time());
    int delayTime = 0; // delay for task
public:

    void set_delay(const int delayTime) {
        this->delayTime = delayTime;
    }

    template<class Func> 
    void call_func_non_block(Func f){
        if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
            // Recommend to not to change the position of this line
            time = to_ms_since_boot(get_absolute_time());
            f();
        }
    }

    template<class Ret, class Func> 
    Ret call_func_non_block(Func f){
        if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
            // Recommend to not to change the position of this line
            time = to_ms_since_boot(get_absolute_time());
            return f();
        }
    }
};

enum GPIO {
    GPIO_10=10,
    GPIO_11,
    GPIO_12,
    GPIO_13,
    GPIO_14,
    GPIO_15,
    GPIO_16,
    GPIO_25=25
};

enum GPIO_ADC {
    GPIO_26 = 26,
    GPIO_27
};

/*template<class Func> 
void call_func_non_block(unsigned long currentTime, const int delayTime, Func f){
    if ((to_ms_since_boot(get_absolute_time())-time)>delayTime) {
        // Recommend to not to change the position of this line
        time = to_ms_since_boot(get_absolute_time());
        f();
    }
}*/

NoSleepDelayFunctionCaller blinkerCaller, adcReadCaller;

int main() {
    // led blink
    Blinker blinker;

    stdio_init_all();

    adc_init();
    adc_gpio_init(GPIO_26);
    adc_select_input(0);

    const int blinkDealyTime = 300; // delay for blink
    const int adcDelayTime = 100; // Delay for every adc read

    blinkerCaller.set_delay(blinkDealyTime);
    adcReadCaller.set_delay(adcDelayTime);

    auto adc_reading = []() {
                            uint16_t result = adc_read();
                            printf("Raw value: %d\n", result);
                       };

    while (1) {

        // blink
        blinkerCaller.call_func_non_block(std::bind(&Blinker::blink_non_block, &blinker));

        // read adc
        adcReadCaller.call_func_non_block(adc_reading);
    }

    return 0;
}