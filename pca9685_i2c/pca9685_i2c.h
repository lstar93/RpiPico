#ifndef PCA9685_I2C_H
#define PCA9685_I2C_H

#include <memory>
#include <vector>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

enum I2C_SPEED {
    DEFAULT = 100000,
    FAST = 400000
};

class I2C {
    i2c_inst_t* i2c;

public:

    I2C(i2c_inst_t* i2c, uint8_t sda, uint8_t scl, uint32_t i2c_speed)
    : i2c(i2c) {
        // initialize i2c
        stdio_init_all();

        i2c_init(i2c, i2c_speed);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
        // Make the I2C pins available to picotool
        bi_decl(bi_2pins_with_func(sda, scl, GPIO_FUNC_I2C));
    }

    operator i2c_inst_t*() const {
        return i2c;
    }

    void write_register(uint8_t dev_address, uint8_t reg, uint8_t value) const {
        uint8_t buff[2] = {reg, value};
        i2c_write_blocking(i2c, dev_address, buff, 2, false);
    }

    void write_register(uint8_t dev_address, std::vector<uint8_t> values) const {
        auto buffer = std::make_unique<uint8_t[]>(values.size());

        for(int i=0; i<values.size(); ++i) {
            buffer[i] = values[i];
        }

        i2c_write_blocking(i2c, dev_address, buffer.get(), values.size(), false);
    }

    uint8_t read_register(uint8_t dev_address, uint8_t reg) const {
        i2c_write_blocking(i2c, dev_address, &reg, 1, false);	// write register
        i2c_read_blocking(i2c, dev_address, &reg, 1, false);		// read value
        return reg;
    }
};

// PCA9685 registers
#define PCA9685_MODE1_REG       0x00
#define PCA9685_MODE2_REG       0x01
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_ALL_LED_ON_L    0xfa
#define PCA9685_PRESCALE        0xfe
#define PCA9685_SLEEP_BIT       0x10
#define PCA9685_EXTCLK_BIT      0x40
#define PCA9685_RESTART_BIT     0x80

// https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
class PCA9685 {
    const uint8_t address;
    const I2C i2c;

public:
    PCA9685(i2c_inst_t* _i2c = i2c0, uint8_t sda = 4, uint8_t scl = 5, uint8_t _address = 0x40/*default*/, uint32_t i2c_speed = I2C_SPEED::DEFAULT)
    : address(_address),  i2c(_i2c, sda, scl, i2c_speed) {
        
        // initialize device
        i2c.write_register(address, PCA9685_MODE1_REG, 0xA0); // A0 = restart + enable autoincrement
    }

    void set_frequency(uint16_t freq) const {
        // calculate prescaler value
        int preScalerVal = (25000000 / (4096 * freq)) - 1;
        if (preScalerVal > 255) {
            preScalerVal = 255;
        }
        
        if (preScalerVal < 3) {
            preScalerVal = 3;
        }

        // need to be in sleep mode to set the pre-scaler
        uint8_t MODE1_REG = i2c.read_register(address, PCA9685_MODE1_REG);

        // if restart is set also set bit no 4 PCA9685_SLEEP_BIT
        i2c.write_register(address, PCA9685_MODE1_REG, ((MODE1_REG & 0x7F) | PCA9685_SLEEP_BIT)); 

        // set prescaler value
        i2c.write_register(address, PCA9685_PRESCALE, static_cast<uint8_t>(preScalerVal));

        // reset
        i2c.write_register(address, PCA9685_MODE1_REG, MODE1_REG);

        sleep_us(500);
    }

    void set_pwm(uint8_t pin, uint16_t led_on, uint16_t led_off) const {
        if (pin > 15) {
            pin = 15;
        }

        std::vector<uint8_t> buffer(5);
        buffer[0] = PCA9685_LED0_ON_L + (4 * pin);
        buffer[1] = (0x00FF & led_on);
        buffer[2] = (0xFF00 & led_on) >> 8;
        buffer[3] = (0x00FF & led_off);
        buffer[4] = (0xFF00 & led_off) >> 8;

        i2c.write_register(address, buffer);
    }

};

#endif