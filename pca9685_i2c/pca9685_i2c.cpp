/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define PCA9685_DEFAULT_I2C_ADDR 0x40

#define PCA9685_MODE1_REG       0x00
#define PCA9685_MODE2_REG       0x01
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_ALL_LED_ON_L    0xfa
#define PCA9685_PRESCALE        0xfe

#define PCA9685_SLEEP_BIT       0x10
#define PCA9685_EXTCLK_BIT      0x40
#define PCA9685_RESTART_BIT     0x80

// By default these devices are on bus address 0x40
const int addr = PCA9685_DEFAULT_I2C_ADDR;

#ifdef i2c0

void write_register(uint8_t reg, uint8_t value) {
	uint8_t buff[2] = {reg, value};
	i2c_write_blocking(i2c0, addr, buff, 2, false);
	return;
}

uint8_t read_register(uint8_t reg) {
	i2c_write_blocking(i2c0, addr, &reg, 1, false);	// write register
	i2c_read_blocking(i2c0, addr, &reg, 1, false);		// read value
	return reg;
}

void initialize_pca9685() {
	// configure the PCA9685 for driving servos
	write_register(PCA9685_MODE1_REG, 0xA0);
	write_register(PCA9685_MODE2_REG, 0x04);
}

void set_frequency(uint16_t freq) {
	int preScalerVal = (25000000 / (4096 * freq)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3) preScalerVal = 3;

	// need to be in sleep mode to set the pre-scaler
	uint8_t MODE1_REG = read_register(PCA9685_MODE1_REG);
    printf("Reg: %d\n", MODE1_REG);

	write_register(PCA9685_MODE1_REG, ((MODE1_REG & 0x7F) | 0x10));

    // set prescaler
	write_register(PCA9685_PRESCALE, (uint8_t)preScalerVal);

    // reset
    write_register(PCA9685_MODE1_REG, MODE1_REG);

	sleep_us(500);
}

void set_pwm(uint8_t pin, uint16_t led_on, uint16_t led_off) {
    if (pin > 15) {
        pin = 15;
    }

	uint8_t buffer[5];
	
	buffer[0] = PCA9685_LED0_ON_L + (4 * pin);
	buffer[1] = (0x00FF & led_on);
	buffer[2] = (0xFF00 & led_on) >> 8;
	buffer[3] = (0x00FF & led_off);
	buffer[4] = (0xFF00 & led_off) >> 8;
	
	i2c_write_blocking(i2c0, addr, buffer, 5, false);
}
#endif

#define I2C_SDA 8
#define I2C_SCL 9

int main() {
    printf("Init all\n");
    stdio_init_all();

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));

    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // sleep_ms(200);

    initialize_pca9685();

    // set 1000MHz frequency // tip: default servo working frequency: 50Hz/20ms
    // printf("Call set_frequency\n");
    set_frequency(50);

    while (1) {

        set_pwm(0, 0, 500);
        set_pwm(1, 0, 1500);
        set_pwm(2, 0, 2000);

        gpio_put(LED_PIN, 1);

        sleep_ms(500);

        gpio_put(LED_PIN, 0);

        sleep_ms(500);
    }

    return 0;
}