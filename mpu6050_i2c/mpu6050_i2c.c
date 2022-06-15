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

/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/

static int muxaddr = 0x70;
static void enable_multiplexer(uint8_t bus) {
    // Enable all channels
    uint8_t busnum = 1 << bus;
    i2c_write_blocking(i2c_default, muxaddr, &busnum, 1, false);
}

// By default these devices  are on bus address 0x68
static int addr = 0x68;

#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
#endif

int main() {
    stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    printf("Hello, MPU6050! Reading raw data from registers...\n");

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    enable_multiplexer(0);
    sleep_ms(100);
    mpu6050_reset();
    sleep_ms(100);
    enable_multiplexer(2);
    sleep_ms(100);
    mpu6050_reset();
    sleep_ms(100);
    enable_multiplexer(4);
    sleep_ms(100);
    mpu6050_reset();
    sleep_ms(100);

    int16_t acceleration0[3], gyro0[3], temp0;
    int16_t acceleration1[3], gyro1[3], temp1;
    int16_t acceleration2[3], gyro2[3], temp2;

    uint32_t cnt = 0;
    while (1) {

        if(cnt%3 == 0) {
            enable_multiplexer(0);
            mpu6050_read_raw(acceleration0, gyro0, &temp0);
        } else if(cnt%3 == 1) {
            enable_multiplexer(2);
            mpu6050_read_raw(acceleration1, gyro1, &temp1);
        } else if(cnt%3 == 2) {
            enable_multiplexer(4);
            mpu6050_read_raw(acceleration2, gyro2, &temp2);

            // These are the raw numbers from the chip, so will need tweaking to be really useful.
            // See the datasheet for more information
            printf("Acc0. X = %d, Y = %d, Z = %d || Acc1. X = %d, Y = %d, Z = %d || Acc2. X = %d, Y = %d, Z = %d\n", acceleration0[0], acceleration0[1], acceleration0[2], acceleration1[0], acceleration1[1], acceleration1[2], acceleration2[0], acceleration2[1], acceleration2[2]);
            printf("Gyro0. X = %d, Y = %d, Z = %d || Gyro1. X = %d, Y = %d, Z = %d || Gyro2. X = %d, Y = %d, Z = %d\n", gyro0[0], gyro0[1], gyro0[2], gyro1[0], gyro1[1], gyro1[2], gyro2[0], gyro2[1], gyro2[2]);
            // Temperature is simple so use the datasheet calculation to get deg C.
            // Note this is chip temperature.
            printf("Temp0. = %f || Temp1. = %f || Temp2. = %f\n", (temp0 / 340.0) + 36.53, (temp1 / 340.0) + 36.53, (temp2 / 340.0) + 36.53);
        }
        ++cnt;

        sleep_ms(100);
    }

#endif
    return 0;
}
