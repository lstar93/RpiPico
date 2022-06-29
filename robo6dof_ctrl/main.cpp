/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pca9685_i2c/pca9685_i2c.h"
#include "button_irq/button_irq.h"
#include "hardware/gpio.h"
#include <map>
#include <functional>
#include <sstream>

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

// servo driver signal limits
enum towerpro_mg995_limits {
    min = 500,
    max = 2500
};

using roboarm_pose_t = std::map<uint8_t, uint16_t>; // pin to signal width map
std::string roboarm_pose_to_str(roboarm_pose_t& pose) {
    if(!pose.empty()) {  
        std::ostringstream os;
        os << "{ ";
        for(const auto& [pin, signal]: pose) {
            os << std::to_string(pin) << ":" << signal << " ";
        }
        os << "}";
        return os.str();
    }
    return "{}";    
}

int main() {
    stdio_init_all();

    // led blink
    const uint LED_PIN = GPIO_25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // pca9685 on deafult gpio 4 and 5, i2c0, pins 6 and 7
    auto pca9685 = std::make_shared<PCA9685>();
    ServoDriver servo_drv(pca9685);

    // 50Hz for servo driving and set 1500 ms (typically middle position) init value for all servos
    // limit servo signal from 500 to 2500
    servo_drv.init_servo_driver(1500, towerpro_mg995_limits::min, towerpro_mg995_limits::max); 

    // Drive servos
    uint16_t servo_position = 1500;
    Button_IRQ::CreateOn(GPIO_13).set_callback(
        [&]() {
            if(servo_position > towerpro_mg995_limits::min) {
                servo_position -= 100;
            }
            printf("Choosed position %d ms\n", servo_position);
        }
    );
    Button_IRQ::CreateOn(GPIO_14).set_callback(
        [&]() {
            if(servo_position < towerpro_mg995_limits::max) {
                servo_position += 100;
            }
            printf("Choosed position %d ms\n", servo_position);
        }
    );

    // Choose servo to drive
    uint8_t servo_pin = 0;
    Button_IRQ::CreateOn(GPIO_10).set_callback(
        [&]() {
            if(servo_pin > 0) {
                servo_pin -= 1;
            }
            printf("Choosed servo %d\n", servo_pin);
        }
    );
    Button_IRQ::CreateOn(GPIO_11).set_callback(
        [&]() {
            if(servo_pin < 5) {
                servo_pin += 1;
            }
            printf("Choosed servo %d\n", servo_pin);
        }
    );

    roboarm_pose_t roboarm_pose; // prepare vector for robo arm pose
    // Add servo position to vector
    Button_IRQ::CreateOn(GPIO_12).set_callback(
        [&]() {
            printf("Adding servo %d position %d ms to pose vector\n", servo_pin, servo_position);
            //servo_drv.set_servo_position(servo_pin, servo_position);
            roboarm_pose[servo_pin] = servo_position;
            servo_position = 1500; // reset value
            printf("Current pose vector %s\n", roboarm_pose_to_str(roboarm_pose).c_str());
        }
    );

    // Add all servos positions to vector of roboarm position
    Button_IRQ::CreateOn(GPIO_15).set_callback(
        [&]() {
            if(roboarm_pose.size() < 6) {
                printf("Pose should have at least 6 elements instead of %d!\n", roboarm_pose.size());
                return;
            }
            printf("Setting pose: %s\n", roboarm_pose_to_str(roboarm_pose).c_str());
            for(const auto& [pin, signal]: roboarm_pose) {
                servo_drv.set_servo_position(pin, signal);
            }
            roboarm_pose.clear(); // prepare map for another pose
            servo_pin = 0; // back to first servo
        }
    );

    // Toggle servo edge position
    /*auto position = towerpro_mg995_limits::min;
    Button_IRQ::CreateOn(GPIO_15).set_callback(
        [&]() {
            printf("Setting servo %d position to %d ms\n", servo_pin, position);
            servo_drv.set_servo_position(servo_pin, position);
            position = (position == towerpro_mg995_limits::min ? towerpro_mg995_limits::max : towerpro_mg995_limits::min); // toggle
        }
    );*/

    while (1) {
        // led blink
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    return 0;
}