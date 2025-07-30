#pragma once

#include <driver/i2c.h>
#include <driver/gpio.h>
#include <cstdint>

namespace bmi270
{

    struct I2CConfig
    {
        i2c_port_t port = I2C_NUM_0;
        gpio_num_t sdaPin = GPIO_NUM_NC;
        gpio_num_t sclPin = GPIO_NUM_NC;
        uint32_t clockSpeed = 400000; // 400kHz by default
        bool pullupEnable = true;
        uint32_t timeoutMs = 1000;

        // I2C master configuration
        bool masterMode = true;
        uint8_t slaveAddr = 0x68; // Default BMI270 I2C address (can be 0x68 or 0x69)
    };

} // namespace bmi270
