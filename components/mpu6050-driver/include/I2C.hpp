#pragma once
#include <driver/i2c.h>
#include <esp_err.h>

/// Simple I2C master wrapper for MPU6050
class I2C
{
public:
    struct Config
    {
        i2c_port_t port = I2C_NUM_0;
        gpio_num_t sda_pin;
        gpio_num_t scl_pin;
        uint32_t frequency_hz = 400000; ///< Bus clock
    };

    explicit I2C(const Config &cfg);

    /// Initialize the I2C driver
    esp_err_t init();

    /// Write bytes to a register
    esp_err_t write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);

    /// Read bytes starting from a register
    esp_err_t read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

    /// Write a single byte to a register
    esp_err_t writeByte(uint8_t addr, uint8_t reg, uint8_t value);

    /// Read a single byte from a register
    esp_err_t readByte(uint8_t addr, uint8_t reg, uint8_t *value);

private:
    Config cfg_;
    bool initialized_ = false;
};
