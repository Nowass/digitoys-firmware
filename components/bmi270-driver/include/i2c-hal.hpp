#pragma once

#include "I2CConfig.hpp"
#include <driver/i2c.h>
#include <esp_err.h>
#include <span>
#include <cstddef> // std::size_t, std::byte
#include <cstdint>

namespace bmi270
{

    class [[nodiscard]] I2C_HAL final
    {
    public:
        I2C_HAL() = default;
        ~I2C_HAL();

        // Initialize I2C bus with given configuration
        esp_err_t init(const I2CConfig &cfg);

        // Write data to I2C device at specified register address
        esp_err_t write(uint8_t regAddr, std::span<const std::byte> data) const;

        // Write single byte to I2C device at specified register address
        esp_err_t writeByte(uint8_t regAddr, uint8_t data) const;

        // Read data from I2C device starting at specified register address
        esp_err_t read(uint8_t regAddr, std::span<std::byte> buffer) const;

        // Read single byte from I2C device at specified register address
        esp_err_t readByte(uint8_t regAddr, uint8_t &data) const;

        // Check if device is present on the bus (ping)
        esp_err_t ping() const;

        // Deinitialize I2C bus
        void deinit();

        // Get current slave address
        uint8_t getSlaveAddress() const { return slaveAddr_; }

        // Set new slave address (useful if device has configurable address)
        void setSlaveAddress(uint8_t addr) { slaveAddr_ = addr; }

    private:
        i2c_port_t port_ = I2C_NUM_0;
        uint8_t slaveAddr_ = 0x68;
        uint32_t timeoutMs_ = 1000;
        bool initialized_ = false;
    };

} // namespace bmi270
