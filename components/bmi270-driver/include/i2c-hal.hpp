#pragma once

#include "I2CConfig.hpp"
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <span>
#include <cstddef> // std::size_t, std::byte
#include <cstdint>

namespace bmi270
{

    /**
     * @brief I2C Bus Priming Result
     */
    enum class PrimingResult
    {
        SUCCESS,           // Priming completed successfully
        NO_PRIMING_NEEDED, // Bus was already working at low speed
        FAILED             // Priming failed
    };

    /**
     * @brief Enhanced I2C Hardware Abstraction Layer with built-in bus priming
     *
     * This class provides a complete I2C HAL solution that includes:
     * - Standard I2C operations (read, write, ping)
     * - Built-in ESP32C6 I2C bus priming for reliable cold-boot initialization
     * - Production-ready initialization with automatic priming
     */
    class [[nodiscard]] I2C_HAL final
    {
    public:
        I2C_HAL() = default;
        ~I2C_HAL();

        // === Standard I2C Operations ===

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

        // === Enhanced Production-Ready Initialization ===

        /**
         * @brief Initialize I2C with automatic bus priming for ESP32C6 cold-boot reliability
         *
         * This method combines generic I2C bus priming with standard initialization
         * to ensure reliable operation on ESP32C6, especially after cold boot.
         *
         * The priming sequence:
         * 1. Attempt I2C operations at 100kHz (expected to fail, but primes bus)
         * 2. Clean destruction and timing delay
         * 3. Initialize at target speed with reliable communication
         *
         * @param config Target I2C configuration for production use
         * @param deviceAddr I2C device address to use for priming operations (default: config.slaveAddr)
         * @param testRegAddr Register address for dummy read operations (default: 0x00)
         * @param enablePriming Enable automatic priming (default: true)
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t initWithPriming(const I2CConfig &config,
                                  uint8_t deviceAddr = 0xFF, // 0xFF means use config.slaveAddr
                                  uint8_t testRegAddr = 0x00,
                                  bool enablePriming = true);

        /**
         * @brief Perform generic I2C bus priming sequence
         *
         * This static method can be used independently to prime any I2C bus
         * before initializing specific devices.
         *
         * @param config Target I2C configuration
         * @param deviceAddr I2C device address for priming operations
         * @param testRegAddr Register address for dummy operations
         * @return PrimingResult indicating the outcome
         */
        static PrimingResult primeI2CBus(const I2CConfig &config,
                                         uint8_t deviceAddr = 0x68,
                                         uint8_t testRegAddr = 0x00);

    private:
        i2c_port_t port_ = I2C_NUM_0;
        uint8_t slaveAddr_ = 0x68;
        uint32_t timeoutMs_ = 1000;
        bool initialized_ = false;

        /**
         * @brief Perform additional dummy I2C operations for thorough bus priming
         *
         * @param hal I2C HAL instance to use for operations
         * @param testRegAddr Register address for dummy operations
         */
        static void performDummyOperations(I2C_HAL &hal, uint8_t testRegAddr);
    };

} // namespace bmi270
