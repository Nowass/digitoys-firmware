#pragma once

#include "i2c-hal.hpp"
#include "I2CConfig.hpp"
#include "bmi270-defs.hpp"
#include <esp_err.h>
#include <cstdint>

namespace bmi270
{

    /**
     * @brief BMI270 sensor driver class
     *
     * This class provides a C++ interface to the BMI270 6-axis inertial
     * measurement unit (IMU) sensor from Bosch.
     *
     * Implementation follows a step-by-step approach:
     * Step 1: Basic device initialization and chip ID verification (CURRENT)
     * Step 2: Configuration file upload and advanced initialization
     * Step 3: Accelerometer and gyroscope data reading
     * Step 4: Advanced features (FIFO, interrupts, motion detection)
     */
    class BMI270 final
    {
    public:
        BMI270() = default;
        ~BMI270() = default;

        // Non-copyable, non-movable for safety
        BMI270(const BMI270 &) = delete;
        BMI270 &operator=(const BMI270 &) = delete;
        BMI270(BMI270 &&) = delete;
        BMI270 &operator=(BMI270 &&) = delete;

        /**
         * @brief Initialize the BMI270 sensor
         *
         * Step 1 implementation:
         * - Initialize I2C communication
         * - Verify device presence
         * - Read and verify chip ID
         * - Perform basic initialization
         *
         * @param config I2C configuration parameters
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t init(const I2CConfig &config);

        /**
         * @brief Check if the sensor is initialized
         * @return true if initialized, false otherwise
         */
        bool isInitialized() const { return initialized_; }

        /**
         * @brief Read the chip ID
         * @param chipId Reference to store the chip ID
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t readChipId(uint8_t &chipId);

        /**
         * @brief Verify that the connected device is a BMI270
         * @return ESP_OK if BMI270 is detected, error code otherwise
         */
        esp_err_t verifyChipId();

        /**
         * @brief Verify chip ID with retry mechanism and detailed logging
         * @return ESP_OK if BMI270 is detected after retries, error code otherwise
         */
        esp_err_t verifyChipIdWithRetry();

        /**
         * @brief Perform a soft reset of the sensor
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t softReset();

        /**
         * @brief Read the error register
         * @param errorReg Reference to store the error register value
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t readErrorRegister(uint8_t &errorReg);

        /**
         * @brief Read the status register
         * @param status Reference to store the status register value
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t readStatus(uint8_t &status);

        /**
         * @brief Get the I2C slave address currently being used
         * @return I2C slave address
         */
        uint8_t getSlaveAddress() const { return i2c_.getSlaveAddress(); }

        /**
         * @brief Test I2C communication with the sensor
         * @return ESP_OK if communication works, error code otherwise
         */
        esp_err_t testCommunication();

    private:
        I2C_HAL i2c_;              ///< I2C hardware abstraction layer
        bool initialized_ = false; ///< Initialization status flag
        uint8_t chipId_ = 0;       ///< Cached chip ID value

        // Retry mechanism constants
        static constexpr uint32_t MAX_CHIP_ID_RETRIES = 5;      ///< Maximum chip ID verification attempts
        static constexpr uint32_t CHIP_ID_RETRY_DELAY_MS = 10;  ///< Delay between chip ID retry attempts
        static constexpr uint32_t I2C_TRANSACTION_DELAY_MS = 5; ///< Delay for I2C transaction recovery

        /**
         * @brief Internal helper to check if device is initialized
         * @return ESP_ERR_INVALID_STATE if not initialized, ESP_OK otherwise
         */
        esp_err_t checkInitialized() const;

        /**
         * @brief Internal helper for delay operations
         * @param delayMs Delay time in milliseconds
         */
        void delay(uint32_t delayMs) const;
    };

} // namespace bmi270
