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

        /**
         * @brief Check internal status register for initialization readiness
         * @return ESP_OK if internal status is ready, error code otherwise
         */
        esp_err_t checkInternalStatus();

        /**
         * @brief Initialize power management settings
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t initPowerManagement();

        /**
         * @brief Initialize basic sensor configuration
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t initBasicConfiguration();

        /**
         * @brief Perform final system validation
         * @return ESP_OK if validation passes, error code otherwise
         */
        esp_err_t performFinalValidation();

        /**
         * @brief Perform BMI2 sensor initialization (equivalent to bmi2_sec_init)
         *
         * This method follows the exact sequence from the Bosch reference implementation:
         * 1. Set APS flag as after reset, the sensor is on advance power save mode
         * 2. Read chip-id of the BMI2 sensor
         * 3. Validate chip-id
         * 4. Assign resolution to the structure
         * 5. Set manual enable flag
         * 6. Set default values for axis re-mapping
         * 7. Perform soft-reset to bring all register values to their default values
         * 8. Write configuration file
         *
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t performBMI2Init();

        /**
         * @brief Write BMI270 configuration file to device
         *
         * This method implements the configuration file upload sequence:
         * 1. Prepare device for configuration upload
         * 2. Upload configuration data in chunks
         * 3. Verify configuration load status
         *
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t writeConfigFile();

        /**
         * @brief Perform soft reset with configuration file upload
         *
         * This method implements the complete soft reset sequence:
         * 1. Send soft reset command
         * 2. Wait for reset completion
         * 3. Write configuration file
         * 4. Verify configuration load
         *
         * @return ESP_OK on success, error code otherwise
         */
        esp_err_t performSoftReset();
    };

} // namespace bmi270
