/**
 * @file bmi270-production.hpp
 * @brief Production-ready BMI270 driver with built-in priming sequence
 *
 * This driver incorporates the discovered I2C bus priming sequence
 * to ensure reliable initialization on ESP32C6 cold boot.
 */

#pragma once

#include "bmi270.hpp"
#include "I2CConfig.hpp"
#include "i2c-bus-priming.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace bmi270
{

    /**
     * @brief Production BMI270 driver with automatic I2C bus priming
     *
     * This class wraps the standard BMI270 driver with the discovered
     * priming sequence to ensure reliable cold-boot initialization.
     */
    class BMI270Production
    {
    public:
        /**
         * @brief Initialize BMI270 with automatic priming sequence
         *
         * This method implements the discovered priming sequence:
         * 1. Attempt initialization at 100kHz (expected to fail, primes bus)
         * 2. Clean object destruction and 100ms delay
         * 3. Initialize at target speed (200kHz recommended)
         * 4. Validate successful operation
         *
         * @param config I2C configuration for production use
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t initWithPriming(const I2CConfig &config)
        {
            const char *TAG = "BMI270_PRODUCTION";

            ESP_LOGI(TAG, "=== BMI270 Production Initialization ===");
            ESP_LOGI(TAG, "Using generic I2C bus priming for reliable cold-boot");

            // Store the target configuration
            targetConfig_ = config;

            // STEP 1: Generic I2C bus priming (sensor-independent)
            ESP_LOGI(TAG, "Step 1: Generic I2C bus priming...");

            PrimingResult primingResult = I2CBusPriming::primeI2CBus(config, config.slaveAddr, 0x00);

            if (primingResult == PrimingResult::NO_PRIMING_NEEDED)
            {
                ESP_LOGI(TAG, "I2C bus was already working - adjusting to 100kHz for maximum reliability");
                targetConfig_.clockSpeed = 100000; // Use the slower, more reliable speed
            }
            else if (primingResult == PrimingResult::FAILED)
            {
                ESP_LOGE(TAG, "✗ Generic I2C bus priming failed");
                return ESP_FAIL;
            }
            else
            {
                ESP_LOGI(TAG, "✓ Generic I2C bus priming completed successfully");
            }

            // STEP 2: BMI270 initialization at target speed
            ESP_LOGI(TAG, "Step 2: BMI270 initialization at %lu Hz...", targetConfig_.clockSpeed);

            esp_err_t result = sensor_.init(targetConfig_);
            if (result != ESP_OK)
            {
                ESP_LOGE(TAG, "✗ BMI270 initialization failed: %s", esp_err_to_name(result));
                return result;
            }

            // STEP 3: Validation of successful initialization
            ESP_LOGI(TAG, "Step 3: Validating stable BMI270 operation...");

            // Test chip ID multiple times for stability
            int successCount = 0;
            for (int i = 0; i < 5; i++)
            {
                uint8_t chipId;
                if (sensor_.readChipId(chipId) == ESP_OK && chipId == constants::CHIP_ID_VALUE)
                {
                    successCount++;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (successCount >= 4) // Allow 1 failure out of 5
            {
                ESP_LOGI(TAG, "✓ Validation passed: %d/5 successful chip ID reads", successCount);

                // Test error register
                uint8_t errorReg;
                if (sensor_.readErrorRegister(errorReg) == ESP_OK && errorReg == 0)
                {
                    ESP_LOGI(TAG, "✓ No errors detected in BMI270 error register");
                }
                else
                {
                    ESP_LOGW(TAG, "⚠ BMI270 error register: 0x%02X", errorReg);
                }

                initialized_ = true;

                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "🎉 BMI270 Production Initialization SUCCESSFUL!");
                ESP_LOGI(TAG, "  Configuration: %lu Hz, SDA=%d, SCL=%d, Addr=0x%02X",
                         targetConfig_.clockSpeed, targetConfig_.sdaPin,
                         targetConfig_.sclPin, targetConfig_.slaveAddr);
                ESP_LOGI(TAG, "  Generic I2C priming: Applied successfully");
                ESP_LOGI(TAG, "  Status: Ready for production use");
                ESP_LOGI(TAG, "");

                return ESP_OK;
            }
            else
            {
                ESP_LOGE(TAG, "✗ Validation failed: %d/5 chip ID reads successful", successCount);
                ESP_LOGE(TAG, "BMI270 is not stable enough for production use");
                return ESP_FAIL;
            }
        }

        /**
         * @brief Check if BMI270 is initialized and ready
         * @return true if initialized, false otherwise
         */
        bool isInitialized() const { return initialized_; }

        /**
         * @brief Get reference to the underlying BMI270 sensor
         * @return Reference to BMI270 sensor (only valid if initialized)
         */
        BMI270 &getSensor()
        {
            return sensor_;
        }

        /**
         * @brief Get const reference to the underlying BMI270 sensor
         * @return Const reference to BMI270 sensor (only valid if initialized)
         */
        const BMI270 &getSensor() const
        {
            return sensor_;
        }

        /**
         * @brief Get the configuration used for production initialization
         * @return I2C configuration
         */
        const I2CConfig &getConfig() const { return targetConfig_; }

    private:
        BMI270 sensor_;            ///< The actual BMI270 sensor instance
        I2CConfig targetConfig_;   ///< Target configuration for production
        bool initialized_ = false; ///< Initialization status
    };

    /**
     * @brief Quick production initialization function
     *
     * Convenience function for easy BMI270 production initialization
     * with recommended settings.
     *
     * @param sensor Reference to BMI270Production instance to initialize
     * @param sdaPin SDA GPIO pin (default: GPIO_NUM_4)
     * @param sclPin SCL GPIO pin (default: GPIO_NUM_5)
     * @param clockSpeed I2C clock speed (default: 200000 Hz)
     * @param slaveAddr BMI270 I2C address (default: 0x68)
     * @return ESP_OK on success, error code on failure
     */
    inline esp_err_t initProductionBMI270(BMI270Production &sensor,
                                          gpio_num_t sdaPin = GPIO_NUM_4,
                                          gpio_num_t sclPin = GPIO_NUM_5,
                                          uint32_t clockSpeed = 200000,
                                          uint8_t slaveAddr = 0x68)
    {
        I2CConfig config;
        config.port = I2C_NUM_0;
        config.sdaPin = sdaPin;
        config.sclPin = sclPin;
        config.clockSpeed = clockSpeed;
        config.slaveAddr = slaveAddr;

        return sensor.initWithPriming(config);
    }

} // namespace bmi270
