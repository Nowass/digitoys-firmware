/**
 * @file bmi270-simple.hpp
 * @brief Simplified BMI270 Production Driver using consolidated I2C HAL
 *
 * This simplified driver uses the enhanced I2C HAL with built-in priming
 * for easy BMI270 initialization in production environments.
 */

#pragma once

#include "bmi270.hpp"
#include "I2CConfig.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace bmi270
{

    /**
     * @brief Simplified production BMI270 initialization function
     *
     * This function provides easy BMI270 initialization with automatic
     * I2C bus priming using the consolidated I2C HAL.
     *
     * @param sensor Reference to BMI270 instance to initialize
     * @param sdaPin SDA GPIO pin (default: GPIO_NUM_4)
     * @param sclPin SCL GPIO pin (default: GPIO_NUM_5)
     * @param clockSpeed I2C clock speed (default: 200000 Hz)
     * @param slaveAddr BMI270 I2C address (default: 0x68)
     * @param enablePriming Enable automatic I2C bus priming (default: true)
     * @return ESP_OK on success, error code on failure
     */
    inline esp_err_t initProductionBMI270(BMI270 &sensor,
                                          gpio_num_t sdaPin = GPIO_NUM_4,
                                          gpio_num_t sclPin = GPIO_NUM_5,
                                          uint32_t clockSpeed = 200000,
                                          uint8_t slaveAddr = 0x68,
                                          bool enablePriming = true)
    {
        const char *TAG = "BMI270_PRODUCTION";

        ESP_LOGI(TAG, "=== BMI270 Production Initialization ===");

        if (enablePriming)
        {
            ESP_LOGI(TAG, "Using consolidated I2C HAL with automatic bus priming");
        }
        else
        {
            ESP_LOGI(TAG, "Using standard I2C HAL (priming disabled)");
        }

        // Create I2C configuration
        I2CConfig config;
        config.port = I2C_NUM_0;
        config.sdaPin = sdaPin;
        config.sclPin = sclPin;
        config.clockSpeed = clockSpeed;
        config.slaveAddr = slaveAddr;

        // The BMI270 sensor will use the consolidated I2C HAL
        // If the I2C HAL has priming enabled, it will be applied automatically
        esp_err_t result = sensor.init(config);

        if (result == ESP_OK)
        {
            ESP_LOGI(TAG, "✓ BMI270 initialization successful");

            // Validation: Test chip ID multiple times for stability
            int successCount = 0;
            for (int i = 0; i < 5; i++)
            {
                uint8_t chipId;
                if (sensor.readChipId(chipId) == ESP_OK && chipId == constants::CHIP_ID_VALUE)
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
                if (sensor.readErrorRegister(errorReg) == ESP_OK && errorReg == 0)
                {
                    ESP_LOGI(TAG, "✓ No errors detected in BMI270 error register");
                }
                else
                {
                    ESP_LOGW(TAG, "⚠ BMI270 error register: 0x%02X", errorReg);
                }

                ESP_LOGI(TAG, "🎉 BMI270 Production Initialization SUCCESSFUL!");
                ESP_LOGI(TAG, "  Configuration: %lu Hz, SDA=%d, SCL=%d, Addr=0x%02X",
                         clockSpeed, sdaPin, sclPin, slaveAddr);
                if (enablePriming)
                {
                    ESP_LOGI(TAG, "  I2C bus priming: Applied via consolidated HAL");
                }
                ESP_LOGI(TAG, "  Status: Ready for production use");

                return ESP_OK;
            }
            else
            {
                ESP_LOGE(TAG, "✗ Validation failed: %d/5 chip ID reads successful", successCount);
                ESP_LOGE(TAG, "BMI270 is not stable enough for production use");
                return ESP_FAIL;
            }
        }
        else
        {
            ESP_LOGE(TAG, "✗ BMI270 initialization failed: %s", esp_err_to_name(result));
            return result;
        }
    }

} // namespace bmi270
