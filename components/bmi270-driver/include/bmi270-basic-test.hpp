/**
 * @file bmi270_basic_test.hpp
 * @brief Basic test functions for BMI270 Step 1 implementation
 *
 * This file contains simple test functions to verify BMI270 basic
 * initialization and chip ID reading functionality.
 */

#pragma once

#include "bmi270.hpp"
#include "I2CConfig.hpp"
#include <esp_log.h>

namespace bmi270
{

    /**
     * @brief Basic test function for BMI270 Step 1 functionality
     *
     * This test function:
     * 1. Initializes BMI270 with I2C on GPIO4/GPIO5
     * 2. Verifies chip ID
     * 3. Tests basic register reading
     * 4. Reports results
     *
     * @return ESP_OK if all tests pass, error code otherwise
     */
    inline esp_err_t runBasicTest()
    {
        const char *TAG = "BMI270_TEST";

        ESP_LOGI(TAG, "=== BMI270 Basic Test (Step 1) ===");

        // Configure I2C for BMI270 (using GPIO6/GPIO7 to avoid conflict with LiDAR motor on GPIO4)
        I2CConfig config;
        config.port = I2C_NUM_0;
        config.sdaPin = GPIO_NUM_4; // Changed from GPIO4 to avoid LiDAR motor conflict
        config.sclPin = GPIO_NUM_5; // Changed from GPIO5
        config.clockSpeed = 400000; // 400kHz
        config.slaveAddr = 0x68;    // BMI270 default address

        // Initialize BMI270 sensor
        BMI270 sensor;
        esp_err_t err = sensor.init(config);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "✗ BMI270 initialization failed: %s", esp_err_to_name(err));

            // Try alternative address
            ESP_LOGI(TAG, "Trying alternative I2C address 0x69...");
            config.slaveAddr = 0x69;
            err = sensor.init(config);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "✗ BMI270 initialization failed with both addresses");
                return err;
            }
        }

        ESP_LOGI(TAG, "✓ BMI270 initialization successful");
        ESP_LOGI(TAG, "  Using I2C address: 0x%02X", sensor.getSlaveAddress());

        // Test chip ID reading
        uint8_t chipId;
        err = sensor.readChipId(chipId);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "✓ Chip ID read successful: 0x%02X", chipId);
            if (chipId == constants::CHIP_ID_VALUE)
            {
                ESP_LOGI(TAG, "✓ Chip ID matches BMI270 expected value (0x%02X)", constants::CHIP_ID_VALUE);
            }
            else
            {
                ESP_LOGW(TAG, "⚠ Unexpected chip ID: expected 0x%02X, got 0x%02X", constants::CHIP_ID_VALUE, chipId);
            }
        }
        else
        {
            ESP_LOGE(TAG, "✗ Failed to read chip ID: %s", esp_err_to_name(err));
        }

        // Test status register reading
        uint8_t status;
        err = sensor.readStatus(status);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "✓ Status register read successful: 0x%02X", status);
        }
        else
        {
            ESP_LOGW(TAG, "⚠ Failed to read status register: %s", esp_err_to_name(err));
        }

        // Test error register reading
        uint8_t errorReg;
        err = sensor.readErrorRegister(errorReg);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "✓ Error register read successful: 0x%02X", errorReg);
            if (errorReg == 0)
            {
                ESP_LOGI(TAG, "✓ No errors detected in error register");
            }
            else
            {
                ESP_LOGW(TAG, "⚠ Error register shows errors: 0x%02X", errorReg);
            }
        }
        else
        {
            ESP_LOGW(TAG, "⚠ Failed to read error register: %s", esp_err_to_name(err));
        }

        ESP_LOGI(TAG, "=== BMI270 Basic Test Completed ===");
        return ESP_OK;
    }

} // namespace bmi270
