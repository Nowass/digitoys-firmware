#include "bmi270.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace bmi270
{

    namespace
    {
        constexpr const char *TAG = "BMI270";

        // BMI270 Power-on timing constants (from Bosch reference implementation)
        // CRITICAL: BMI270 requires proper power-on sequencing to read correct chip ID (0x24)
        constexpr uint32_t POWER_ON_RESET_DELAY_MS = 2;  // Initial power-on reset delay
        constexpr uint32_t SOFT_RESET_DELAY_MS = 2;      // After soft reset command (increased from 1ms)
        constexpr uint32_t POWER_UP_DELAY_MS = 1;        // Power up delay (450us from BMI2_POWER_SAVE_MODE_DELAY_IN_US -> 1ms)
        constexpr uint32_t COMMAND_DELAY_MS = 1;         // After command execution (2us from BMI2_NORMAL_MODE_DELAY_IN_US -> 1ms)
        constexpr uint32_t I2C_TRANSACTION_DELAY_MS = 1; // Between I2C transactions for stability
        constexpr uint32_t CHIP_ID_RETRY_DELAY_MS = 5;   // Delay between chip ID read attempts
        constexpr uint32_t MAX_CHIP_ID_RETRIES = 10;     // Maximum retries for chip ID reading
    }

    esp_err_t BMI270::init(const I2CConfig &config)
    {
        ESP_LOGI(TAG, "Initializing BMI270 sensor with consolidated I2C HAL and automatic priming");

        // Step 1: Initialize I2C communication with automatic priming
        esp_err_t err = i2c_.initWithPriming(config, config.slaveAddr, registers::CHIP_ID, true);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize I2C HAL with priming: %s", esp_err_to_name(err));
            return err;
        }

        // Step 2: Initial power-on reset delay (allow BMI270 to complete boot sequence)
        ESP_LOGI(TAG, "Waiting for BMI270 power-on reset completion...");
        delay(POWER_ON_RESET_DELAY_MS);

        // Step 3: Test basic I2C communication first
        ESP_LOGI(TAG, "Testing I2C communication...");
        err = testCommunication();
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Initial I2C communication test failed, proceeding with reset sequence");
        }

        // Step 4: Perform soft reset to ensure clean state
        ESP_LOGI(TAG, "Performing soft reset...");
        err = softReset();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(err));
            return err;
        }

        // Step 5: Wait for sensor to complete reset and boot sequence
        ESP_LOGI(TAG, "Waiting for BMI270 to complete reset sequence...");
        delay(POWER_UP_DELAY_MS);

        // Step 6: Read and verify chip ID with retry mechanism
        ESP_LOGI(TAG, "Reading chip ID with retry mechanism...");
        err = verifyChipIdWithRetry();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Chip ID verification failed after retries: %s", esp_err_to_name(err));
            return err;
        }

        // Step 7: Final communication test
        err = testCommunication();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Final communication test failed: %s", esp_err_to_name(err));
            return err;
        }

        // Step 8: Check for any initialization errors
        uint8_t errorReg;
        err = readErrorRegister(errorReg);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Could not read error register: %s", esp_err_to_name(err));
        }
        else if (errorReg != 0)
        {
            ESP_LOGW(TAG, "Error register shows: 0x%02X", errorReg);
        }
        else
        {
            ESP_LOGI(TAG, "✓ No errors detected in error register");
        }

        initialized_ = true;
        ESP_LOGI(TAG, "✓ BMI270 initialization completed successfully");
        ESP_LOGI(TAG, "  Chip ID: 0x%02X (expected: 0x%02X)", chipId_, constants::CHIP_ID_VALUE);
        ESP_LOGI(TAG, "  I2C address: 0x%02X", i2c_.getSlaveAddress());

        return ESP_OK;
    }
    esp_err_t BMI270::readChipId(uint8_t &chipId)
    {
        esp_err_t err = i2c_.readByte(registers::CHIP_ID, chipId);
        if (err == ESP_OK)
        {
            chipId_ = chipId; // Cache the chip ID
            ESP_LOGD(TAG, "Read chip ID: 0x%02X", chipId);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read chip ID: %s", esp_err_to_name(err));
        }
        return err;
    }

    esp_err_t BMI270::verifyChipId()
    {
        uint8_t chipId;
        esp_err_t err = readChipId(chipId);
        if (err != ESP_OK)
        {
            return err;
        }

        if (chipId != constants::CHIP_ID_VALUE)
        {
            ESP_LOGE(TAG, "Invalid chip ID: expected 0x%02X, got 0x%02X",
                     constants::CHIP_ID_VALUE, chipId);
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGI(TAG, "✓ BMI270 chip ID verified: 0x%02X", chipId);
        return ESP_OK;
    }

    esp_err_t BMI270::verifyChipIdWithRetry()
    {
        for (uint32_t attempt = 1; attempt <= MAX_CHIP_ID_RETRIES; attempt++)
        {
            uint8_t chipId;
            esp_err_t err = readChipId(chipId);

            if (err == ESP_OK)
            {
                ESP_LOGI(TAG, "Attempt %lu: Chip ID read = 0x%02X", attempt, chipId);

                if (chipId == constants::CHIP_ID_VALUE)
                {
                    ESP_LOGI(TAG, "✓ Chip ID verified successfully on attempt %lu: 0x%02X",
                             attempt, chipId);
                    return ESP_OK;
                }
                else
                {
                    ESP_LOGW(TAG, "Attempt %lu: Wrong chip ID - expected 0x%02X, got 0x%02X",
                             attempt, constants::CHIP_ID_VALUE, chipId);

                    if (chipId == 0x20)
                    {
                        ESP_LOGW(TAG, "Chip ID 0x20 suggests BMI270 is in boot/initialization state");
                        ESP_LOGW(TAG, "Waiting longer for BMI270 to complete boot sequence...");
                        delay(CHIP_ID_RETRY_DELAY_MS * 2); // Longer delay for boot completion
                    }
                    else if (chipId == 0x00 || chipId == 0xFF)
                    {
                        ESP_LOGW(TAG, "Chip ID 0x%02X suggests I2C communication issue", chipId);
                        delay(I2C_TRANSACTION_DELAY_MS);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Unknown chip ID 0x%02X", chipId);
                        delay(CHIP_ID_RETRY_DELAY_MS);
                    }
                }
            }
            else
            {
                ESP_LOGE(TAG, "Attempt %lu: Failed to read chip ID: %s",
                         attempt, esp_err_to_name(err));
                delay(I2C_TRANSACTION_DELAY_MS);
            }
        }

        ESP_LOGE(TAG, "✗ Chip ID verification failed after %lu attempts", MAX_CHIP_ID_RETRIES);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t BMI270::softReset()
    {
        ESP_LOGI(TAG, "Performing soft reset");

        esp_err_t err = i2c_.writeByte(registers::CMD, constants::SOFT_RESET_CMD);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send soft reset command: %s", esp_err_to_name(err));
            return err;
        }

        // Wait for reset to complete
        delay(SOFT_RESET_DELAY_MS);

        ESP_LOGI(TAG, "✓ Soft reset completed");
        return ESP_OK;
    }

    esp_err_t BMI270::readErrorRegister(uint8_t &errorReg)
    {
        esp_err_t err = i2c_.readByte(registers::ERR_REG, errorReg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read error register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGD(TAG, "Error register: 0x%02X", errorReg);
        }
        return err;
    }

    esp_err_t BMI270::readStatus(uint8_t &status)
    {
        esp_err_t err = i2c_.readByte(registers::STATUS, status);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read status register: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGD(TAG, "Status register: 0x%02X", status);
        }
        return err;
    }

    esp_err_t BMI270::testCommunication()
    {
        ESP_LOGD(TAG, "Testing I2C communication");

        // Test basic I2C communication with ping
        esp_err_t err = i2c_.ping();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C ping failed: %s", esp_err_to_name(err));
            return err;
        }

        // Try to read chip ID as a communication test
        uint8_t chipId;
        err = i2c_.readByte(registers::CHIP_ID, chipId);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read chip ID during communication test: %s", esp_err_to_name(err));
            return err;
        }

        ESP_LOGD(TAG, "✓ I2C communication test passed, chip ID: 0x%02X", chipId);
        return ESP_OK;
    }

    esp_err_t BMI270::checkInitialized() const
    {
        if (!initialized_)
        {
            ESP_LOGE(TAG, "BMI270 not initialized. Call init() first.");
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }

    void BMI270::delay(uint32_t delayMs) const
    {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }

} // namespace bmi270
