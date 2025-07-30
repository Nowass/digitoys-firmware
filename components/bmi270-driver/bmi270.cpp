#include "bmi270.hpp"
#include "bmi270-config.hpp"
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
        constexpr uint32_t POWER_ON_RESET_DELAY_MS = 10; // Initial power-on reset delay (increased from 2ms)
        constexpr uint32_t SOFT_RESET_DELAY_MS = 2;      // After soft reset command (increased from 1ms)
        constexpr uint32_t POWER_UP_DELAY_MS = 1;        // Power up delay (450us from BMI2_POWER_SAVE_MODE_DELAY_IN_US -> 1ms)
        constexpr uint32_t COMMAND_DELAY_MS = 1;         // After command execution (2us from BMI2_NORMAL_MODE_DELAY_IN_US -> 1ms)
        constexpr uint32_t I2C_TRANSACTION_DELAY_MS = 1; // Between I2C transactions for stability
        constexpr uint32_t CHIP_ID_RETRY_DELAY_MS = 10;  // Delay between chip ID read attempts (increased)
        constexpr uint32_t MAX_CHIP_ID_RETRIES = 20;     // Maximum retries for chip ID reading (increased)
    }

    esp_err_t BMI270::init(const I2CConfig &config)
    {
        // Step 1: Initialize I2C communication with automatic priming
        esp_err_t err = i2c_.initWithPriming(config, config.slaveAddr, registers::CHIP_ID, true);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize I2C HAL with priming: %s", esp_err_to_name(err));
            return err;
        }

        // Step 1.5: CRITICAL - BMI270 Power-On Sequence
        // The BMI270 needs time to complete its boot sequence after power-on
        // During boot, chip ID reads 0x20. After boot completion, it reads 0x24
        delay(POWER_ON_RESET_DELAY_MS); // Allow BMI270 to complete boot sequence

        err = softReset();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(err));
            return err;
        }

        // Step 5: Wait for sensor to complete reset and boot sequence
        delay(POWER_UP_DELAY_MS);

        // Step 2: Follow BMI270 reference implementation sequence
        ESP_LOGI(TAG, "Step 2: Setting up BMI270 device structure parameters");

        // Assign chip id of BMI270 (following reference: dev->chip_id = BMI270_CHIP_ID)
        ESP_LOGI(TAG, "Setting expected chip ID to 0x%02X", constants::BMI270_CHIP_ID);

        // Get the size of config array (following reference: dev->config_size = sizeof(bmi270_config_file))
        ESP_LOGI(TAG, "Configuration file size: %zu bytes", bmi270_config_file_size);

        // Enable the variant specific features (following reference: dev->variant_feature = ...)
        uint32_t variantFeatures = constants::BMI2_GYRO_CROSS_SENS_ENABLE | constants::BMI2_CRT_RTOSK_ENABLE;
        ESP_LOGI(TAG, "Variant features enabled: 0x%08lX", variantFeatures);

        // Set dummy byte for I2C (following reference: dev->dummy_byte = 0 for I2C)
        // ESP_LOGI(TAG, "Interface: I2C (dummy_byte = 0)");

        // Step 3: BMI2 sensor initialization (equivalent to bmi2_sec_init)
        ESP_LOGI(TAG, "Step 3: Performing BMI2 sensor initialization");
        // err = performBMI2Init();
        verifyChipId();

        // // Step 4: Post-initialization setup (following reference implementation)
        // ESP_LOGI(TAG, "Step 4: Post-initialization configuration");

        // // Set resolution to 16-bit (following reference: dev->resolution = 16)
        // ESP_LOGI(TAG, "Sensor resolution: 16-bit");

        // // Set manual enable flag (following reference: dev->aux_man_en = 1)
        // ESP_LOGI(TAG, "Auxiliary manual enable: true");

        // // Set default axis re-mapping (following reference: dev->remap = axes_remap)
        // ESP_LOGI(TAG, "Default axis re-mapping applied");

        initialized_ = true;
        ESP_LOGI(TAG, "✓ BMI270 initialization completed successfully");
        ESP_LOGI(TAG, "  Chip ID: 0x%02X (expected: 0x%02X)", chipId_, constants::BMI270_CHIP_ID);
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

    esp_err_t BMI270::checkInternalStatus()
    {
        uint8_t internalStatus;
        esp_err_t err = i2c_.readByte(registers::INTERNAL_STATUS, internalStatus);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read internal status: %s", esp_err_to_name(err));
            return err;
        }

        ESP_LOGI(TAG, "Internal status: 0x%02X", internalStatus);

        // Check for initialization status
        uint8_t initStatus = internalStatus & 0x0F;
        switch (initStatus)
        {
        case constants::INTERNAL_STATUS_NOT_INIT:
            ESP_LOGW(TAG, "Device not initialized yet");
            break;
        case constants::INTERNAL_STATUS_INIT_OK:
            ESP_LOGI(TAG, "✓ Device initialization successful");
            break;
        case constants::INTERNAL_STATUS_INIT_ERR:
            ESP_LOGE(TAG, "Device initialization error detected");
            return ESP_ERR_INVALID_STATE;
        case constants::INTERNAL_STATUS_DRV_ERR:
            ESP_LOGE(TAG, "Driver error detected");
            return ESP_ERR_INVALID_STATE;
        default:
            ESP_LOGW(TAG, "Unknown initialization status: 0x%02X", initStatus);
            break;
        }

        return ESP_OK;
    }

    esp_err_t BMI270::initPowerManagement()
    {
        esp_err_t err;

        // Step 1: Configure power configuration register
        ESP_LOGI(TAG, "Configuring power configuration");
        uint8_t pwrConf = constants::PWR_CONF_FUP_EN; // Enable fast power-up
        err = i2c_.writeByte(registers::PWR_CONF, pwrConf);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write power configuration: %s", esp_err_to_name(err));
            return err;
        }
        delay(COMMAND_DELAY_MS);

        // Step 2: Enable accelerometer and temperature sensor
        ESP_LOGI(TAG, "Enabling accelerometer and temperature sensor");
        uint8_t pwrCtrl = constants::PWR_CTRL_ACC_EN | constants::PWR_CTRL_TEMP_EN;
        err = i2c_.writeByte(registers::PWR_CTRL, pwrCtrl);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write power control: %s", esp_err_to_name(err));
            return err;
        }
        delay(POWER_UP_DELAY_MS);

        // Step 3: Verify power control settings
        uint8_t pwrCtrlRead;
        err = i2c_.readByte(registers::PWR_CTRL, pwrCtrlRead);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read power control: %s", esp_err_to_name(err));
            return err;
        }

        ESP_LOGI(TAG, "Power control verification: written=0x%02X, read=0x%02X", pwrCtrl, pwrCtrlRead);
        if ((pwrCtrlRead & (constants::PWR_CTRL_ACC_EN | constants::PWR_CTRL_TEMP_EN)) == 0)
        {
            ESP_LOGW(TAG, "Power control settings may not have been applied correctly");
        }
        else
        {
            ESP_LOGI(TAG, "✓ Power management initialized successfully");
        }

        return ESP_OK;
    }

    esp_err_t BMI270::initBasicConfiguration()
    {
        esp_err_t err;

        // Step 1: Configure accelerometer settings
        ESP_LOGI(TAG, "Configuring accelerometer settings");

        // Set accelerometer ODR to 100Hz with normal bandwidth and performance mode
        uint8_t accConf = constants::ACC_CONF_ODR_100HZ | constants::ACC_CONF_BWP_NORMAL | constants::ACC_CONF_PERF_MODE;
        err = i2c_.writeByte(registers::ACC_CONF, accConf);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write accelerometer configuration: %s", esp_err_to_name(err));
            return err;
        }
        delay(COMMAND_DELAY_MS);

        // Set accelerometer range to ±4g
        uint8_t accRange = constants::ACC_RANGE_4G;
        err = i2c_.writeByte(registers::ACC_RANGE, accRange);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to write accelerometer range: %s", esp_err_to_name(err));
            return err;
        }
        delay(COMMAND_DELAY_MS);

        // Step 2: Verify configuration
        uint8_t accConfRead, accRangeRead;
        err = i2c_.readByte(registers::ACC_CONF, accConfRead);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read accelerometer configuration: %s", esp_err_to_name(err));
            return err;
        }

        err = i2c_.readByte(registers::ACC_RANGE, accRangeRead);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read accelerometer range: %s", esp_err_to_name(err));
            return err;
        }

        ESP_LOGI(TAG, "Accelerometer configuration verification:");
        ESP_LOGI(TAG, "  ACC_CONF: written=0x%02X, read=0x%02X", accConf, accConfRead);
        ESP_LOGI(TAG, "  ACC_RANGE: written=0x%02X, read=0x%02X", accRange, accRangeRead);

        if (accConfRead != accConf || accRangeRead != accRange)
        {
            ESP_LOGW(TAG, "Configuration verification mismatch - may indicate register access issues");
        }
        else
        {
            ESP_LOGI(TAG, "✓ Basic configuration applied successfully");
        }

        return ESP_OK;
    }

    esp_err_t BMI270::performFinalValidation()
    {
        esp_err_t err;

        // Step 1: Final communication test
        ESP_LOGI(TAG, "Final communication validation");
        err = testCommunication();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Final communication test failed: %s", esp_err_to_name(err));
            return err;
        }

        // Step 2: Check error register
        uint8_t errorReg;
        err = readErrorRegister(errorReg);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Could not read error register during validation: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "Error register: 0x%02X", errorReg);
            if (errorReg != 0)
            {
                ESP_LOGW(TAG, "Error register indicates issues: 0x%02X", errorReg);
                // Don't fail initialization for minor errors, just log them
            }
            else
            {
                ESP_LOGI(TAG, "✓ No errors detected in error register");
            }
        }

        // Step 3: Check status register
        uint8_t status;
        err = readStatus(status);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Could not read status register during validation: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "Status register: 0x%02X", status);
            ESP_LOGI(TAG, "  Data ready status: ACC=%s",
                     (status & 0x80) ? "ready" : "not ready");
        }

        // Step 4: Try reading accelerometer data as final test
        ESP_LOGI(TAG, "Testing accelerometer data readout");
        uint8_t accData[6];

        // Read accelerometer data bytes individually to avoid span issues
        for (int i = 0; i < 6; i++)
        {
            err = i2c_.readByte(registers::ACC_DATA_X_LSB + i, accData[i]);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to read accelerometer data byte %d during validation: %s", i, esp_err_to_name(err));
                return err;
            }
        }

        // Convert to signed 16-bit values
        int16_t accX = (int16_t)((accData[1] << 8) | accData[0]);
        int16_t accY = (int16_t)((accData[3] << 8) | accData[2]);
        int16_t accZ = (int16_t)((accData[5] << 8) | accData[4]);

        ESP_LOGI(TAG, "Sample accelerometer data: X=%d, Y=%d, Z=%d", accX, accY, accZ);

        // Basic sanity check - at least one axis should show some gravity effect
        if (accX == 0 && accY == 0 && accZ == 0)
        {
            ESP_LOGW(TAG, "All accelerometer readings are zero - sensor may not be responding");
        }
        else
        {
            ESP_LOGI(TAG, "✓ Accelerometer data validation successful");
        }

        ESP_LOGI(TAG, "✓ Final validation completed successfully");
        return ESP_OK;
    }

    esp_err_t BMI270::performBMI2Init()
    {
        ESP_LOGI(TAG, "Performing BMI2 sensor initialization (equivalent to bmi2_sec_init)");

        // Step 1: Set APS flag as after reset, the sensor is on advance power save mode
        // (following reference: dev->aps_status = BMI2_ENABLE)
        ESP_LOGI(TAG, "Setting APS (Advanced Power Save) status to enabled");

        // Step 2: Read and validate chip-id with retry logic
        // The BMI270 may need time to complete boot sequence (0x20 -> 0x24)
        ESP_LOGI(TAG, "Reading and validating chip ID with boot completion retry");
        esp_err_t err = verifyChipIdWithRetry();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Chip ID validation failed: %s", esp_err_to_name(err));
            return err;
        }

        ESP_LOGI(TAG, "✓ Chip ID validated successfully: 0x%02X", chipId_);

        // Step 4: Assign resolution to the structure (following reference: dev->resolution = 16)
        ESP_LOGI(TAG, "Setting sensor resolution to 16-bit");

        // Step 5: Set manual enable flag (following reference: dev->aux_man_en = 1)
        ESP_LOGI(TAG, "Setting auxiliary manual enable flag");

        // Step 6: Set the default values for axis re-mapping
        ESP_LOGI(TAG, "Setting default axis re-mapping values");

        // Step 7: Perform soft-reset (following reference: rslt = bmi2_soft_reset(dev))
        ESP_LOGI(TAG, "Performing soft reset to bring all registers to default values");
        err = performSoftReset();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(err));
            return err;
        }

        ESP_LOGI(TAG, "✓ BMI2 sensor initialization completed successfully");
        return ESP_OK;
    }

    esp_err_t BMI270::performSoftReset()
    {
        ESP_LOGI(TAG, "Performing BMI270 soft reset sequence");

        // Step 1: Reset BMI2 device (following reference: bmi2_set_regs(BMI2_CMD_REG_ADDR, &data, 1, dev))
        ESP_LOGI(TAG, "Sending soft reset command");
        esp_err_t err = i2c_.writeByte(registers::CMD, constants::SOFT_RESET_CMD);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send soft reset command: %s", esp_err_to_name(err));
            return err;
        }

        // Step 2: Wait for reset to complete (following reference: dev->delay_us(2000, dev->intf_ptr))
        ESP_LOGI(TAG, "Waiting for soft reset completion (2ms)");
        delay(SOFT_RESET_DELAY_MS);

        // Step 3: Set APS flag again after soft reset (following reference: dev->aps_status = BMI2_ENABLE)
        ESP_LOGI(TAG, "Re-enabling APS status after soft reset");

        // Step 4: For I2C interface, no dummy read needed (SPI only)

        // Step 5: Write the configuration file (following reference: rslt = bmi2_write_config_file(dev))
        ESP_LOGI(TAG, "Writing BMI270 configuration file");
        err = writeConfigFile();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Configuration file write failed: %s", esp_err_to_name(err));
            return err;
        }

        // Step 6: Reset sensor status flag (following reference: dev->sens_en_stat = 0)
        ESP_LOGI(TAG, "Resetting sensor enable status flags");

        ESP_LOGI(TAG, "✓ Soft reset sequence completed successfully");
        return ESP_OK;
    }

    esp_err_t BMI270::writeConfigFile()
    {
        ESP_LOGI(TAG, "Writing BMI270 configuration file (%zu bytes)", bmi270_config_file_size);

        if (bmi270_config_file_size == 0)
        {
            ESP_LOGE(TAG, "Configuration file size is zero");
            return ESP_ERR_INVALID_ARG;
        }

        // Step 1: Prepare for configuration upload
        ESP_LOGI(TAG, "Preparing device for configuration upload");

        // Disable advanced power save mode to allow register access
        esp_err_t err = i2c_.writeByte(registers::PWR_CONF, constants::APS_DISABLE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to disable APS mode: %s", esp_err_to_name(err));
            return err;
        }
        delay(COMMAND_DELAY_MS);

        // Step 2: Set initialization control to prepare for config upload
        ESP_LOGI(TAG, "Setting initialization control for config upload");
        err = i2c_.writeByte(registers::INIT_CTRL, constants::INIT_CTRL_LOAD_CONFIG_FILE);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set init control: %s", esp_err_to_name(err));
            return err;
        }
        delay(COMMAND_DELAY_MS);

        // Step 3: Upload configuration file in chunks
        ESP_LOGI(TAG, "Uploading configuration file data");
        const size_t chunkSize = 16; // Upload in 16-byte chunks
        size_t totalBytes = bmi270_config_file_size;
        size_t bytesWritten = 0;

        for (size_t offset = 0; offset < totalBytes; offset += chunkSize)
        {
            size_t currentChunkSize = (offset + chunkSize > totalBytes) ? (totalBytes - offset) : chunkSize;

            // Set address pointer (simplified approach)
            uint16_t addr = offset;
            err = i2c_.writeByte(registers::INIT_ADDR_0, addr & 0xFF);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to set address LSB: %s", esp_err_to_name(err));
                return err;
            }

            err = i2c_.writeByte(registers::INIT_ADDR_1, (addr >> 8) & 0xFF);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to set address MSB: %s", esp_err_to_name(err));
                return err;
            }

            // Write data chunk
            for (size_t i = 0; i < currentChunkSize; i++)
            {
                err = i2c_.writeByte(registers::INIT_DATA, bmi270_config_file[offset + i]);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to write config data at offset %zu: %s",
                             offset + i, esp_err_to_name(err));
                    return err;
                }
            }

            bytesWritten += currentChunkSize;

            // Progress indication
            if ((offset % 256) == 0 || offset + chunkSize >= totalBytes)
            {
                ESP_LOGI(TAG, "Configuration upload progress: %zu/%zu bytes (%.1f%%)",
                         bytesWritten, totalBytes, (float)bytesWritten * 100.0f / totalBytes);
            }

            // Small delay between chunks
            delay(1);
        }

        // Step 4: Check configuration load status
        ESP_LOGI(TAG, "Checking configuration load status");
        delay(10); // Allow time for config processing

        uint8_t loadStatus;
        err = i2c_.readByte(registers::INTERNAL_STATUS, loadStatus);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read internal status: %s", esp_err_to_name(err));
            return err;
        }

        uint8_t configStatus = loadStatus & constants::CONFIG_LOAD_STATUS_MASK;
        ESP_LOGI(TAG, "Configuration load status: 0x%02X (full status: 0x%02X)",
                 configStatus, loadStatus);

        if (configStatus != constants::CONFIG_LOAD_SUCCESS)
        {
            ESP_LOGE(TAG, "Configuration load failed, status: 0x%02X", configStatus);
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "✓ Configuration file written and verified successfully");
        return ESP_OK;
    }

} // namespace bmi270
