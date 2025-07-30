/**
 * @file i2c-bus-priming.hpp
 * @brief Generic I2C Bus Priming Utility
 *
 * This utility provides a generic I2C bus priming mechanism that is
 * independent of any specific sensor. It performs dummy I2C operations
 * to initialize the I2C bus hardware and ensure reliable communication.
 *
 * The priming sequence discovered for ESP32C6 cold-boot:
 * 1. Attempt I2C operations at 100kHz (expected to fail, primes bus)
 * 2. Clean object destruction and delay
 * 3. Initialize at target speed
 */

#pragma once

#include "i2c-hal.hpp"
#include "I2CConfig.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace bmi270
{

    /**
     * @brief Generic I2C Bus Priming Result
     */
    enum class PrimingResult
    {
        SUCCESS,           // Priming completed successfully
        NO_PRIMING_NEEDED, // Bus was already working at low speed
        FAILED             // Priming failed
    };

    /**
     * @brief Generic I2C Bus Priming Utility
     *
     * This class provides sensor-independent I2C bus priming for ESP32C6.
     * It uses generic I2C operations (ping, dummy reads) to prime the bus
     * without relying on specific sensor knowledge.
     */
    class I2CBusPriming
    {
    public:
        /**
         * @brief Perform generic I2C bus priming sequence
         *
         * This method implements the discovered priming sequence:
         * 1. Attempt I2C operations at 100kHz (expected to fail, but primes the bus)
         * 2. Clean destruction and timing delay
         * 3. Bus is now ready for reliable operation at target speed
         *
         * @param config Target I2C configuration for production use
         * @param deviceAddr I2C device address to use for priming operations
         * @param testRegAddr Register address to use for dummy read operations (default: 0x00)
         * @return PrimingResult indicating the outcome
         */
        static PrimingResult primeI2CBus(const I2CConfig &config,
                                         uint8_t deviceAddr = 0x68,
                                         uint8_t testRegAddr = 0x00)
        {
            const char *TAG = "I2C_BUS_PRIMING";

            ESP_LOGI(TAG, "=== Generic I2C Bus Priming ===");
            ESP_LOGI(TAG, "Target: %lu Hz, Device: 0x%02X, SDA=%d, SCL=%d",
                     config.clockSpeed, deviceAddr, config.sdaPin, config.sclPin);

            // STEP 1: Priming sequence at 100kHz (expected to fail)
            ESP_LOGI(TAG, "Step 1: I2C bus priming at 100kHz...");

            {
                // Create priming configuration at 100kHz
                I2CConfig primingConfig = config;
                primingConfig.clockSpeed = 100000; // Force 100kHz for priming
                primingConfig.slaveAddr = deviceAddr;

                // Attempt priming operations (in its own scope for clean destruction)
                I2C_HAL primingHal;
                esp_err_t initResult = primingHal.init(primingConfig);

                if (initResult == ESP_OK)
                {
                    ESP_LOGI(TAG, "✓ I2C HAL initialized at 100kHz");

                    // Perform multiple dummy operations to prime the bus
                    esp_err_t pingResult = primingHal.ping();

                    if (pingResult == ESP_OK)
                    {
                        ESP_LOGI(TAG, "⚠ Unexpected: Device responds at 100kHz immediately");

                        // Try a dummy read operation to further test
                        uint8_t dummyData;
                        esp_err_t readResult = primingHal.readByte(testRegAddr, dummyData);

                        if (readResult == ESP_OK)
                        {
                            ESP_LOGI(TAG, "✓ 100kHz is fully functional - no priming needed");
                            ESP_LOGI(TAG, "Dummy read from reg 0x%02X: 0x%02X", testRegAddr, dummyData);
                            return PrimingResult::NO_PRIMING_NEEDED;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "✓ Ping succeeded but read failed - treating as priming");
                        }
                    }
                    else
                    {
                        ESP_LOGI(TAG, "✓ Expected: 100kHz ping failed (%s) - bus is now primed",
                                 esp_err_to_name(pingResult));
                    }

                    // Try a few more dummy operations for thorough priming
                    performDummyOperations(primingHal, testRegAddr);
                }
                else
                {
                    ESP_LOGI(TAG, "✓ Expected: 100kHz init failed (%s) - bus priming attempted",
                             esp_err_to_name(initResult));
                }

                // HAL destructor called here automatically - this is critical for the priming
            }

            // STEP 2: Critical cleanup delay (discovered timing requirement)
            ESP_LOGI(TAG, "Step 2: 100ms cleanup delay for complete I2C reset...");
            vTaskDelay(pdMS_TO_TICKS(100));

            ESP_LOGI(TAG, "✓ I2C bus priming sequence completed");
            ESP_LOGI(TAG, "Ready for production initialization at %lu Hz", config.clockSpeed);

            return PrimingResult::SUCCESS;
        }

    private:
        /**
         * @brief Perform additional dummy I2C operations for thorough bus priming
         *
         * @param hal I2C HAL instance to use for operations
         * @param testRegAddr Register address for dummy operations
         */
        static void performDummyOperations(I2C_HAL &hal, uint8_t testRegAddr)
        {
            const char *TAG = "I2C_BUS_PRIMING";

            // Perform several dummy operations to thoroughly exercise the I2C bus
            ESP_LOGD(TAG, "Performing additional dummy operations...");

            // Dummy ping operations
            for (int i = 0; i < 3; i++)
            {
                hal.ping();
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            // Dummy read operations at different register addresses
            uint8_t dummyData;
            hal.readByte(testRegAddr, dummyData);
            hal.readByte(testRegAddr + 1, dummyData);
            hal.readByte(0xFF, dummyData); // Try invalid register to exercise error paths

            // Small delay between operations
            vTaskDelay(pdMS_TO_TICKS(5));

            ESP_LOGD(TAG, "Dummy operations completed");
        }
    };

} // namespace bmi270
