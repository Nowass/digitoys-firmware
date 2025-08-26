#include "LiDARConfig.hpp"
#include <DigitoysCoreAll.hpp>
#include <Logger.hpp>

using namespace digitoys::core;
using namespace digitoys::constants;

namespace lidar
{

    const char *LiDARConfig::TAG = "LiDARConfig";

    esp_err_t LiDARConfig::validate() const
    {
        // Register with centralized logging system (one-time registration)
        static bool registered = false;
        if (!registered) {
            DIGITOYS_REGISTER_COMPONENT("LiDARConfig", "CONFIG");
            registered = true;
        }

        // Validate UART configuration
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateUartPort(uartPort, "UART port"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateGpio(txPin, "UART TX pin"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateGpio(rxPin, "UART RX pin"));

        // Validate baud rate
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateRange(baudRate, 9600, 921600, "UART baud rate"));

        // Validate DMA buffer size
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateBufferSize(dmaBufferLen, 512, 8192, "DMA buffer size"));

        // Validate angles
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateRange(angleMinDeg, 0.0f, 360.0f, "minimum angle"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateRange(angleMaxDeg, 0.0f, 360.0f, "maximum angle"));

        // Validate motor configuration
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateGpioOutput(motorPin, "motor control pin"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateLedcChannel(motorChannel, "motor LEDC channel"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateFrequency(motorFreqHz, 1000, 80000, "motor PWM frequency"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateDutyPercent(motorDutyPct, "motor duty percentage"));

        // Validate distance thresholds
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateRange(obstacleThreshold, 0.1f, 10.0f, "obstacle threshold"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateRange(warningThreshold, 0.1f, 10.0f, "warning threshold"));

        // Logical validation: warning threshold should be >= obstacle threshold
        if (warningThreshold < obstacleThreshold)
        {
            DIGITOYS_LOGE("LiDARConfig", "Warning threshold (%.2f m) must be >= obstacle threshold (%.2f m)",
                          warningThreshold, obstacleThreshold);
            return ESP_ERR_INVALID_ARG;
        }

        DIGITOYS_LOGI("LiDARConfig", "Configuration validation passed");
        return ESP_OK;
    }

    LiDARConfig LiDARConfig::getDefault()
    {
        LiDARConfig config;

        // Use safe default values
        config.uartPort = UART_NUM_1;
        config.txPin = GPIO_NUM_NC; // Must be set by user
        config.rxPin = GPIO_NUM_NC; // Must be set by user
        config.baudRate = 230400;
        config.dmaBufferLen = 2048;

        config.angleMinDeg = 0.0f;
        config.angleMaxDeg = 360.0f;

        config.motorPin = GPIO_NUM_NC; // Must be set by user
        config.motorChannel = LEDC_CHANNEL_0;
        config.motorFreqHz = 30000;
        config.motorDutyPct = 50;

        config.obstacleThreshold = 0.8f;
        config.warningThreshold = 1.2f;

        return config;
    }

    LiDARConfig LiDARConfig::createFromConstants()
    {
        LiDARConfig config;

        // Use centralized constants
        config.uartPort = hardware::LIDAR_UART_PORT;
        config.txPin = pins::LIDAR_TX;
        config.rxPin = pins::LIDAR_RX;
        config.baudRate = hardware::LIDAR_UART_BAUD;
        config.dmaBufferLen = hardware::LIDAR_DMA_BUFFER_SIZE;

        config.angleMinDeg = lidar_const::DEFAULT_ANGLE_MIN_DEG;
        config.angleMaxDeg = lidar_const::DEFAULT_ANGLE_MAX_DEG;

        config.motorPin = pins::LIDAR_MOTOR;
        config.motorChannel = hardware::LIDAR_MOTOR_CHANNEL;
        config.motorFreqHz = hardware::LIDAR_MOTOR_FREQ_HZ;
        config.motorDutyPct = hardware::LIDAR_MOTOR_DUTY_PCT;

        config.obstacleThreshold = distances::DEFAULT_OBSTACLE_THRESHOLD_M;
        config.warningThreshold = distances::DEFAULT_WARNING_THRESHOLD_M;

        return config;
    }

    LiDARConfig LiDARConfig::createProductionConfig()
    {
        auto config = createFromConstants();
        esp_err_t result = config.validate();
        digitoys::core::ComponentConfigFactory::validateConfigCreation("Production LiDAR Config", result);
        if (result == ESP_OK)
        {
            DIGITOYS_LOGI("LiDARConfig", "Created production config: UART%d, TX:%d, RX:%d, Motor:%d",
                          config.uartPort, config.txPin, config.rxPin, config.motorPin);
        }
        return config;
    }

    LiDARConfig LiDARConfig::createTestConfig()
    {
        auto config = createFromConstants();

        // Override with test-specific settings
        config.dmaBufferLen = 1024; // Smaller buffer for testing
        config.angleMinDeg = 0.0f;  // Full 360° scan for testing
        config.angleMaxDeg = 360.0f;
        config.obstacleThreshold = 1.0f; // More conservative for testing
        config.warningThreshold = 1.5f;

        esp_err_t result = config.validate();
        digitoys::core::ComponentConfigFactory::validateConfigCreation("Test LiDAR Config", result);
        return config;
    }

} // namespace lidar
