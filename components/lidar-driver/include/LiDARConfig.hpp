#pragma once

#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_err.h>

namespace lidar
{

  struct LiDARConfig
  {
    // UART configuration
    uart_port_t uartPort = UART_NUM_1;
    gpio_num_t txPin = GPIO_NUM_NC;
    gpio_num_t rxPin = GPIO_NUM_NC;
    int baudRate = 230400;
    int dmaBufferLen = 1024;

    // Angles defining the sector used for obstacle detection.
    // If `angleMinDeg` is greater than `angleMaxDeg`, the range wraps
    // around 360° (e.g. 350° .. 10°).
    float angleMinDeg = 0.0f;
    float angleMaxDeg = 360.0f;

    // Motor control configuration
    gpio_num_t motorPin = GPIO_NUM_NC;
    ledc_channel_t motorChannel = LEDC_CHANNEL_0;
    uint32_t motorFreqHz = 50000;
    int motorDutyPct = 50;

    // High level settings
    float obstacleThreshold = 0.8f; // [m] - distance threshold for obstacle detection
    float warningThreshold = 1.2f;  // [m] - distance threshold for warning

    /**
     * @brief Validate this configuration
     * @return ESP_OK if valid, error code otherwise
     */
    esp_err_t validate() const;

    /**
     * @brief Get default configuration with safe values
     * @return Default LiDAR configuration
     */
    static LiDARConfig getDefault();

    /**
     * @brief Create configuration using centralized constants
     * @return Configuration populated with values from digitoys::constants
     */
    static LiDARConfig createFromConstants();

    /**
     * @brief Create production configuration with validation
     * @return Validated production LiDAR configuration
     */
    static LiDARConfig createProductionConfig();

    /**
     * @brief Create test configuration with validation
     * @return Validated test LiDAR configuration
     */
    static LiDARConfig createTestConfig();

  private:
    static const char *TAG;
  };

} // namespace lidar
