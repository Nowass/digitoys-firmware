#pragma once

#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

namespace lidar
{

  struct LiDARConfig
  {
    uart_port_t uartPort = UART_NUM_1;
    gpio_num_t txPin = GPIO_NUM_NC;
    gpio_num_t rxPin = GPIO_NUM_NC;
    int baudRate = 230400;
    int dmaBufferLen = 1024;

    // Angles defining the sector used for obstacle detection.Add commentMore actions
    // If `angleMinDeg` is greater than `angleMaxDeg`, the range wraps
    // around 360° (e.g. 350° .. 10°).
    float angleMinDeg = 0.0f;
    float angleMaxDeg = 360.0f;

    gpio_num_t motorPin = GPIO_NUM_NC;
    ledc_channel_t motorChannel = LEDC_CHANNEL_0;
    uint32_t motorFreqHz = 50000;
    int motorDutyPct = 50;

    // --- High level settings ---
    float obstacleThreshold = 0.8f; // [m]
    float warningThreshold = 1.2f;  // [m]
  };

} // namespace lidar
