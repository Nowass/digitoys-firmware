/**
 * @file LiDARConfig.hpp
 * @brief Configuration parameters for the LiDAR driver.
 */
#pragma once

#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>

namespace lidar
{

  /// Runtime configuration for the LiDAR driver
  struct LiDARConfig
  {
    uart_port_t uartPort = UART_NUM_1; ///< UART to which the sensor is attached
    gpio_num_t txPin = GPIO_NUM_NC;    ///< UART TX pin
    gpio_num_t rxPin = GPIO_NUM_NC;    ///< UART RX pin
    int baudRate = 230400;             ///< Serial baud rate
    int dmaBufferLen = 1024;           ///< DMA buffer length for UART driver

    // Angles defining the sector used for obstacle detection. If
    // `angleMinDeg` is greater than `angleMaxDeg`, the range wraps
    // around 360° (e.g. 350° .. 10°).
    float angleMinDeg = 0.0f; ///< start angle of detection sector
    float angleMaxDeg = 360.0f; ///< end angle of detection sector

    gpio_num_t motorPin = GPIO_NUM_NC;     ///< GPIO controlling motor PWM
    ledc_channel_t motorChannel = LEDC_CHANNEL_0; ///< LEDC channel for motor
    uint32_t motorFreqHz = 50000;         ///< PWM frequency for motor
    int motorDutyPct = 50;                ///< Duty percent while running

    // --- High level settings ---
    float obstacleThreshold = 0.8f; ///< distance causing braking
    float warningThreshold = 1.2f;  ///< distance for early warning
  };

} // namespace lidar
