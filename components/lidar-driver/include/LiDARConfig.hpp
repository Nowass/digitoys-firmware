// LiDARConfig.hpp (modernized C++23)
#pragma once

#include <driver/uart.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <cstdint>

namespace lidar
{

  struct LiDARConfig
  {
    uart_port_t uartPort = UART_NUM_1;
    gpio_num_t txPin = GPIO_NUM_NC;
    gpio_num_t rxPin = GPIO_NUM_NC;
    int baudRate = 230400;   // Default baud rate (bps)
    int dmaBufferLen = 1024; // DMA buffer size in bytes

    float angleMinDeg = 0.0f;   // Min scan angle
    float angleMaxDeg = 360.0f; // Max scan angle

    gpio_num_t motorPin = GPIO_NUM_NC; // Motor control pin
    ledc_channel_t motorChannel = LEDC_CHANNEL_0;
    int motorFreqHz = 50000; // PWM frequency (Hz)
    int motorDutyPct = 50;   // PWM duty cycle (0–100%)
  };

} // namespace lidar
