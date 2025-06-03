#pragma once
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

namespace lidar
{

  struct LiDARConfig
  {
    uart_port_t uartPort;
    gpio_num_t txPin;
    gpio_num_t rxPin;
    size_t dmaBufferLen;
    float angleMinDeg;
    float angleMaxDeg;
    gpio_num_t motorPin;         // LEDC PWM output pin
    ledc_channel_t motorChannel; // LEDC channel
    uint32_t motorFreqHz;        // Motor PWM frequency (e.g. 400 Hz)
    uint8_t motorDutyPct;        // Motor PWM duty cycle (0–100)
  };

  // Returns board- or app-specific defaults
  static inline LiDARConfig defaultConfig()
  {
    return LiDARConfig{
        .uartPort = UART_NUM_1,
        .txPin = GPIO_NUM_17,
        .rxPin = GPIO_NUM_16,
        .dmaBufferLen = 512,
        .angleMinDeg = -12.0,
        .angleMaxDeg = +12.0,
        .motorPin = GPIO_NUM_18,
        .motorChannel = LEDC_CHANNEL_0,
        .motorFreqHz = 400,
        .motorDutyPct = 50};
  }

} // namespace lidar