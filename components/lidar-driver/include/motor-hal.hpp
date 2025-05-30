#pragma once
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"


namespace lidar {

class MotorHAL {
public:
  esp_err_t init(ledc_channel_t channel,
                 gpio_num_t pin,
                 uint32_t freq_hz,
                 uint8_t duty_pct);

  esp_err_t start();
  esp_err_t stop();
  esp_err_t setSpeed(uint8_t duty_pct);

private:
  ledc_channel_t _channel;
  gpio_num_t     _pin;
};

} // namespace lidar