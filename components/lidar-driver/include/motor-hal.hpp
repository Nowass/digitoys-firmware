#pragma once

#include "LiDARConfig.hpp"
#include <esp_err.h>

namespace lidar
{

  class [[nodiscard]] Motor_HAL final
  {
  public:
    Motor_HAL() = default;
    ~Motor_HAL();

    esp_err_t init(const LiDARConfig &cfg);
    esp_err_t start();
    esp_err_t stop();
    void deinit();

  private:
    gpio_num_t gpioPin_ = GPIO_NUM_NC;
    ledc_channel_t pwmChannel_ = LEDC_CHANNEL_0;
    int pwmResolutionBits_ = 10;
    int pwmDuty_ = 512; // 50% duty cycle
    bool isInitialized_ = false;
  };

} // namespace lidar
