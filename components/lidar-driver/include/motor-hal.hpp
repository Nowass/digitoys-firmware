/**
 * @file motor-hal.hpp
 * @brief Simple PWM based control for the LiDAR spin motor.
 */
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

    /// Configure LEDC for the motor
    /// @param cfg configuration structure
    esp_err_t init(const LiDARConfig &cfg);
    /// Start spinning the motor
    esp_err_t start();
    /// Stop the motor
    esp_err_t stop();
    /// Release LEDC resources
    void deinit();

  private:
    gpio_num_t gpioPin_ = GPIO_NUM_NC;
    ledc_channel_t pwmChannel_ = LEDC_CHANNEL_0;
    int pwmResolutionBits_ = 10;
    int pwmDuty_ = 0;
    bool isInitialized_ = false;
  };

} // namespace lidar
