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
    int gpioPin_ = -1;
    bool isInitialized_ = false;
  };

} // namespace lidar
