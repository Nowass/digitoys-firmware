#pragma once
#include <math.h>
#include <string.h>

#include <algorithm>
#include "pointdata.hpp"
#include <vector>

namespace lidar
{

  class [[nodiscard]] NearDistanceFilter final
  {
  public:
    explicit NearDistanceFilter(int speed);
    ~NearDistanceFilter() = default;

    [[nodiscard]] std::vector<PointData> NearFilter(const std::vector<PointData> &input) const;

  private:
    int curr_speed_;
    static constexpr float kScanFrequency = 10.0f;
    static constexpr int kIntensitySingle = 180;
    static constexpr int kIntensityLow = 200;
  };

} // namespace lidar
