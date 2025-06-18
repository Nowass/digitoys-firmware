#pragma once
/**
 * @file near-distance-filter.hpp
 * @brief Filters clusters of points that are too close to be reliable.
 */
#include <math.h>
#include <string.h>

#include <algorithm>
#include "pointdata.hpp"
#include <vector>

namespace lidar
{

  /// Filters spurious measurements very close to the sensor.
  class [[nodiscard]] NearDistanceFilter final
  {
  public:
    /// @param speed current rotation speed from the LiDAR
    explicit NearDistanceFilter(int speed);
    ~NearDistanceFilter() = default;

    /// Remove unreliable close-range points from @p input
    [[nodiscard]] std::vector<PointData> NearFilter(const std::vector<PointData> &input) const;

  private:
    int curr_speed_;
    static constexpr float kScanFrequency = 10.0f;
    static constexpr int kIntensitySingle = 180;
    static constexpr int kIntensityLow = 200;
  };

} // namespace lidar

