#ifndef __NEAR_DISTANCE_FILTER_HPP_
#define __NEAR_DISTANCE_FILTER_HPP_

#include <math.h>
#include <string.h>

#include <algorithm>

#include "pointdata.hpp"

namespace lidar
{

  class NearDistanceFilter
  {
  private:
    const int kIntensityLow = 15;     // Low intensity threshold
    const int kIntensitySingle = 220; // Discrete points require higher intensity
    const int kScanFrequency = 4500;  // Default scan frequency, to change, read
                                      // according to radar protocol
    double curr_speed_;
    NearDistanceFilter() = delete;
    NearDistanceFilter(const NearDistanceFilter &) = delete;
    NearDistanceFilter &operator=(const NearDistanceFilter &) = delete;

  public:
    NearDistanceFilter(int speed);
    std::vector<PointData> NearFilter(const std::vector<PointData> &tmp) const;
    ~NearDistanceFilter();
  };

} // namespace lidar

#endif //__NEAR_DISTANCE_FILTER_HPP_