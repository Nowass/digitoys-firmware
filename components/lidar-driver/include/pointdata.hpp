/**
 * @file pointdata.hpp
 * @brief Basic structures representing LiDAR measurement points.
 */
#ifndef _POINT_DATA_HPP_
#define _POINT_DATA_HPP_

#include <stdint.h>

#include <iostream>
#include <vector>

namespace lidar
{

#define ANGLE_TO_RADIAN(angle) ((angle) * 3141.59 / 180000)
#define RADIAN_TO_ANGLED(angle) ((angle) * 180000 / 3141.59)

  /// Single point measurement from the LiDAR
  struct PointData
  {
    // Polar coordinate representation
    float angle;       ///< angle in degrees [0,360)
    uint16_t distance; ///< distance in millimetres
    uint8_t intensity; ///< return signal strength
    // Cartesian coordinate representation
    double x;
    double y;
    PointData(float angle, uint16_t distance, uint8_t intensity, double x = 0,
              double y = 0)
    {
      this->angle = angle;
      this->distance = distance;
      this->intensity = intensity;
      this->x = x;
      this->y = y;
    }
    PointData() {}
    friend std::ostream &operator<<(std::ostream &os, const PointData &data)
    {
      os << data.angle << " " << data.distance << " " << (int)data.intensity
         << " " << data.x << " " << data.y;
      return os;
    }
  };

  typedef std::vector<PointData> Points2D;

} // namespace lidar

#endif // _POINT_DATA_HPP_
