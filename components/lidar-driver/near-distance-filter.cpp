#include "near-distance-filter.hpp"
#include <algorithm>
#include <cmath>

namespace lidar
{

  NearDistanceFilter::NearDistanceFilter(int speed)
      : curr_speed_(speed) {}

  std::vector<PointData> NearDistanceFilter::NearFilter(const std::vector<PointData> &input) const
  {
    std::vector<PointData> normal, pending, item;
    std::vector<std::vector<PointData>> groups;

    if (input.empty())
      return normal;

    // 1. Split data into near (within 5m) and normal
    for (const auto &pt : input)
    {
      if (pt.distance < 5000)
      {
        pending.push_back(pt);
      }
      else
      {
        normal.push_back(pt);
      }
    }

    // 2. Calculate grouping threshold
    const double angle_delta_limit = static_cast<double>(curr_speed_) / kScanFrequency * 2.0;

    // 3. Sort by angle
    std::sort(pending.begin(), pending.end(), [](const PointData &a, const PointData &b)
              { return a.angle < b.angle; });

    PointData last{-10, 0, 0};

    // 4. Group close points
    for (const auto &pt : pending)
    {
      if (std::abs(pt.angle - last.angle) > angle_delta_limit ||
          std::abs(pt.distance - last.distance) > last.distance * 0.03)
      {
        if (!item.empty())
        {
          groups.push_back(item);
          item.clear();
        }
      }
      item.push_back(pt);
      last = pt;
    }
    if (!item.empty())
      groups.push_back(item);

    if (groups.empty())
      return normal;

    // 5. Merge 0° ↔ 359° group if similar
    const auto &first_pt = groups.front().front();
    const auto &last_pt = groups.back().back();

    if (std::fabs(first_pt.angle + 360.0 - last_pt.angle) < angle_delta_limit &&
        std::abs(first_pt.distance - last_pt.distance) < last_pt.distance * 0.03)
    {
      groups.front().insert(groups.front().begin(), groups.back().begin(), groups.back().end());
      groups.pop_back();
    }

    // 6. Filter groups
    for (auto &cluster : groups)
    {
      if (cluster.empty())
        continue;

      if (cluster.size() > 15)
      {
        normal.insert(normal.end(), cluster.begin(), cluster.end());
        continue;
      }

      if (cluster.size() < 3)
      {
        int avg_intensity = 0;
        for (const auto &pt : cluster)
          avg_intensity += pt.intensity;
        avg_intensity /= static_cast<int>(cluster.size());

        if (avg_intensity < kIntensitySingle)
        {
          for (auto &pt : cluster)
          {
            pt.distance = 0;
            pt.intensity = 0;
          }
        }
      }
      else
      {
        double avg_intensity = 0.0;
        double avg_distance = 0.0;
        for (const auto &pt : cluster)
        {
          avg_intensity += pt.intensity;
          avg_distance += pt.distance;
        }
        avg_intensity /= cluster.size();
        avg_distance /= cluster.size();

        if (avg_intensity <= kIntensityLow)
        {
          for (auto &pt : cluster)
          {
            pt.distance = 0;
            pt.intensity = 0;
          }
        }
      }

      normal.insert(normal.end(), cluster.begin(), cluster.end());
    }

    return normal;
  }

} // namespace lidar
