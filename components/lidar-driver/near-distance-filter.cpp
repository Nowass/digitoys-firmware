#include "near-distance-filter.hpp"
/**
 * @file near-distance-filter.cpp
 * @brief Implementation of LiDAR near distance filtering algorithm.
 */
#include <algorithm>
#include <cmath>
#include <ranges>
#include <iterator>

namespace lidar
{

  NearDistanceFilter::NearDistanceFilter(int speed)
      : curr_speed_(speed) {}

  std::vector<PointData> NearDistanceFilter::NearFilter(const std::vector<PointData> &input) const
  {
    std::vector<PointData> normal, pending;
    std::vector<std::vector<PointData>> groups;

    if (input.empty())
      return normal;

    // 1. Split into 'pending' (<5m) and 'normal'
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

    if (pending.empty())
      return normal;

    // 2. Calculate delta angle threshold
    const double angle_delta_limit = static_cast<double>(curr_speed_) / kScanFrequency * 2.0;

    // 3. Sort by angle (C++20 style)
    std::ranges::sort(pending, {}, &PointData::angle);

    // 4. Group by similarity
    std::vector<PointData> item;
    item.reserve(16);
    groups.reserve(pending.size() / 3);

    PointData last{-10, 0, 0};

    for (const auto &pt : pending)
    {
      const bool newGroup =
          std::abs(pt.angle - last.angle) > angle_delta_limit ||
          std::abs(pt.distance - last.distance) > last.distance * 0.03;

      if (newGroup && !item.empty())
      {
        groups.push_back(std::move(item));
        item.clear();
      }

      item.push_back(pt);
      last = pt;
    }
    if (!item.empty())
      groups.push_back(std::move(item));

    if (groups.empty())
      return normal;

    // 5. Merge 0° ↔ 359° wraparound groups if similar
    const auto &first = groups.front().front();
    const auto &lastPt = groups.back().back();

    if (std::fabs(first.angle + 360.0 - lastPt.angle) < angle_delta_limit &&
        std::abs(first.distance - lastPt.distance) < lastPt.distance * 0.03)
    {
      auto &frontGroup = groups.front();
      auto &backGroup = groups.back();
      frontGroup.insert(frontGroup.begin(),
                        std::make_move_iterator(backGroup.begin()),
                        std::make_move_iterator(backGroup.end()));
      groups.pop_back();
    }

    // 6. Evaluate each group
    for (auto &cluster : groups)
    {
      if (cluster.empty())
        continue;

      const std::size_t n = cluster.size();
      if (n > 15)
      {
        normal.insert(normal.end(),
                      std::make_move_iterator(cluster.begin()),
                      std::make_move_iterator(cluster.end()));
        continue;
      }

      if (n < 3)
      {
        int avg_intensity = 0;
        for (const auto &pt : cluster)
          avg_intensity += pt.intensity;
        avg_intensity /= static_cast<int>(n);

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
        double avg_intensity = 0.0, avg_distance = 0.0;
        for (const auto &pt : cluster)
        {
          avg_intensity += pt.intensity;
          avg_distance += pt.distance;
        }
        avg_intensity /= n;
        avg_distance /= n;

        if (avg_intensity <= kIntensityLow)
        {
          for (auto &pt : cluster)
          {
            pt.distance = 0;
            pt.intensity = 0;
          }
        }
      }

      normal.insert(normal.end(),
                    std::make_move_iterator(cluster.begin()),
                    std::make_move_iterator(cluster.end()));
    }

    return normal;
  }

} // namespace lidar
