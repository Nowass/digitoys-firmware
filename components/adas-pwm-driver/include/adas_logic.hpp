// adas_logic.hpp
#pragma once

#include <utility>

namespace adas
{

    /// Obstacle info as published by LidarTask
    struct ObstacleData
    {
        float distance_m; ///< nearest obstacle distance in meters
        float angle_deg;  ///< angle of obstacle relative to front (°)
    };

    /// Compute ADAS throttle and steering commands.
    /// @param obs current obstacle data
    /// @param raw_throttle incoming throttle duty normalized [0,1]
    /// @param raw_steering incoming steering duty normalized [0,1]
    /// @return {throttle, steering}, each normalized [0,1]
    std::pair<float, float> compute_adas_commands(const ObstacleData &obs,
                                                  float raw_throttle,
                                                  float raw_steering);

} // namespace adas