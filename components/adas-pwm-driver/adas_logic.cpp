// adas_logic.cpp
#include "adas_logic.hpp"
#include <algorithm>

namespace adas
{

    // Distances for braking
    static constexpr float MIN_SAFE_DISTANCE = 0.5f; // meters

    std::pair<float, float> compute_adas_commands(const ObstacleData &obs,
                                                  float raw_throttle,
                                                  float raw_steering)
    {
        // PWM duty percent swing: [6%..12%]. Center 9% = no motion
        constexpr float MIN_PERCENT = 0.06f; // 6%
        constexpr float MAX_PERCENT = 0.12f; // 12%

        // Determine throttle output percent
        float throttle_percent;
        if (obs.distance_m <= MIN_SAFE_DISTANCE)
        {
            // Obstacle too close: brake (full reverse = 6%)
            throttle_percent = MIN_PERCENT;
        }
        else
        {
            // No close obstacle: passthrough the raw throttle
            throttle_percent = MIN_PERCENT + (MAX_PERCENT - MIN_PERCENT) * raw_throttle;
        }
        // Convert back to normalized [0,1] for driver setDuty()
        float throttle = (throttle_percent - MIN_PERCENT) / (MAX_PERCENT - MIN_PERCENT);

        // Steering always passthrough for now
        float steering = raw_steering;

        return {throttle, steering};
    }

} // namespace adas
