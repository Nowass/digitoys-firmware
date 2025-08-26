#include "ObstacleDetector.hpp"
#include <algorithm>

namespace control
{

    float ObstacleDetector::calculateBrakeDistance(float current_duty) const
    {
        if (current_duty <= ControlConstants::ZERO_SPEED + ControlConstants::DIRECTION_TOLERANCE)
        {
            return ControlConstants::MIN_BRAKE_DISTANCE; // At neutral/reverse - use minimum
        }

        // Normalize duty to speed factor (0.0 to 1.0)
        float speed_factor = calculateSpeedFactor(current_duty);

        // Quadratic scaling for more aggressive high-speed braking
        float scaled_factor = speed_factor * speed_factor; // Square it for exponential increase

        // Linear interpolation between min and max brake distance
        return ControlConstants::MIN_BRAKE_DISTANCE +
               scaled_factor * (ControlConstants::MAX_BRAKE_DISTANCE - ControlConstants::MIN_BRAKE_DISTANCE);
    }

    float ObstacleDetector::calculateWarningDistance(float current_duty) const
    {
        if (current_duty <= ControlConstants::ZERO_SPEED + ControlConstants::DIRECTION_TOLERANCE)
        {
            return ControlConstants::MIN_WARNING_DISTANCE;
        }

        float speed_factor = calculateSpeedFactor(current_duty);

        // Quadratic scaling for more aggressive high-speed warning
        float scaled_factor = speed_factor * speed_factor;

        return ControlConstants::MIN_WARNING_DISTANCE +
               scaled_factor * (ControlConstants::MAX_WARNING_DISTANCE - ControlConstants::MIN_WARNING_DISTANCE);
    }

    bool ObstacleDetector::isDynamicObstacle(float distance, float current_duty) const
    {
        return distance <= calculateBrakeDistance(current_duty);
    }

    bool ObstacleDetector::isDynamicWarning(float distance, float current_duty) const
    {
        float warning_distance = calculateWarningDistance(current_duty);
        return !isDynamicObstacle(distance, current_duty) && distance <= warning_distance;
    }

    float ObstacleDetector::calculateSpeedFactor(float current_duty) const
    {
        // Normalize duty to speed factor (0.0 to 1.0)
        float speed_factor = (current_duty - ControlConstants::ZERO_SPEED) /
                             (ControlConstants::HIGH_SPEED_DUTY - ControlConstants::ZERO_SPEED);
        return std::max(0.0f, std::min(1.0f, speed_factor)); // Clamp to [0,1]
    }

} // namespace control
