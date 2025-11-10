#include "ObstacleDetector.hpp"
#include <Constants.hpp>
#include <algorithm>

namespace control
{

    float ObstacleDetector::calculateBrakeDistance(float current_duty) const
    {
        if (current_duty <= digitoys::constants::pwm::NEUTRAL_DUTY + digitoys::constants::pwm::DIRECTION_TOLERANCE)
        {
            return digitoys::constants::distances::MIN_BRAKE_DISTANCE_M; // At neutral/reverse - use minimum
        }

        // Normalize duty to speed factor (0.0 to 1.0)
        float speed_factor = calculateSpeedFactor(current_duty);

        // Quadratic scaling for more aggressive high-speed braking
        float scaled_factor = speed_factor * speed_factor; // Square it for exponential increase

        // Linear interpolation between min and max brake distance
        return digitoys::constants::distances::MIN_BRAKE_DISTANCE_M +
               scaled_factor * (digitoys::constants::distances::MAX_BRAKE_DISTANCE_M - digitoys::constants::distances::MIN_BRAKE_DISTANCE_M);
    }

    float ObstacleDetector::calculateWarningDistance(float current_duty) const
    {
        if (current_duty <= digitoys::constants::pwm::NEUTRAL_DUTY + digitoys::constants::pwm::DIRECTION_TOLERANCE)
        {
            return digitoys::constants::distances::MIN_WARNING_DISTANCE_M;
        }

        float speed_factor = calculateSpeedFactor(current_duty);

        // Quadratic scaling for more aggressive high-speed warning
        float scaled_factor = speed_factor * speed_factor;

        return digitoys::constants::distances::MIN_WARNING_DISTANCE_M +
               scaled_factor * (digitoys::constants::distances::MAX_WARNING_DISTANCE_M - digitoys::constants::distances::MIN_WARNING_DISTANCE_M);
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
        float speed_factor = (current_duty - digitoys::constants::pwm::NEUTRAL_DUTY) /
                             (digitoys::constants::control_task::HIGH_SPEED_DUTY - digitoys::constants::pwm::NEUTRAL_DUTY);
        return std::max(0.0f, std::min(1.0f, speed_factor)); // Clamp to [0,1]
    }

} // namespace control
