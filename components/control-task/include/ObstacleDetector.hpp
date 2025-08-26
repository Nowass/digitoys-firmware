#pragma once

#include "ControlState.hpp"

namespace control
{

    /**
     * @brief Handles obstacle detection with dynamic distance thresholds based on speed
     */
    class ObstacleDetector
    {
    public:
        ObstacleDetector() = default;

        /**
         * @brief Calculate dynamic brake distance based on current speed
         * @param current_duty Current PWM duty cycle representing speed
         * @return Brake distance in meters
         */
        float calculateBrakeDistance(float current_duty) const;

        /**
         * @brief Calculate dynamic warning distance based on current speed
         * @param current_duty Current PWM duty cycle representing speed
         * @return Warning distance in meters
         */
        float calculateWarningDistance(float current_duty) const;

        /**
         * @brief Determine if current distance constitutes a dynamic obstacle
         * @param distance Current obstacle distance
         * @param current_duty Current PWM duty cycle
         * @return true if obstacle detected based on dynamic threshold
         */
        bool isDynamicObstacle(float distance, float current_duty) const;

        /**
         * @brief Determine if current distance constitutes a dynamic warning
         * @param distance Current obstacle distance
         * @param current_duty Current PWM duty cycle
         * @return true if warning detected based on dynamic threshold
         */
        bool isDynamicWarning(float distance, float current_duty) const;

    private:
        /**
         * @brief Calculate normalized speed factor from duty cycle
         * @param current_duty Current PWM duty cycle
         * @return Speed factor normalized to [0,1]
         */
        float calculateSpeedFactor(float current_duty) const;
    };

} // namespace control
