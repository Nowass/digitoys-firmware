#pragma once

#include "ControlState.hpp"
#include "RCInputProcessor.hpp"
#include "ObstacleDetector.hpp"
#include "adas_pwm_driver.hpp"
#include "LiDAR.hpp"

namespace control
{

    /**
     * @brief Handles speed control logic including braking, warning, and slowdown
     */
    class SpeedController
    {
    public:
        /**
         * @brief Actions the speed controller can take
         */
        enum class Action
        {
            RESUME_PASSTHROUGH, // Resume normal RC passthrough
            APPLY_BRAKE,        // Apply full brake
            GRADUAL_SLOWDOWN,   // Apply gradual slowdown
            MAINTAIN_SPEED,     // Maintain current speed
            CLEAR_WARNING,      // Clear warning state
            NO_ACTION           // No action needed
        };

        SpeedController() = default;

        /**
         * @brief Handle reverse motion - clears all brake states
         * @param state Control state to modify
         * @return Action to take
         */
        Action handleReverseMotion(ControlState &state);

        /**
         * @brief Handle obstacle state (full brake scenario)
         * @param state Control state
         * @param rc_status Current RC status
         * @param current_time Current time in milliseconds
         * @param driver PWM driver instance
         * @param rc_processor RC processor for checking input
         * @return Action to take
         */
        Action handleObstacleState(ControlState &state, const RCInputProcessor::RCStatus &rc_status,
                                   uint32_t current_time, adas::PwmDriver &driver,
                                   const RCInputProcessor &rc_processor);

        /**
         * @brief Handle warning state (gradual slowdown scenario)
         * @param state Control state
         * @param rc_status Current RC status
         * @param distance Current obstacle distance
         * @param current_time Current time in milliseconds
         * @param driver PWM driver instance
         * @param rc_processor RC processor for checking input
         * @param obstacle_detector For calculating warning distances
         * @return Action to take
         */
        Action handleWarningState(ControlState &state, const RCInputProcessor::RCStatus &rc_status,
                                  float distance, uint32_t current_time, adas::PwmDriver &driver,
                                  const RCInputProcessor &rc_processor, const ObstacleDetector &obstacle_detector);

        /**
         * @brief Handle clear path (no obstacles detected)
         * @param state Control state to clear if needed
         * @return Action to take
         */
        Action handleClearPath(ControlState &state);

        /**
         * @brief Handle neutral throttle position
         * @param state Control state to clear if needed
         * @return Action to take
         */
        Action handleNeutralThrottle(ControlState &state);

        /**
         * @brief Execute the specified action
         * @param action Action to execute
         * @param driver PWM driver instance
         * @param state Control state
         * @param slowdown_duty Duty cycle for slowdown (when applicable)
         */
        void executeAction(Action action, adas::PwmDriver &driver, ControlState &state,
                           float slowdown_duty = 0.0f);

    private:
        /**
         * @brief Apply gradual slowdown logic
         * @param state Control state
         * @param distance Current distance
         * @return New slowdown duty cycle
         */
        float applyGradualSlowdown(ControlState &state, float distance);

        /**
         * @brief Check if RC input allows escape from brake/warning
         * @param rc_status Current RC status
         * @param slowdown_duty Current slowdown duty (for warning state)
         * @return true if escape is allowed
         */
        bool allowsEscape(const RCInputProcessor::RCStatus &rc_status, float slowdown_duty = 0.0f) const;
    };

} // namespace control
