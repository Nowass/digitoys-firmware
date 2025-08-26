#pragma once

#include "ControlState.hpp"
#include "adas_pwm_driver.hpp"

namespace control
{

    /**
     * @brief Handles RC input processing and direction detection
     */
    class RCInputProcessor
    {
    public:
        /**
         * @brief Status of RC input processing
         */
        struct RCStatus
        {
            float current_input = 0.0f;    // Current RC input duty cycle
            bool driving_forward = false;  // True if driving forward
            bool wants_reverse = false;    // True if reverse input detected
            bool throttle_pressed = false; // True if throttle is pressed (any direction)
            bool at_neutral = false;       // True if at neutral position

            // Additional cached vs direct reading info for diagnostics
            float cached_duty = 0.0f;     // Cached duty reading
            float direct_duty = 0.0f;     // Direct duty reading
            bool cached_throttle = false; // Cached throttle state
        };

        RCInputProcessor() = default;

        /**
         * @brief Process current RC input and determine status
         * @param driver PWM driver instance
         * @return RCStatus with all relevant RC information
         */
        RCStatus processRCInput(adas::PwmDriver &driver) const;

        /**
         * @brief Check if it's time to check RC input during brake/warning states
         * @param current_time Current time in milliseconds
         * @param last_check_time Last RC check time
         * @return true if RC check should be performed
         */
        bool shouldCheckRCDuringBrake(uint32_t current_time, uint32_t last_check_time) const;

        /**
         * @brief Perform RC check during brake/warning by temporarily resuming passthrough
         * @param driver PWM driver instance
         * @return RCStatus after checking RC input
         */
        RCStatus performRCCheck(adas::PwmDriver &driver) const;

        /**
         * @brief Log diagnostic information about RC input (called periodically)
         * @param status Current RC status
         */
        void logDiagnostics(const RCStatus &status, int &log_counter) const;

    private:
        /**
         * @brief Determine if RC input represents forward motion
         * @param duty_cycle RC input duty cycle
         * @return true if forward motion detected
         */
        bool isDrivingForward(float duty_cycle) const;

        /**
         * @brief Determine if RC input represents reverse motion
         * @param duty_cycle RC input duty cycle
         * @return true if reverse motion detected
         */
        bool wantsReverse(float duty_cycle) const;

        /**
         * @brief Determine if RC input is at neutral position
         * @param duty_cycle RC input duty cycle
         * @return true if at neutral
         */
        bool isAtNeutral(float duty_cycle) const;

        /**
         * @brief Determine if throttle is pressed in any direction
         * @param duty_cycle RC input duty cycle
         * @return true if throttle pressed
         */
        bool isThrottlePressed(float duty_cycle) const;
    };

} // namespace control
