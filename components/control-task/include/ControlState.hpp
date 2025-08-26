#pragma once

#include <cstdint>

namespace control
{

    /**
     * @brief Constants used throughout the control system
     */
    struct ControlConstants
    {
        // PWM duty constants
        static constexpr float BRAKE = 0.058f;       // full brake duty
        static constexpr float ZERO_SPEED = 0.0856f; // neutral duty (measured from RC)
        static constexpr float DUTY_STEP = 0.005f;
        static constexpr float DIRECTION_TOLERANCE = 0.005f; // tolerance for forward/reverse detection

        // Dynamic braking parameters
        static constexpr float MIN_BRAKE_DISTANCE = 0.5f;            // Minimum brake distance (safety)
        static constexpr float MAX_BRAKE_DISTANCE = 3.5f;            // Maximum brake distance (increased for earlier braking)
        static constexpr float MIN_WARNING_DISTANCE = 1.0f;          // Minimum warning distance
        static constexpr float MAX_WARNING_DISTANCE = 5.0f;          // Maximum warning distance (increased for earlier warning)
        static constexpr float HIGH_SPEED_DUTY = ZERO_SPEED + 0.03f; // ~3% above neutral (for scaling)

        // Timing constants
        static constexpr uint32_t RC_CHECK_INTERVAL_MS = 100; // Check RC input every 100ms during brake
        static constexpr uint32_t CONTROL_LOOP_DELAY_MS = 50; // Main control loop delay
        static constexpr uint32_t RC_READ_DELAY_MS = 20;      // Brief delay for fresh RC reading

        // Logging intervals
        static constexpr int DUTY_TEST_LOG_INTERVAL = 40; // 40 * 50ms = 2 seconds
        static constexpr int THRESHOLD_LOG_INTERVAL = 40; // 40 * 50ms = 2 seconds
    };

    /**
     * @brief Manages the state of the control system
     */
    class ControlState
    {
    public:
        ControlState() = default;

        // State getters
        bool isObstacleState() const { return obstacle_state_; }
        bool isWarningState() const { return warning_state_; }
        float getSlowdownDuty() const { return slowdown_duty_; }
        float getLastDistance() const { return last_distance_; }
        uint32_t getLastRCCheckTime() const { return last_rc_check_time_; }

        // State setters
        void setObstacleState(bool state) { obstacle_state_ = state; }
        void setWarningState(bool state) { warning_state_ = state; }
        void setSlowdownDuty(float duty) { slowdown_duty_ = duty; }
        void setLastDistance(float distance) { last_distance_ = distance; }
        void setLastRCCheckTime(uint32_t time) { last_rc_check_time_ = time; }

        // State operations
        void clearAllStates()
        {
            obstacle_state_ = false;
            warning_state_ = false;
        }

        void initializeWarningState(float initial_duty, float distance, uint32_t current_time)
        {
            warning_state_ = true;
            obstacle_state_ = false;
            slowdown_duty_ = initial_duty;
            last_distance_ = distance;
            last_rc_check_time_ = current_time;
        }

        void initializeObstacleState(uint32_t current_time)
        {
            obstacle_state_ = true;
            warning_state_ = false;
            last_rc_check_time_ = current_time;
        }

        // Logging counters
        int &getDutyTestLogCounter() { return duty_test_log_counter_; }
        int &getThresholdLogCounter() { return threshold_log_counter_; }

    private:
        // Core state variables
        bool obstacle_state_ = false;
        bool warning_state_ = false;
        float slowdown_duty_ = 0.0f;
        float last_distance_ = 0.0f;
        uint32_t last_rc_check_time_ = 0;

        // Logging counters
        int duty_test_log_counter_ = 0;
        int threshold_log_counter_ = 0;
    };

} // namespace control
