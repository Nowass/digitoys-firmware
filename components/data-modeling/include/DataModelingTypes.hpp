#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace digitoys::datamodeling
{
    /**
     * @brief Enhanced data model for RC car behavior analysis
     * 
     * This structure groups data logically for advanced analytics:
     * - Group 1: RC Control & Input
     * - Group 2: Safety & Obstacle Detection  
     * - Group 3: Motion & Physics
     * - Group 4: Test Session & Events
     */
    struct BehaviorDataPoint
    {
        // === GROUP 1: RC CONTROL & INPUT ===
        struct RCControl
        {
            float rc_input = 0.0f;          // RC strength (0.0-1.0)
            float rc_raw_pwm = 0.0f;        // Raw PWM value for calibration
            bool driving_forward = false;   // Direction boolean
            bool wants_reverse = false;     // Reverse input detected
            bool throttle_pressed = false;  // Any throttle input active
            std::string control_mode = "manual"; // manual/auto/override
        } rc_control;

        // === GROUP 2: SAFETY & OBSTACLE DETECTION ===
        struct SafetyData
        {
            float obstacle_distance = 0.0f;     // LiDAR reading (meters)
            float warning_distance = 0.0f;      // Dynamic warning threshold
            float brake_distance = 0.0f;        // Dynamic brake threshold
            float safety_margin = 0.0f;         // obstacle_distance - brake_distance
            bool is_warning_state = false;      // Warning active
            bool is_obstacle_state = false;     // Emergency brake active
            float obstacle_confidence = 1.0f;   // LiDAR reading reliability (0.0-1.0)
        } safety_data;

        // === GROUP 3: MOTION & PHYSICS ===
        struct PhysicsData
        {
            float calculated_speed = 0.0f;      // Derived from distance_delta + sample_rate (m/s)
            float speed_delta = 0.0f;           // Speed change between samples
            float distance_delta = 0.0f;        // Distance change between samples  
            float deceleration = 0.0f;          // Calculated deceleration (m/s²)
            float time_to_impact = 0.0f;        // Collision estimate (seconds)
            float actual_stopping_distance = 0.0f; // Measured brake-to-stop distance
            float stopping_time = 0.0f;         // Time from brake to complete stop
            float physics_accuracy = 0.0f;      // Calculated vs actual accuracy ratio
        } physics_data;

        // === GROUP 4: TEST SESSION & EVENTS ===
        struct SessionData
        {
            uint32_t session_id = 0;            // Test session identifier (1,2,3...)
            std::string scenario_description = ""; // Scenario description
            uint64_t absolute_timestamp_us = 0;  // Absolute ESP timer timestamp
            uint64_t session_relative_ms = 0;    // Milliseconds since session start
            uint32_t brake_event_id = 0;        // Unique brake event counter
            uint32_t stop_event_id = 0;         // Manual stop event counter  
            bool stop_event_marked = false;     // Manual stop flag from dashboard
            uint64_t console_log_timestamp = 0; // Reference to console log
        } session_data;

        // Convenience constructor
        BehaviorDataPoint() = default;
        
        // Copy constructor for data snapshots
        BehaviorDataPoint(const BehaviorDataPoint&) = default;
        BehaviorDataPoint& operator=(const BehaviorDataPoint&) = default;
    };

    /**
     * @brief Test session metadata and results
     */
    struct TestSessionData
    {
        uint32_t session_id = 0;
        std::string description;
        uint64_t start_timestamp_us = 0;
        uint64_t end_timestamp_us = 0;
        uint32_t duration_ms = 0;
        
        uint32_t sample_count = 0;
        uint32_t brake_event_count = 0;
        uint32_t stop_event_count = 0;
        
        float max_speed = 0.0f;
        float avg_speed = 0.0f;
        float total_distance = 0.0f;
        bool is_active = false;
    };

    /**
     * @brief Data export configuration
     */
    struct ExportConfig
    {
        bool include_session_header = true;     // Include session info at top
        bool include_console_logs = false;      // Include console log correlation
        bool export_all_groups = true;         // Export all data groups
        std::vector<uint32_t> session_ids;     // Specific sessions to export (empty = all)
        std::string export_format = "csv";     // csv, json, both
        
        // Group-specific export flags
        bool export_rc_control = true;
        bool export_safety_data = true;
        bool export_physics_data = true;
        bool export_session_data = true;
    };

} // namespace digitoys::datamodeling
