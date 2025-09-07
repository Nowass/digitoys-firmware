#pragma once

#include "IDataSource.hpp"

// Forward declarations to avoid heavy include dependencies
namespace control
{
    class ControlTask;
    struct RCStatus;
    class ObstacleDetector;
    class ControlState;
}

namespace lidar
{
    class LiDAR;
}

namespace digitoys::datalogger
{
    /**
     * @brief Data source for capturing control system telemetry
     *
     * This source extracts real-time control system data including:
     * - RC input processing (speeds, throttle states, direction)
     * - LiDAR obstacle detection (distances, warnings)
     * - Dynamic braking calculations
     * - Control state information
     * - Physics analysis data for braking optimization
     */
    class ControlTaskDataSource : public IDataSource
    {
    public:
        /**
         * @brief Constructor
         * @param control_task Reference to the ControlTask to monitor
         * @param sample_rate_ms Data collection rate in milliseconds
         * @param enabled Whether data collection is enabled
         */
        ControlTaskDataSource(control::ControlTask *control_task,
                              uint32_t sample_rate_ms = 200,
                              bool enabled = true);

        virtual ~ControlTaskDataSource() = default;

        // IDataSource interface implementation
        DataSourceConfig getSourceConfig() const override;
        esp_err_t collectData(std::vector<DataEntry> &entries) override;
        bool isReady() const override;
        esp_err_t onRegistered() override;
        esp_err_t onUnregistered() override;

        /**
         * @brief Set whether to capture detailed physics data
         * @param enable True to include advanced physics calculations
         */
        void setCapturePhysicsData(bool enable) { capture_physics_data_ = enable; }

        /**
         * @brief Set minimum speed threshold for data collection
         * @param threshold Minimum speed (0.0-1.0) to start collecting
         */
        void setSpeedThreshold(float threshold) { speed_threshold_ = threshold; }

        /**
         * @brief Get statistics about data collection
         * @param total_samples Total samples collected
         * @param physics_samples Physics samples collected
         * @param brake_events Number of brake events detected
         * @param warning_events Number of warning events detected
         */
        void getStatistics(uint32_t &total_samples, uint32_t &physics_samples,
                           uint32_t &brake_events, uint32_t &warning_events) const;

    private:
        static const char *TAG;

        DataSourceConfig config_;
        control::ControlTask *control_task_;

        // Configuration
        bool capture_physics_data_ = true;
        float speed_threshold_ = 0.1f; // Minimum speed to collect data

        // Statistics
        uint32_t total_samples_ = 0;
        uint32_t physics_samples_ = 0;
        uint32_t brake_events_ = 0;
        uint32_t warning_events_ = 0;

        // State tracking for event detection
        bool last_obstacle_state_ = false;
        bool last_warning_state_ = false;
        float last_distance_ = 0.0f;
        float last_speed_ = 0.0f;

        /**
         * @brief Access control task's private data safely
         * This is a friend-like access pattern for data extraction
         */
        struct ControlDataSnapshot
        {
            // RC Input data
            float current_input = 0.0f;
            float cached_duty = 0.0f;
            float direct_duty = 0.0f;
            bool throttle_pressed = false;
            bool driving_forward = false;
            bool wants_reverse = false;
            bool cached_throttle = false;

            // LiDAR data
            float distance = 0.0f;
            bool obstacle_detected = false;
            bool warning_active = false;

            // Dynamic calculations
            float brake_distance = 0.0f;
            float warning_distance = 0.0f;

            // Control states
            bool is_obstacle_state = false;
            bool is_warning_state = false;

            // Physics data
            float speed_delta = 0.0f;    // Speed change from last sample
            float distance_delta = 0.0f; // Distance change from last sample
            float deceleration = 0.0f;   // Calculated deceleration
            float time_to_impact = 0.0f; // Estimated time to collision
        };

        /**
         * @brief Capture current control system state
         * @return Snapshot of current control data
         */
        ControlDataSnapshot captureControlState();

        /**
         * @brief Calculate physics analysis data
         * @param snapshot Current control snapshot
         * @param timestamp_us Current timestamp
         */
        void calculatePhysicsData(ControlDataSnapshot &snapshot, uint64_t timestamp_us);

        /**
         * @brief Detect and count control events
         * @param snapshot Current control snapshot
         */
        void updateEventCounters(const ControlDataSnapshot &snapshot);

        /**
         * @brief Add basic control data entries
         * @param entries Vector to add entries to
         * @param snapshot Control data snapshot
         * @param timestamp Current timestamp
         */
        void addBasicControlData(std::vector<DataEntry> &entries,
                                 const ControlDataSnapshot &snapshot,
                                 uint64_t timestamp);

        /**
         * @brief Add physics analysis data entries
         * @param entries Vector to add entries to
         * @param snapshot Control data snapshot
         * @param timestamp Current timestamp
         */
        void addPhysicsData(std::vector<DataEntry> &entries,
                            const ControlDataSnapshot &snapshot,
                            uint64_t timestamp);
    };

} // namespace digitoys::datalogger
