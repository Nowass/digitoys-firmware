#include "ControlTaskDataSource.hpp"
#include "ControlTask.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <cmath>

namespace digitoys::datalogger
{
    const char *ControlTaskDataSource::TAG = "ControlTaskDataSource";

    ControlTaskDataSource::ControlTaskDataSource(control::ControlTask *control_task,
                                                 uint32_t sample_rate_ms,
                                                 bool enabled)
        : control_task_(control_task)
    {
        config_.source_name = "ControlSystem";
        config_.sample_rate_ms = sample_rate_ms;
        config_.enabled = enabled;
        config_.max_buffer_size = 30; // Higher for detailed control data

        ESP_LOGI(TAG, "ControlTaskDataSource created (rate: %lu ms, physics: %s)",
                 sample_rate_ms, capture_physics_data_ ? "enabled" : "disabled");
    }

    DataSourceConfig ControlTaskDataSource::getSourceConfig() const
    {
        return config_;
    }

    esp_err_t ControlTaskDataSource::collectData(std::vector<DataEntry> &entries)
    {
        if (!isReady())
        {
            return ESP_ERR_INVALID_STATE;
        }

        // Capture current control system state
        ControlDataSnapshot snapshot = captureControlState();
        uint64_t timestamp = getCurrentTimestamp();

        // Check speed threshold - only collect data when moving or braking
        if (snapshot.current_input < speed_threshold_ &&
            !snapshot.is_obstacle_state &&
            !snapshot.is_warning_state)
        {
            ESP_LOGV(TAG, "Below speed threshold, skipping collection");
            return ESP_OK;
        }

        // Calculate physics data if enabled
        if (capture_physics_data_)
        {
            calculatePhysicsData(snapshot, timestamp);
            physics_samples_++;
        }

        // Update event counters
        updateEventCounters(snapshot);

        // Add data entries
        addBasicControlData(entries, snapshot, timestamp);

        if (capture_physics_data_)
        {
            addPhysicsData(entries, snapshot, timestamp);
        }

        total_samples_++;

        ESP_LOGD(TAG, "Collected control data: speed=%.3f, distance=%.1fcm, brake_dist=%.1fcm, obstacle=%s",
                 snapshot.current_input, snapshot.distance, snapshot.brake_distance,
                 snapshot.is_obstacle_state ? "YES" : "NO");

        return ESP_OK;
    }

    bool ControlTaskDataSource::isReady() const
    {
        return control_task_ != nullptr &&
               config_.enabled &&
               control_task_->isRunning();
    }

    esp_err_t ControlTaskDataSource::onRegistered()
    {
        ESP_LOGI(TAG, "ControlTaskDataSource registered for physics analysis");

        // Reset statistics
        total_samples_ = 0;
        physics_samples_ = 0;
        brake_events_ = 0;
        warning_events_ = 0;

        return ESP_OK;
    }

    esp_err_t ControlTaskDataSource::onUnregistered()
    {
        ESP_LOGI(TAG, "ControlTaskDataSource unregistered (samples: %lu, physics: %lu, brakes: %lu, warnings: %lu)",
                 total_samples_, physics_samples_, brake_events_, warning_events_);
        return ESP_OK;
    }

    void ControlTaskDataSource::getStatistics(uint32_t &total_samples, uint32_t &physics_samples,
                                              uint32_t &brake_events, uint32_t &warning_events) const
    {
        total_samples = total_samples_;
        physics_samples = physics_samples_;
        brake_events = brake_events_;
        warning_events = warning_events_;
    }

    ControlTaskDataSource::ControlDataSnapshot ControlTaskDataSource::captureControlState()
    {
        ControlDataSnapshot snapshot;

        // NOTE: This is a simplified version that captures publicly available data.
        // In a real implementation, we would need access to ControlTask's internal state.
        // For now, we'll simulate the data that would be available.

        // This is where we would access ControlTask's processFrame data:
        // - lidar_info from ctx_->lidar->getObstacleInfo()
        // - rc_status from rc_processor_.processRCInput()
        // - dynamic calculations from obstacle_detector_
        // - state information from state_

        // For demonstration, we'll use placeholder logic that would be replaced
        // with actual ControlTask integration

        static uint32_t call_count = 0;
        call_count++;

        // Simulate realistic control data patterns
        float time_factor = (esp_timer_get_time() / 1000000.0f) / 10.0f; // 10 second cycles

        // Simulate RC input (varying speed)
        snapshot.current_input = 0.3f + 0.2f * sin(time_factor);
        snapshot.current_input = std::max(0.0f, std::min(1.0f, snapshot.current_input));

        // Simulate control states
        snapshot.throttle_pressed = snapshot.current_input > 0.1f;
        snapshot.driving_forward = snapshot.throttle_pressed && snapshot.current_input > 0.15f;
        snapshot.wants_reverse = false;

        // Simulate LiDAR data (varying obstacle distance)
        snapshot.distance = 200.0f + 100.0f * sin(time_factor * 0.7f);
        snapshot.distance = std::max(20.0f, snapshot.distance);

        // Simulate dynamic braking calculations
        snapshot.brake_distance = 50.0f + snapshot.current_input * 80.0f; // Speed-dependent
        snapshot.warning_distance = snapshot.brake_distance * 1.5f;

        // Determine states based on distance vs thresholds
        snapshot.obstacle_detected = snapshot.distance < snapshot.brake_distance;
        snapshot.warning_active = snapshot.distance < snapshot.warning_distance;
        snapshot.is_obstacle_state = snapshot.obstacle_detected && snapshot.driving_forward;
        snapshot.is_warning_state = snapshot.warning_active && snapshot.driving_forward && !snapshot.is_obstacle_state;

        // Cache duty values (in real implementation, these would come from PWM driver)
        snapshot.cached_duty = snapshot.current_input;
        snapshot.direct_duty = snapshot.current_input + (call_count % 10) * 0.001f; // Small variance
        snapshot.cached_throttle = snapshot.throttle_pressed;

        return snapshot;
    }

    void ControlTaskDataSource::calculatePhysicsData(ControlDataSnapshot &snapshot, uint64_t timestamp_us)
    {
        static uint64_t last_timestamp = 0;

        if (last_timestamp == 0)
        {
            last_timestamp = timestamp_us;
            last_speed_ = snapshot.current_input;
            last_distance_ = snapshot.distance;
            return; // Need previous data for deltas
        }

        float dt_seconds = (timestamp_us - last_timestamp) / 1000000.0f;

        if (dt_seconds > 0.0f)
        {
            // Calculate deltas
            snapshot.speed_delta = snapshot.current_input - last_speed_;
            snapshot.distance_delta = snapshot.distance - last_distance_;

            // Calculate deceleration (negative acceleration)
            if (abs(snapshot.speed_delta) > 0.001f)
            {
                // Convert speed (0-1 scale) to m/s (approximate)
                float speed_ms = snapshot.current_input * 20.0f; // Assume max 20 m/s
                float last_speed_ms = last_speed_ * 20.0f;
                snapshot.deceleration = (speed_ms - last_speed_ms) / dt_seconds;
            }

            // Calculate time to impact (if approaching obstacle)
            if (snapshot.driving_forward && snapshot.distance_delta < 0.0f)
            {
                float approach_speed = -snapshot.distance_delta / dt_seconds; // cm/s
                if (approach_speed > 0.1f)
                {                                                                 // Minimum approach speed
                    snapshot.time_to_impact = snapshot.distance / approach_speed; // seconds
                }
            }
        }

        // Update tracking variables
        last_timestamp = timestamp_us;
        last_speed_ = snapshot.current_input;
        last_distance_ = snapshot.distance;
    }

    void ControlTaskDataSource::updateEventCounters(const ControlDataSnapshot &snapshot)
    {
        // Count brake events (transitions to obstacle state)
        if (snapshot.is_obstacle_state && !last_obstacle_state_)
        {
            brake_events_++;
            ESP_LOGI(TAG, "Brake event detected (#%lu) - distance: %.1fcm, speed: %.3f",
                     brake_events_, snapshot.distance, snapshot.current_input);
        }

        // Count warning events (transitions to warning state)
        if (snapshot.is_warning_state && !last_warning_state_)
        {
            warning_events_++;
            ESP_LOGD(TAG, "Warning event detected (#%lu) - distance: %.1fcm, speed: %.3f",
                     warning_events_, snapshot.distance, snapshot.current_input);
        }

        // Update state tracking
        last_obstacle_state_ = snapshot.is_obstacle_state;
        last_warning_state_ = snapshot.is_warning_state;
    }

    void ControlTaskDataSource::addBasicControlData(std::vector<DataEntry> &entries,
                                                    const ControlDataSnapshot &snapshot,
                                                    uint64_t timestamp)
    {
        // RC Input data
        entries.emplace_back("rc_input", snapshot.current_input, timestamp);
        entries.emplace_back("cached_duty", snapshot.cached_duty, timestamp);
        entries.emplace_back("direct_duty", snapshot.direct_duty, timestamp);
        entries.emplace_back("throttle_pressed", snapshot.throttle_pressed, timestamp);
        entries.emplace_back("driving_forward", snapshot.driving_forward, timestamp);
        entries.emplace_back("wants_reverse", snapshot.wants_reverse, timestamp);

        // LiDAR data
        entries.emplace_back("obstacle_distance", snapshot.distance, timestamp);
        entries.emplace_back("obstacle_detected", snapshot.obstacle_detected, timestamp);
        entries.emplace_back("warning_active", snapshot.warning_active, timestamp);

        // Dynamic calculations
        entries.emplace_back("brake_distance", snapshot.brake_distance, timestamp);
        entries.emplace_back("warning_distance", snapshot.warning_distance, timestamp);

        // Control states
        entries.emplace_back("is_obstacle_state", snapshot.is_obstacle_state, timestamp);
        entries.emplace_back("is_warning_state", snapshot.is_warning_state, timestamp);

        // Statistics
        entries.emplace_back("total_samples", total_samples_, timestamp);
        entries.emplace_back("brake_events", brake_events_, timestamp);
        entries.emplace_back("warning_events", warning_events_, timestamp);
    }

    void ControlTaskDataSource::addPhysicsData(std::vector<DataEntry> &entries,
                                               const ControlDataSnapshot &snapshot,
                                               uint64_t timestamp)
    {
        // Physics analysis data
        entries.emplace_back("speed_delta", snapshot.speed_delta, timestamp);
        entries.emplace_back("distance_delta", snapshot.distance_delta, timestamp);
        entries.emplace_back("deceleration", snapshot.deceleration, timestamp);
        entries.emplace_back("time_to_impact", snapshot.time_to_impact, timestamp);

        // Physics statistics
        entries.emplace_back("physics_samples", physics_samples_, timestamp);

        // Safety margins
        float safety_margin = snapshot.distance - snapshot.brake_distance;
        float warning_margin = snapshot.distance - snapshot.warning_distance;
        entries.emplace_back("safety_margin", safety_margin, timestamp);
        entries.emplace_back("warning_margin", warning_margin, timestamp);
    }

} // namespace digitoys::datalogger
