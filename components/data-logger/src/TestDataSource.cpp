#include "TestDataSource.hpp"
#include <esp_log.h>
#include <cmath>
#include <sstream>

namespace digitoys::datalogger
{
    const char *TestDataSource::TAG = "TestDataSource";

    TestDataSource::TestDataSource(const std::string &source_name,
                                   uint32_t sample_rate_ms,
                                   bool enabled)
        : rng_(esp_timer_get_time()) // Seed with current time
          ,
          noise_dist_(-0.1f, 0.1f) // ±10% noise
          ,
          failure_dist_(1, 100) // 1-100 for percentage chances
    {
        config_.source_name = source_name;
        config_.sample_rate_ms = sample_rate_ms;
        config_.enabled = enabled;
        config_.max_buffer_size = 20; // Keep reasonable buffer

        // Initialize random phase offset for more realistic data
        std::uniform_real_distribution<float> phase_dist(0.0f, 2.0f * M_PI);
        phase_offset_ = phase_dist(rng_);

        ESP_LOGI(TAG, "TestDataSource '%s' created (rate: %lu ms, enabled: %s)",
                 source_name.c_str(), sample_rate_ms, enabled ? "true" : "false");
    }

    DataSourceConfig TestDataSource::getSourceConfig() const
    {
        return config_;
    }

    esp_err_t TestDataSource::collectData(std::vector<DataEntry> &entries)
    {
        if (!is_ready_ || !config_.enabled)
        {
            return ESP_ERR_INVALID_STATE;
        }

        // Simulate occasional collection failures
        if (shouldSimulateFailure())
        {
            collection_failures_++;
            ESP_LOGW(TAG, "Simulating collection failure (total: %lu)", collection_failures_);
            return ESP_ERR_TIMEOUT; // Simulate sensor timeout
        }

        uint64_t timestamp = getCurrentTimestamp();

        // Generate various types of sensor data
        float speed = generateSpeed(timestamp);
        float distance = generateDistance(timestamp);
        float temperature = generateTemperature(timestamp);
        std::string status = generateSystemStatus(timestamp);

        // Create data entries using convenience constructors
        entries.emplace_back("speed_kmh", speed, timestamp);
        entries.emplace_back("distance_cm", distance, timestamp);
        entries.emplace_back("temperature_c", temperature, timestamp);
        entries.emplace_back("system_status", status, timestamp);

        // Add some integer data
        entries.emplace_back("data_points", static_cast<uint32_t>(data_points_generated_), timestamp);
        entries.emplace_back("collection_failures", static_cast<uint32_t>(collection_failures_), timestamp);

        // Add boolean status
        bool sensor_healthy = (collection_failures_ < 5);
        entries.emplace_back("sensor_healthy", sensor_healthy, timestamp);

        data_points_generated_++;

        ESP_LOGD(TAG, "Generated %zu data entries (cycle: %lu, speed: %.1f km/h, distance: %.0f cm)",
                 entries.size(), data_points_generated_, speed, distance);

        return ESP_OK;
    }

    bool TestDataSource::isReady() const
    {
        return is_ready_ && config_.enabled;
    }

    esp_err_t TestDataSource::onRegistered()
    {
        start_time_us_ = getCurrentTimestamp();
        is_ready_ = true;

        ESP_LOGI(TAG, "TestDataSource '%s' registered and ready", config_.source_name.c_str());
        return ESP_OK;
    }

    esp_err_t TestDataSource::onUnregistered()
    {
        is_ready_ = false;

        ESP_LOGI(TAG, "TestDataSource '%s' unregistered (generated %lu data points, %lu failures)",
                 config_.source_name.c_str(), data_points_generated_, collection_failures_);
        return ESP_OK;
    }

    float TestDataSource::generateSpeed(uint64_t timestamp_us)
    {
        float elapsed_seconds = (timestamp_us - start_time_us_) / 1000000.0f;
        float base_speed = 0.0f;

        switch (data_pattern_)
        {
        case DataPattern::STEADY:
            base_speed = 30.0f; // Steady 30 km/h
            break;

        case DataPattern::SINUSOIDAL:
            // Sine wave between 10-50 km/h with 20 second period
            base_speed = 30.0f + 20.0f * sin(2.0f * M_PI * elapsed_seconds / 20.0f + phase_offset_);
            break;

        case DataPattern::RANDOM:
            // Random walk with bounds
            last_speed_ += (noise_dist_(rng_) * 5.0f);                  // ±5 km/h change
            last_speed_ = std::max(0.0f, std::min(80.0f, last_speed_)); // Clamp 0-80 km/h
            base_speed = last_speed_;
            break;

        case DataPattern::SPIKES:
            base_speed = 25.0f;
            // Random spikes every ~10 seconds
            if (failure_dist_(rng_) <= 3)
            {                        // 3% chance
                base_speed += 40.0f; // Speed spike
            }
            break;
        }

        // Add some realistic noise
        float noise = base_speed * noise_dist_(rng_);
        return std::max(0.0f, base_speed + noise);
    }

    float TestDataSource::generateDistance(uint64_t timestamp_us)
    {
        float elapsed_seconds = (timestamp_us - start_time_us_) / 1000000.0f;
        float base_distance = 0.0f;

        switch (data_pattern_)
        {
        case DataPattern::STEADY:
            base_distance = 150.0f; // Steady 150 cm
            break;

        case DataPattern::SINUSOIDAL:
            // Sine wave between 50-250 cm with 15 second period
            base_distance = 150.0f + 100.0f * sin(2.0f * M_PI * elapsed_seconds / 15.0f + phase_offset_);
            break;

        case DataPattern::RANDOM:
            // Random walk with bounds
            last_distance_ += (noise_dist_(rng_) * 20.0f);                      // ±20 cm change
            last_distance_ = std::max(10.0f, std::min(500.0f, last_distance_)); // Clamp 10-500 cm
            base_distance = last_distance_;
            break;

        case DataPattern::SPIKES:
            base_distance = 180.0f;
            // Sudden close obstacles
            if (failure_dist_(rng_) <= 2)
            {                          // 2% chance
                base_distance = 30.0f; // Sudden obstacle
            }
            break;
        }

        // Add noise
        float noise = base_distance * noise_dist_(rng_);
        return std::max(5.0f, base_distance + noise); // Minimum 5 cm
    }

    float TestDataSource::generateTemperature(uint64_t timestamp_us)
    {
        float elapsed_seconds = (timestamp_us - start_time_us_) / 1000000.0f;

        // Slow temperature variation between 20-35°C with 60 second period
        float base_temp = 27.5f + 7.5f * sin(2.0f * M_PI * elapsed_seconds / 60.0f + phase_offset_);

        // Add small noise
        float noise = base_temp * noise_dist_(rng_) * 0.5f; // Smaller noise for temperature
        return base_temp + noise;
    }

    std::string TestDataSource::generateSystemStatus(uint64_t timestamp_us)
    {
        // Cycle through different status messages
        uint32_t cycle = (timestamp_us - start_time_us_) / (config_.sample_rate_ms * 1000);

        const std::vector<std::string> statuses = {
            "NORMAL",
            "MONITORING",
            "ACTIVE",
            "CALIBRATING",
            "READY"};

        if (shouldSimulateFailure() && failure_dist_(rng_) <= 1)
        { // 1% chance
            return "ERROR";
        }

        return statuses[cycle % statuses.size()];
    }

    bool TestDataSource::shouldSimulateFailure() const
    {
        if (!simulate_failures_)
        {
            return false;
        }

        // 2% chance of failure when enabled
        return failure_dist_(rng_) <= 2;
    }

} // namespace digitoys::datalogger
