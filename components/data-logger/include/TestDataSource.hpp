#pragma once

#include "IDataSource.hpp"
#include <esp_timer.h>
#include <random>

namespace digitoys::datalogger
{
    /**
     * @brief Test data source that generates synthetic telemetry data
     *
     * This source generates realistic sensor data patterns for testing
     * the data logger collection system. It simulates various types of
     * telemetry that might come from real vehicle sensors.
     */
    class TestDataSource : public IDataSource
    {
    public:
        /**
         * @brief Constructor
         * @param source_name Name of this test source
         * @param sample_rate_ms Data generation rate in milliseconds
         * @param enabled Whether the source is enabled
         */
        TestDataSource(const std::string &source_name = "TestSource",
                       uint32_t sample_rate_ms = 250,
                       bool enabled = true);

        virtual ~TestDataSource() = default;

        // IDataSource interface implementation
        DataSourceConfig getSourceConfig() const override;
        esp_err_t collectData(std::vector<DataEntry> &entries) override;
        bool isReady() const override;
        esp_err_t onRegistered() override;
        esp_err_t onUnregistered() override;

        /**
         * @brief Set whether to simulate sensor failures
         * @param enable True to enable random failures
         */
        void setSimulateFailures(bool enable) { simulate_failures_ = enable; }

        /**
         * @brief Set the data generation pattern
         * @param pattern Data pattern type
         */
        enum class DataPattern
        {
            STEADY,     ///< Steady values with small noise
            SINUSOIDAL, ///< Sine wave patterns
            RANDOM,     ///< Random walk patterns
            SPIKES      ///< Occasional spike patterns
        };
        void setDataPattern(DataPattern pattern) { data_pattern_ = pattern; }

        /**
         * @brief Get total data points generated
         * @return Number of data collection cycles
         */
        uint32_t getDataPointsGenerated() const { return data_points_generated_; }

    private:
        static const char *TAG;

        DataSourceConfig config_;
        DataPattern data_pattern_ = DataPattern::SINUSOIDAL;
        bool simulate_failures_ = false;
        bool is_ready_ = false;

        // Statistics
        uint32_t data_points_generated_ = 0;
        uint32_t collection_failures_ = 0;

        // Simulation state
        uint64_t start_time_us_ = 0;
        float phase_offset_ = 0.0f;
        float last_speed_ = 25.0f;     // Last speed value for random walk
        float last_distance_ = 100.0f; // Last distance for random walk

        // Random number generation
        mutable std::mt19937 rng_;
        mutable std::uniform_real_distribution<float> noise_dist_;
        mutable std::uniform_int_distribution<int> failure_dist_;

        /**
         * @brief Generate speed data based on pattern
         * @param timestamp_us Current timestamp
         * @return Speed value in km/h
         */
        float generateSpeed(uint64_t timestamp_us);

        /**
         * @brief Generate distance data based on pattern
         * @param timestamp_us Current timestamp
         * @return Distance value in centimeters
         */
        float generateDistance(uint64_t timestamp_us);

        /**
         * @brief Generate temperature data
         * @param timestamp_us Current timestamp
         * @return Temperature in Celsius
         */
        float generateTemperature(uint64_t timestamp_us);

        /**
         * @brief Generate system status
         * @param timestamp_us Current timestamp
         * @return System status string
         */
        std::string generateSystemStatus(uint64_t timestamp_us);

        /**
         * @brief Check if we should simulate a failure this cycle
         * @return True if failure should be simulated
         */
        bool shouldSimulateFailure() const;
    };

} // namespace digitoys::datalogger
