#pragma once

#include "DataLogger.hpp"
#include "TestDataSource.hpp"
#include <esp_err.h>
#include <memory>

// Forward declarations to avoid heavy includes
namespace control
{
    class ControlTask;
}

namespace digitoys::datalogger
{
    // Forward declaration for ControlTaskDataSource
    class ControlTaskDataSource;

    /**
     * @brief Advanced demo showing DataLogger with both test and real control data
     *
     * This demonstrates integration of multiple data sources:
     * 1. TestDataSource for synthetic sensor data
     * 2. ControlTaskDataSource for real physics analysis
     * 3. Combined logging and analysis workflow
     */
    class AdvancedDataLoggerDemo
    {
    public:
        /**
         * @brief Initialize the demo with test and control data sources
         * @param control_task Pointer to ControlTask for real data (optional)
         * @return ESP_OK on success
         */
        static esp_err_t initialize(control::ControlTask *control_task = nullptr);

        /**
         * @brief Start the advanced data logging demo
         * @return ESP_OK on success
         */
        static esp_err_t start();

        /**
         * @brief Stop the demo
         * @return ESP_OK on success
         */
        static esp_err_t stop();

        /**
         * @brief Get comprehensive demo statistics
         * @param logger_entries Output parameter for total logger entries
         * @param test_points Output parameter for test source data points
         * @param control_samples Output parameter for control system samples
         * @param physics_samples Output parameter for physics analysis samples
         * @param brake_events Output parameter for brake events detected
         */
        static void getStatistics(size_t &logger_entries,
                                  uint32_t &test_points,
                                  uint32_t &control_samples,
                                  uint32_t &physics_samples,
                                  uint32_t &brake_events);

        /**
         * @brief Print comprehensive demo status
         */
        static void printStatus();

        /**
         * Generate comprehensive physics analysis report
         * Logs detailed braking performance metrics
         */
        static void generatePhysicsReport();

    private:
        static std::unique_ptr<DataLogger> logger_;
        static std::shared_ptr<TestDataSource> test_source_;
        static std::shared_ptr<ControlTaskDataSource> control_source_;
        static bool is_initialized_;
        static bool has_real_control_task_;
    };

} // namespace digitoys::datalogger
