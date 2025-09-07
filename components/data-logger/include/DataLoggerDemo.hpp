#pragma once

#include "DataLogger.hpp"
#include "TestDataSource.hpp"
#include <memory>

namespace digitoys::datalogger
{
    /**
     * @brief Demo class showing how to use DataLogger with TestDataSource
     *
     * This demonstrates the complete data logging workflow:
     * 1. Create and configure DataLogger
     * 2. Create test data sources
     * 3. Register sources with logger
     * 4. Monitor collection process
     */
    class DataLoggerDemo
    {
    public:
        /**
         * @brief Initialize the demo with test data sources
         * @return ESP_OK on success
         */
        static esp_err_t initialize();

        /**
         * @brief Start the data logging demo
         * @return ESP_OK on success
         */
        static esp_err_t start();

        /**
         * @brief Stop the demo
         * @return ESP_OK on success
         */
        static esp_err_t stop();

        /**
         * @brief Get demo statistics
         * @param logger_entries Output parameter for logger entry count
         * @param source1_points Output parameter for source1 data points
         * @param source2_points Output parameter for source2 data points
         */
        static void getStatistics(size_t &logger_entries,
                                  uint32_t &source1_points,
                                  uint32_t &source2_points);

        /**
         * @brief Print current demo status to console
         */
        static void printStatus();

    private:
        static std::unique_ptr<DataLogger> logger_;
        static std::shared_ptr<TestDataSource> test_source1_;
        static std::shared_ptr<TestDataSource> test_source2_;
        static bool is_initialized_;
    };

} // namespace digitoys::datalogger
