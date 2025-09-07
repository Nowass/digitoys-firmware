#pragma once

#include "ComponentBase.hpp"
#include <esp_timer.h>
#include <vector>
#include <memory>

namespace digitoys::datalogger
{
    /**
     * @brief Configuration for data logger
     */
    struct DataLoggerConfig
    {
        bool enabled = false;              ///< Enable/disable logging
        uint32_t max_entries = 1000;       ///< Maximum number of entries
        uint32_t flush_interval_ms = 5000; ///< Auto-flush interval
        size_t max_memory_kb = 64;         ///< Maximum memory usage in KB
    };

    /**
     * @brief Generic data logger component for development data collection
     *
     * Provides a framework for collecting and managing development data from
     * various sources across the system. Only active in development builds
     * to avoid impacting production performance.
     */
    class DataLogger : public digitoys::core::ComponentBase
    {
    public:
        explicit DataLogger(const DataLoggerConfig &config = {});
        virtual ~DataLogger();

        // ComponentBase interface implementation
        esp_err_t initialize() override;
        esp_err_t start() override;
        esp_err_t stop() override;
        esp_err_t shutdown() override;

        /**
         * @brief Get current configuration
         * @return Current DataLoggerConfig
         */
        const DataLoggerConfig &getConfig() const { return config_; }

        /**
         * @brief Check if logging is enabled
         * @return true if enabled, false otherwise
         */
        bool isEnabled() const { return config_.enabled && isRunning(); }

        /**
         * @brief Get memory usage statistics
         * @return Memory usage in bytes
         */
        size_t getMemoryUsage() const;

        /**
         * @brief Get number of logged entries
         * @return Number of entries
         */
        size_t getEntryCount() const;

        /**
         * @brief Force flush all pending data
         * @return ESP_OK on success
         */
        esp_err_t flush();

        /**
         * @brief Clear all logged data
         * @return ESP_OK on success
         */
        esp_err_t clear();

    private:
        static const char *TAG;

        DataLoggerConfig config_;
        esp_timer_handle_t flush_timer_ = nullptr;

        // Memory tracking
        size_t current_memory_usage_ = 0;
        size_t entry_count_ = 0;

        /**
         * @brief Timer callback for auto-flush
         */
        static void flushTimerCallback(void *arg);

        /**
         * @brief Internal flush implementation
         */
        esp_err_t doFlush();

        /**
         * @brief Check memory limits
         */
        bool checkMemoryLimits() const;
    };

} // namespace digitoys::datalogger
