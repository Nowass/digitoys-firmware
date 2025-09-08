#pragma once

#include "ComponentBase.hpp"
#include "IDataSource.hpp"
#include <esp_timer.h>
#include <vector>
#include <memory>
#include <map>

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
        bool monitoring_mode = true;       ///< If true, use circular buffer for dashboard monitoring
        size_t monitoring_buffer_size = 20; ///< Number of entries to keep in monitoring mode
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
         * @brief Get number of logged entries (available for export)
         * @return Number of logged entries
         */
        size_t getEntryCount() const;

        /**
         * @brief Get total entry count across all storages
         * @return Total number of entries
         */
        size_t getTotalEntryCount() const;

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

        /**
         * @brief Register a data source for logging
         * @param source Shared pointer to the data source
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t registerDataSource(DataSourcePtr source);

        /**
         * @brief Unregister a data source
         * @param source_name Name of the source to unregister
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t unregisterDataSource(const std::string &source_name);

        /**
         * @brief Get list of registered data sources
         * @return Vector of source names
         */
        std::vector<std::string> getRegisteredSources() const;

        /**
         * @brief Collect data from all registered sources
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t collectFromSources();

        /**
         * @brief Enable/disable monitoring mode
         * @param enable If true, switch to circular buffer monitoring mode
         * @return ESP_OK on success
         */
        esp_err_t setMonitoringMode(bool enable);

        /**
         * @brief Check if currently in monitoring mode
         * @return true if in monitoring mode, false if in full logging mode
         */
        bool isMonitoringMode() const { return current_monitoring_mode_; }

        /**
         * @brief Get a copy of collected data for analysis
         * @param max_entries Maximum number of recent entries to return (0 = all)
         * @return Vector of recent data entries
         */
        std::vector<DataEntry> getCollectedData(size_t max_entries = 0) const;

        /**
         * @brief Get logged data from persistent storage (for export)
         * @param max_entries Maximum number of entries to return (0 for all)
         * @return Vector of logged data entries
         */
        std::vector<DataEntry> getLoggedData(size_t max_entries = 0) const;

        /**
         * @brief Get monitoring data from circular buffer (for telemetry)
         * @param max_entries Maximum number of entries to return (0 for all)
         * @return Vector of monitoring data entries
         */
        std::vector<DataEntry> getMonitoringData(size_t max_entries = 0) const;

    private:
        static const char *TAG;

        DataLoggerConfig config_;
        esp_timer_handle_t flush_timer_ = nullptr;

        // Memory tracking
        size_t current_memory_usage_ = 0;
        size_t entry_count_ = 0;

        // Data source management
        std::map<std::string, DataSourcePtr> data_sources_;
        
        // Separate data storage for different modes
        std::vector<DataEntry> logged_data_;        // Persistent data from logging sessions
        std::vector<DataEntry> monitoring_data_;    // Circular buffer for monitoring
        std::vector<DataEntry> collected_data_;     // Legacy compatibility (points to active storage)
        
        esp_timer_handle_t collection_timer_ = nullptr;

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

        /**
         * @brief Timer callback for data collection
         */
        static void collectionTimerCallback(void *arg);

        /**
         * @brief Internal data collection from sources
         */
        esp_err_t doDataCollection();

        /**
         * @brief Calculate memory usage for a data entry
         */
        size_t calculateEntrySize(const DataEntry &entry) const;

        /**
         * @brief Add entries for monitoring mode (circular buffer)
         */
        void addEntriesForMonitoring(const std::vector<DataEntry> &entries, const std::string &source_name);

        /**
         * @brief Add entries for logging mode (memory limit checking)
         */
        void addEntriesForLogging(const std::vector<DataEntry> &entries, const std::string &source_name);

        // Current operating mode
        bool current_monitoring_mode_; ///< Current monitoring mode state
    };

} // namespace digitoys::datalogger
