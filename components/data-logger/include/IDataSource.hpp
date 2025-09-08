#pragma once

#include <esp_err.h>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>

namespace digitoys::datalogger
{
    /**
     * @brief Data types supported by the logging system
     */
    enum class DataType
    {
        INT32,  ///< 32-bit signed integer
        UINT32, ///< 32-bit unsigned integer
        FLOAT,  ///< 32-bit floating point
        DOUBLE, ///< 64-bit floating point
        STRING, ///< String data
        BOOLEAN ///< Boolean value
    };

    /**
     * @brief Generic data entry for logging
     */
    struct DataEntry
    {
        std::string key;       ///< Data field name/identifier
        DataType type;         ///< Type of the data
        std::string value;     ///< String representation of the value
        uint64_t timestamp_us; ///< Timestamp in microseconds

        // Convenience constructors
        DataEntry(const std::string &k, int32_t v, uint64_t ts);
        DataEntry(const std::string &k, uint32_t v, uint64_t ts);
        DataEntry(const std::string &k, float v, uint64_t ts);
        DataEntry(const std::string &k, double v, uint64_t ts);
        DataEntry(const std::string &k, const std::string &v, uint64_t ts);
        DataEntry(const std::string &k, bool v, uint64_t ts);
    };

    /**
     * @brief Configuration for data sources
     */
    struct DataSourceConfig
    {
        std::string source_name;       ///< Name of the data source
        uint32_t sample_rate_ms = 100; ///< Data sampling rate in milliseconds
        bool enabled = true;           ///< Enable/disable this source
        uint32_t max_buffer_size = 50; ///< Maximum buffered entries per source
    };

    /**
     * @brief Interface for components that can provide data to the logger
     *
     * Any component wanting to contribute data to the logging system should
     * implement this interface. The DataLogger will call collectData()
     * periodically based on the configured sample rate.
     */
    class IDataSource
    {
    public:
        virtual ~IDataSource() = default;

        /**
         * @brief Get the data source configuration
         * @return DataSourceConfig for this source
         */
        virtual DataSourceConfig getSourceConfig() const = 0;

        /**
         * @brief Collect current data from this source
         * @param entries Vector to populate with current data entries
         * @return ESP_OK on success, error code on failure
         */
        virtual esp_err_t collectData(std::vector<DataEntry> &entries) = 0;

        /**
         * @brief Check if the data source is ready to provide data
         * @return true if ready, false otherwise
         */
        virtual bool isReady() const = 0;

        /**
         * @brief Called when the data source is registered with the logger
         * @return ESP_OK on success, error code on failure
         */
        virtual esp_err_t onRegistered() { return ESP_OK; }

        /**
         * @brief Called when the data source is unregistered from the logger
         * @return ESP_OK on success, error code on failure
         */
        virtual esp_err_t onUnregistered() { return ESP_OK; }

        /**
         * @brief Called when logging mode changes (monitoring vs full logging)
         * @param is_monitoring true for monitoring mode, false for full logging
         */
        virtual void onModeChanged(bool is_monitoring) { (void)is_monitoring; /* Default: do nothing */ }
    };

    /**
     * @brief Smart pointer type for data sources
     */
    using DataSourcePtr = std::shared_ptr<IDataSource>;

    /**
     * @brief Convert DataType to string for serialization
     * @param type DataType to convert
     * @return String representation
     */
    const char *dataTypeToString(DataType type);

    /**
     * @brief Get current timestamp in microseconds
     * @return Current timestamp
     */
    uint64_t getCurrentTimestamp();

} // namespace digitoys::datalogger
