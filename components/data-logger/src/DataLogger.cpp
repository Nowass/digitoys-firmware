#include "DataLogger.hpp"
#include <esp_log.h>
#include <cstring>

namespace digitoys::datalogger
{
    const char *DataLogger::TAG = "DataLogger";

    DataLogger::DataLogger(const DataLoggerConfig &config)
        : ComponentBase("DataLogger"), config_(config)
    {
        ESP_LOGI(TAG, "DataLogger created (enabled: %s, max_entries: %lu, max_memory: %zu KB)",
                 config_.enabled ? "true" : "false",
                 config_.max_entries,
                 config_.max_memory_kb);
    }

    DataLogger::~DataLogger()
    {
        if (flush_timer_)
        {
            esp_timer_delete(flush_timer_);
            flush_timer_ = nullptr;
        }
        ESP_LOGD(TAG, "DataLogger destroyed");
    }

    esp_err_t DataLogger::initialize()
    {
        if (getState() != digitoys::core::ComponentState::UNINITIALIZED)
        {
            ESP_LOGW(TAG, "Already initialized");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Initializing DataLogger...");

        if (!config_.enabled)
        {
            ESP_LOGI(TAG, "DataLogger disabled in configuration");
            setState(digitoys::core::ComponentState::INITIALIZED);
            return ESP_OK;
        }

        // Create flush timer if auto-flush is enabled
        if (config_.flush_interval_ms > 0)
        {
            esp_timer_create_args_t timer_args = {
                .callback = &DataLogger::flushTimerCallback,
                .arg = this,
                .name = "data_logger_flush"};

            esp_err_t ret = esp_timer_create(&timer_args, &flush_timer_);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to create flush timer: %s", esp_err_to_name(ret));
                return ret;
            }
        }

        setState(digitoys::core::ComponentState::INITIALIZED);
        ESP_LOGI(TAG, "DataLogger initialized successfully");
        return ESP_OK;
    }

    esp_err_t DataLogger::start()
    {
        if (getState() != digitoys::core::ComponentState::INITIALIZED)
        {
            ESP_LOGW(TAG, "Not initialized or already running");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Starting DataLogger...");

        if (!config_.enabled)
        {
            ESP_LOGI(TAG, "DataLogger disabled, skipping start");
            setState(digitoys::core::ComponentState::RUNNING);
            return ESP_OK;
        }

        // Start auto-flush timer if configured
        if (flush_timer_ && config_.flush_interval_ms > 0)
        {
            esp_err_t ret = esp_timer_start_periodic(flush_timer_,
                                                     config_.flush_interval_ms * 1000);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to start flush timer: %s", esp_err_to_name(ret));
                return ret;
            }
        }

        // Reset counters
        current_memory_usage_ = 0;
        entry_count_ = 0;

        setState(digitoys::core::ComponentState::RUNNING);
        ESP_LOGI(TAG, "DataLogger started successfully");
        return ESP_OK;
    }

    esp_err_t DataLogger::stop()
    {
        if (getState() != digitoys::core::ComponentState::RUNNING)
        {
            ESP_LOGW(TAG, "Not running");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Stopping DataLogger...");

        // Stop timer
        if (flush_timer_)
        {
            esp_timer_stop(flush_timer_);
        }

        // Final flush
        if (config_.enabled)
        {
            doFlush();
        }

        setState(digitoys::core::ComponentState::STOPPED);
        ESP_LOGI(TAG, "DataLogger stopped");
        return ESP_OK;
    }

    esp_err_t DataLogger::shutdown()
    {
        ESP_LOGI(TAG, "Shutting down DataLogger...");

        // Stop if running
        if (getState() == digitoys::core::ComponentState::RUNNING)
        {
            stop();
        }

        // Cleanup timer
        if (flush_timer_)
        {
            esp_timer_delete(flush_timer_);
            flush_timer_ = nullptr;
        }

        // Clear all data
        if (config_.enabled)
        {
            clear();
        }

        setState(digitoys::core::ComponentState::UNINITIALIZED);
        ESP_LOGI(TAG, "DataLogger shutdown complete");
        return ESP_OK;
    }

    size_t DataLogger::getMemoryUsage() const
    {
        return current_memory_usage_;
    }

    size_t DataLogger::getEntryCount() const
    {
        return entry_count_;
    }

    esp_err_t DataLogger::flush()
    {
        if (!isEnabled())
        {
            return ESP_OK;
        }
        return doFlush();
    }

    esp_err_t DataLogger::clear()
    {
        if (!isEnabled())
        {
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Clearing all data (entries: %zu, memory: %zu bytes)",
                 entry_count_, current_memory_usage_);

        // Reset counters
        current_memory_usage_ = 0;
        entry_count_ = 0;

        ESP_LOGI(TAG, "Data cleared successfully");
        return ESP_OK;
    }

    void DataLogger::flushTimerCallback(void *arg)
    {
        DataLogger *logger = static_cast<DataLogger *>(arg);
        if (logger && logger->isEnabled())
        {
            logger->doFlush();
        }
    }

    esp_err_t DataLogger::doFlush()
    {
        if (entry_count_ == 0)
        {
            return ESP_OK;
        }

        ESP_LOGD(TAG, "Flushing data (entries: %zu, memory: %zu bytes)",
                 entry_count_, current_memory_usage_);

        // TODO: In future steps, implement actual data source flushing
        // For now, just log the flush operation

        return ESP_OK;
    }

    bool DataLogger::checkMemoryLimits() const
    {
        size_t limit_bytes = config_.max_memory_kb * 1024;
        return current_memory_usage_ < limit_bytes;
    }

} // namespace digitoys::datalogger
