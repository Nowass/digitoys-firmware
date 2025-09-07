#include "DataLogger.hpp"
#include <esp_log.h>
#include <cstring>
#include <algorithm>

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
        if (collection_timer_)
        {
            esp_timer_delete(collection_timer_);
            collection_timer_ = nullptr;
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

        // Create data collection timer (always created, started only when sources are registered)
        {
            esp_timer_create_args_t timer_args = {
                .callback = &DataLogger::collectionTimerCallback,
                .arg = this,
                .name = "data_logger_collect"};

            esp_err_t ret = esp_timer_create(&timer_args, &collection_timer_);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to create collection timer: %s", esp_err_to_name(ret));
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

        // Stop timers
        if (flush_timer_)
        {
            esp_timer_stop(flush_timer_);
        }
        if (collection_timer_)
        {
            esp_timer_stop(collection_timer_);
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

        // Cleanup timers
        if (flush_timer_)
        {
            esp_timer_delete(flush_timer_);
            flush_timer_ = nullptr;
        }
        if (collection_timer_)
        {
            esp_timer_delete(collection_timer_);
            collection_timer_ = nullptr;
        }

        // Clear all data and sources
        if (config_.enabled)
        {
            clear();
            data_sources_.clear();
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

        // Clear collected data
        collected_data_.clear();

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

    esp_err_t DataLogger::registerDataSource(DataSourcePtr source)
    {
        if (!source)
        {
            ESP_LOGE(TAG, "Cannot register null data source");
            return ESP_ERR_INVALID_ARG;
        }

        if (!isEnabled())
        {
            ESP_LOGW(TAG, "DataLogger not enabled, skipping data source registration");
            return ESP_OK;
        }

        DataSourceConfig config = source->getSourceConfig();

        if (data_sources_.find(config.source_name) != data_sources_.end())
        {
            ESP_LOGW(TAG, "Data source '%s' already registered", config.source_name.c_str());
            return ESP_ERR_INVALID_STATE;
        }

        // Register the source
        data_sources_[config.source_name] = source;

        // Call registration callback
        esp_err_t ret = source->onRegistered();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Data source '%s' registration callback failed: %s",
                     config.source_name.c_str(), esp_err_to_name(ret));
            data_sources_.erase(config.source_name);
            return ret;
        }

        ESP_LOGI(TAG, "Registered data source '%s' (rate: %lu ms)",
                 config.source_name.c_str(), config.sample_rate_ms);

        // Start collection timer if this is the first source and we're running
        if (data_sources_.size() == 1 && isRunning() && collection_timer_)
        {
            // Use minimum sample rate from all sources
            uint32_t min_rate = config.sample_rate_ms;
            for (const auto &pair : data_sources_)
            {
                DataSourceConfig src_config = pair.second->getSourceConfig();
                min_rate = std::min(min_rate, src_config.sample_rate_ms);
            }

            ret = esp_timer_start_periodic(collection_timer_, min_rate * 1000);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to start collection timer: %s", esp_err_to_name(ret));
            }
        }

        return ESP_OK;
    }

    esp_err_t DataLogger::unregisterDataSource(const std::string &source_name)
    {
        auto it = data_sources_.find(source_name);
        if (it == data_sources_.end())
        {
            ESP_LOGW(TAG, "Data source '%s' not found", source_name.c_str());
            return ESP_ERR_NOT_FOUND;
        }

        // Call unregistration callback
        esp_err_t ret = it->second->onUnregistered();
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Data source '%s' unregistration callback failed: %s",
                     source_name.c_str(), esp_err_to_name(ret));
        }

        data_sources_.erase(it);
        ESP_LOGI(TAG, "Unregistered data source '%s'", source_name.c_str());

        // Stop collection timer if no sources remain
        if (data_sources_.empty() && collection_timer_)
        {
            esp_timer_stop(collection_timer_);
        }

        return ESP_OK;
    }

    std::vector<std::string> DataLogger::getRegisteredSources() const
    {
        std::vector<std::string> sources;
        sources.reserve(data_sources_.size());

        for (const auto &pair : data_sources_)
        {
            sources.push_back(pair.first);
        }

        return sources;
    }

    esp_err_t DataLogger::collectFromSources()
    {
        if (!isEnabled() || data_sources_.empty())
        {
            return ESP_OK;
        }

        return doDataCollection();
    }

    void DataLogger::collectionTimerCallback(void *arg)
    {
        DataLogger *logger = static_cast<DataLogger *>(arg);
        if (logger && logger->isEnabled())
        {
            logger->doDataCollection();
        }
    }

    esp_err_t DataLogger::doDataCollection()
    {
        if (data_sources_.empty())
        {
            return ESP_OK;
        }

        size_t initial_count = collected_data_.size();

        for (const auto &pair : data_sources_)
        {
            const std::string &source_name = pair.first;
            DataSourcePtr source = pair.second;

            if (!source->isReady())
            {
                continue;
            }

            std::vector<DataEntry> entries;
            esp_err_t ret = source->collectData(entries);

            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to collect data from source '%s': %s",
                         source_name.c_str(), esp_err_to_name(ret));
                continue;
            }

            // Add entries to collected data with memory limit checking
            for (const DataEntry &entry : entries)
            {
                size_t entry_size = calculateEntrySize(entry);

                if (!checkMemoryLimits() ||
                    (current_memory_usage_ + entry_size) > (config_.max_memory_kb * 1024))
                {
                    ESP_LOGW(TAG, "Memory limit reached, skipping entry from '%s'", source_name.c_str());
                    break;
                }

                collected_data_.push_back(entry);
                current_memory_usage_ += entry_size;
                entry_count_++;
            }
        }

        size_t collected_count = collected_data_.size() - initial_count;
        if (collected_count > 0)
        {
            ESP_LOGD(TAG, "Collected %zu entries from %zu sources",
                     collected_count, data_sources_.size());
        }

        return ESP_OK;
    }

    size_t DataLogger::calculateEntrySize(const DataEntry &entry) const
    {
        return sizeof(DataEntry) + entry.key.size() + entry.value.size();
    }

} // namespace digitoys::datalogger
