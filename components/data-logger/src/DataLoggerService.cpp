#include "DataLoggerService.hpp"
#include "ControlTaskDataSource.hpp"
#include "ControlTask.hpp"
#include "DataLoggerConfig.hpp"
#include <esp_log.h>

namespace digitoys::datalogger
{

    static const char *TAG = "DataLoggerService";

    DataLoggerService::DataLoggerService(control::ControlTask *control_task,
                                         bool enable_physics_analysis)
        : control_task_(control_task), physics_analysis_enabled_(enable_physics_analysis), is_initialized_(false), is_running_(false)
    {
    }

    esp_err_t DataLoggerService::initialize()
    {
        if (is_initialized_)
        {
            ESP_LOGW(TAG, "DataLogger service already initialized");
            return ESP_OK;
        }

        if (!control_task_)
        {
            ESP_LOGE(TAG, "ControlTask is required for DataLogger service");
            return ESP_ERR_INVALID_ARG;
        }

        ESP_LOGI(TAG, "Initializing DataLogger service for hardware trials...");

        // Create production DataLogger configuration
        // Uses settings from DataLoggerConfig.hpp for easy hardware trial tuning
        DataLoggerConfig config = {
            .enabled = true,
            .max_entries = digitoys::datalogger::config::MAX_ENTRIES,
            .flush_interval_ms = digitoys::datalogger::config::FLUSH_INTERVAL_MS,
            .max_memory_kb = digitoys::datalogger::config::MAX_MEMORY_KB,
            .monitoring_mode = true,  // Start in monitoring mode
            .streaming_mode = false,  // Streaming will be enabled by WifiMonitor when logging starts
            .streaming_buffer_size = 100  // Small circular buffer for real-time display
        };

        logger_ = std::make_unique<DataLogger>(config);

        // Initialize the logger
        esp_err_t ret = logger_->initialize();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize DataLogger: %s", esp_err_to_name(ret));
            return ret;
        }

        // Create control system data source for real hardware data
        control_source_ = std::make_shared<ControlTaskDataSource>(
            control_task_,
            digitoys::datalogger::config::SAMPLE_RATE_MS, // Use configured sample rate
            true                                          // Start enabled
        );

        // Configure physics analysis and speed threshold based on parameters
        control_source_->setCapturePhysicsData(physics_analysis_enabled_);
        control_source_->setSpeedThreshold(digitoys::datalogger::config::SPEED_THRESHOLD);

        is_initialized_ = true;
        ESP_LOGI(TAG, "DataLogger service initialized successfully");
        ESP_LOGI(TAG, "Physics analysis: %s", physics_analysis_enabled_ ? "ENABLED" : "DISABLED");
        ESP_LOGI(TAG, "Memory allocated: %d KB", config.max_memory_kb);

        return ESP_OK;
    }

    esp_err_t DataLoggerService::start()
    {
        if (!is_initialized_)
        {
            ESP_LOGE(TAG, "Service not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        if (is_running_)
        {
            ESP_LOGW(TAG, "Service already running");
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Starting DataLogger service...");

        esp_err_t ret = logger_->start();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start DataLogger: %s", esp_err_to_name(ret));
            return ret;
        }

        // Register the control source AFTER starting the logger
        ret = logger_->registerDataSource(control_source_);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register ControlTaskDataSource: %s", esp_err_to_name(ret));
            return ret;
        }

        is_running_ = true;
        ESP_LOGI(TAG, "DataLogger service started - collecting real vehicle data");

        return ESP_OK;
    }

    esp_err_t DataLoggerService::stop()
    {
        if (!is_running_)
        {
            ESP_LOGW(TAG, "Service not running");
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Stopping DataLogger service...");

        if (logger_)
        {
            // Print final status before stopping
            printStatus();

            esp_err_t ret = logger_->stop();
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to stop DataLogger: %s", esp_err_to_name(ret));
                return ret;
            }
        }

        is_running_ = false;
        ESP_LOGI(TAG, "DataLogger service stopped");

        return ESP_OK;
    }

    bool DataLoggerService::isRunning() const
    {
        return is_running_;
    }

    void DataLoggerService::printStatus() const
    {
        if (!is_initialized_ || !logger_)
        {
            ESP_LOGW(TAG, "Service not initialized - no status available");
            return;
        }

        ESP_LOGI(TAG, "=== DATALOGGER SERVICE STATUS ===");
        ESP_LOGI(TAG, "Service State: %s", is_running_ ? "RUNNING" : "STOPPED");
        ESP_LOGI(TAG, "Physics Analysis: %s", physics_analysis_enabled_ ? "ENABLED" : "DISABLED");

        if (logger_)
        {
            size_t memory_usage = logger_->getMemoryUsage();
            size_t memory_limit = logger_->getConfig().max_memory_kb * 1024;
            float memory_percent = (memory_usage * 100.0f) / memory_limit;

            ESP_LOGI(TAG, "Memory Usage: %zu bytes (%.1f%%)", memory_usage, memory_percent);
            ESP_LOGI(TAG, "Total Entries: %zu", logger_->getEntryCount());

            if (memory_percent > 80.0f)
            {
                ESP_LOGW(TAG, "High memory usage detected");
            }
        }

        // Print control system data statistics
        if (control_source_ && control_source_->isReady())
        {
            uint32_t total_samples, physics_samples, brake_events, warning_events;
            control_source_->getStatistics(total_samples, physics_samples, brake_events, warning_events);

            ESP_LOGI(TAG, "=== CONTROL SYSTEM DATA ===");
            ESP_LOGI(TAG, "Total Samples: %lu", total_samples);
            ESP_LOGI(TAG, "Physics Samples: %lu", physics_samples);
            ESP_LOGI(TAG, "Brake Events: %lu", brake_events);
            ESP_LOGI(TAG, "Warning Events: %lu", warning_events);

            if (physics_analysis_enabled_ && physics_samples > 0)
            {
                ESP_LOGI(TAG, "=== PHYSICS DATA AVAILABLE ===");
                ESP_LOGI(TAG, "Use real-time monitor to view current physics data");
                ESP_LOGI(TAG, "Or check logged entries for detailed analysis");
            }
        }

        ESP_LOGI(TAG, "==============================");
    }

    esp_err_t DataLoggerService::shutdown()
    {
        ESP_LOGI(TAG, "Shutting down DataLogger service...");

        if (is_running_)
        {
            stop();
        }

        if (logger_)
        {
            // Unregister sources
            if (control_source_)
            {
                logger_->unregisterDataSource("ControlSystem");
                control_source_.reset();
            }

            logger_.reset();
        }

        is_initialized_ = false;
        is_running_ = false;

        ESP_LOGI(TAG, "DataLogger service shutdown complete");
        return ESP_OK;
    }

    size_t DataLoggerService::getMemoryUsage() const
    {
        if (logger_)
        {
            return logger_->getMemoryUsage();
        }
        return 0;
    }

    DataLogger *DataLoggerService::getDataLogger() const
    {
        return logger_.get();
    }

} // namespace digitoys::datalogger
