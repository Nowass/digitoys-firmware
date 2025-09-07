#include "DataLoggerDemo.hpp"
#include <esp_log.h>

namespace digitoys::datalogger
{
    static const char *TAG = "DataLoggerDemo";

    // Static member definitions
    std::unique_ptr<DataLogger> DataLoggerDemo::logger_;
    std::shared_ptr<TestDataSource> DataLoggerDemo::test_source1_;
    std::shared_ptr<TestDataSource> DataLoggerDemo::test_source2_;
    bool DataLoggerDemo::is_initialized_ = false;

    esp_err_t DataLoggerDemo::initialize()
    {
        if (is_initialized_)
        {
            ESP_LOGW(TAG, "Demo already initialized");
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Initializing DataLogger demo...");

        // Create DataLogger with demo-friendly configuration
        DataLoggerConfig config = {
            .enabled = true,
            .max_entries = 200,         // Reasonable for demo
            .flush_interval_ms = 10000, // Flush every 10 seconds
            .max_memory_kb = 32         // 32KB should be plenty for demo
        };

        logger_ = std::make_unique<DataLogger>(config);

        // Initialize the logger
        esp_err_t ret = logger_->initialize();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize DataLogger: %s", esp_err_to_name(ret));
            return ret;
        }

        // Create test data sources with different patterns
        test_source1_ = std::make_shared<TestDataSource>("VehicleSensors", 500, true); // 500ms rate
        test_source1_->setDataPattern(TestDataSource::DataPattern::SINUSOIDAL);
        test_source1_->setSimulateFailures(false); // Stable source

        test_source2_ = std::make_shared<TestDataSource>("DiagnosticSensors", 1000, true); // 1000ms rate
        test_source2_->setDataPattern(TestDataSource::DataPattern::RANDOM);
        test_source2_->setSimulateFailures(true); // Occasional failures for realism

        is_initialized_ = true;
        ESP_LOGI(TAG, "DataLogger demo initialized successfully");
        return ESP_OK;
    }

    esp_err_t DataLoggerDemo::start()
    {
        if (!is_initialized_)
        {
            ESP_LOGE(TAG, "Demo not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Starting DataLogger demo...");

        // Start the logger
        esp_err_t ret = logger_->start();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start DataLogger: %s", esp_err_to_name(ret));
            return ret;
        }

        // Register test data sources
        ret = logger_->registerDataSource(test_source1_);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register test source 1: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = logger_->registerDataSource(test_source2_);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register test source 2: %s", esp_err_to_name(ret));
            logger_->unregisterDataSource("VehicleSensors"); // Cleanup
            return ret;
        }

        ESP_LOGI(TAG, "DataLogger demo started successfully");
        ESP_LOGI(TAG, "Registered sources: %zu", logger_->getRegisteredSources().size());

        // Print initial status
        printStatus();

        return ESP_OK;
    }

    esp_err_t DataLoggerDemo::stop()
    {
        if (!is_initialized_)
        {
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Stopping DataLogger demo...");

        if (logger_)
        {
            // Print final statistics
            printStatus();

            // Unregister sources
            logger_->unregisterDataSource("VehicleSensors");
            logger_->unregisterDataSource("DiagnosticSensors");

            // Stop and shutdown logger
            logger_->stop();
            logger_->shutdown();
        }

        // Reset state
        test_source1_.reset();
        test_source2_.reset();
        logger_.reset();
        is_initialized_ = false;

        ESP_LOGI(TAG, "DataLogger demo stopped");
        return ESP_OK;
    }

    void DataLoggerDemo::getStatistics(size_t &logger_entries,
                                       uint32_t &source1_points,
                                       uint32_t &source2_points)
    {
        logger_entries = 0;
        source1_points = 0;
        source2_points = 0;

        if (logger_)
        {
            logger_entries = logger_->getEntryCount();
        }

        if (test_source1_)
        {
            source1_points = test_source1_->getDataPointsGenerated();
        }

        if (test_source2_)
        {
            source2_points = test_source2_->getDataPointsGenerated();
        }
    }

    void DataLoggerDemo::printStatus()
    {
        if (!is_initialized_ || !logger_)
        {
            ESP_LOGI(TAG, "Demo not running");
            return;
        }

        size_t logger_entries;
        uint32_t source1_points, source2_points;
        getStatistics(logger_entries, source1_points, source2_points);

        std::vector<std::string> sources = logger_->getRegisteredSources();

        ESP_LOGI(TAG, "=== DataLogger Demo Status ===");
        ESP_LOGI(TAG, "Logger enabled: %s", logger_->isEnabled() ? "YES" : "NO");
        ESP_LOGI(TAG, "Registered sources: %zu", sources.size());
        for (const auto &source : sources)
        {
            ESP_LOGI(TAG, "  - %s", source.c_str());
        }
        ESP_LOGI(TAG, "Total entries collected: %zu", logger_entries);
        ESP_LOGI(TAG, "Memory usage: %zu bytes", logger_->getMemoryUsage());
        ESP_LOGI(TAG, "VehicleSensors data points: %lu", source1_points);
        ESP_LOGI(TAG, "DiagnosticSensors data points: %lu", source2_points);
        ESP_LOGI(TAG, "=============================");
    }

} // namespace digitoys::datalogger
