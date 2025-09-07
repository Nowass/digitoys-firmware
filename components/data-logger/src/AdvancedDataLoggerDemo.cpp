#include "AdvancedDataLoggerDemo.hpp"
#include "ControlTaskDataSource.hpp"
#include "ControlTask.hpp"
#include <esp_log.h>
#include <memory>

namespace digitoys::datalogger
{
    static const char *TAG = "AdvancedDataLoggerDemo";

    // Static member definitions
    std::unique_ptr<DataLogger> AdvancedDataLoggerDemo::logger_;
    std::shared_ptr<TestDataSource> AdvancedDataLoggerDemo::test_source_;
    std::shared_ptr<ControlTaskDataSource> AdvancedDataLoggerDemo::control_source_;
    bool AdvancedDataLoggerDemo::is_initialized_ = false;
    bool AdvancedDataLoggerDemo::has_real_control_task_ = false;

    esp_err_t AdvancedDataLoggerDemo::initialize(control::ControlTask *control_task)
    {
        if (is_initialized_)
        {
            ESP_LOGW(TAG, "Advanced demo already initialized");
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Initializing Advanced DataLogger demo...");

        // Create DataLogger with enhanced configuration for physics analysis
        DataLoggerConfig config = {
            .enabled = true,
            .max_entries = 500,         // More entries for physics data
            .flush_interval_ms = 15000, // Longer flush interval for detailed analysis
            .max_memory_kb = 64         // More memory for complex data
        };

        logger_ = std::make_unique<DataLogger>(config);

        // Initialize the logger
        esp_err_t ret = logger_->initialize();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize DataLogger: %s", esp_err_to_name(ret));
            return ret;
        }

        // Create test data source with realistic vehicle patterns
        test_source_ = std::make_shared<TestDataSource>("VehicleSimulator", 300, true);
        test_source_->setDataPattern(TestDataSource::DataPattern::SINUSOIDAL);
        test_source_->setSimulateFailures(false); // Clean data for baseline

        // Create control system data source if ControlTask is available
        if (control_task && control_task->isInitialized())
        {
            control_source_ = std::make_shared<ControlTaskDataSource>(control_task, 150, true);
            control_source_->setCapturePhysicsData(true);
            control_source_->setSpeedThreshold(0.05f); // Capture low-speed events
            has_real_control_task_ = true;
            ESP_LOGI(TAG, "Real ControlTask integration enabled");
        }
        else
        {
            ESP_LOGW(TAG, "No ControlTask provided - using test data only");
            has_real_control_task_ = false;
        }

        is_initialized_ = true;
        ESP_LOGI(TAG, "Advanced DataLogger demo initialized successfully (real control: %s)",
                 has_real_control_task_ ? "YES" : "NO");
        return ESP_OK;
    }

    esp_err_t AdvancedDataLoggerDemo::start()
    {
        if (!is_initialized_)
        {
            ESP_LOGE(TAG, "Advanced demo not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Starting Advanced DataLogger demo...");

        // Start the logger
        esp_err_t ret = logger_->start();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start DataLogger: %s", esp_err_to_name(ret));
            return ret;
        }

        // Register test data source
        ret = logger_->registerDataSource(test_source_);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register test source: %s", esp_err_to_name(ret));
            return ret;
        }

        // Register control system data source if available
        if (has_real_control_task_ && control_source_)
        {
            ret = logger_->registerDataSource(control_source_);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to register control source: %s", esp_err_to_name(ret));
                logger_->unregisterDataSource("VehicleSimulator"); // Cleanup
                return ret;
            }
        }

        ESP_LOGI(TAG, "Advanced DataLogger demo started successfully");
        ESP_LOGI(TAG, "Active data sources: %zu", logger_->getRegisteredSources().size());

        // Print initial status
        printStatus();

        return ESP_OK;
    }

    esp_err_t AdvancedDataLoggerDemo::stop()
    {
        if (!is_initialized_)
        {
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Stopping Advanced DataLogger demo...");

        if (logger_)
        {
            // Print final statistics and analysis
            printStatus();
            generatePhysicsReport();

            // Unregister sources
            logger_->unregisterDataSource("VehicleSimulator");
            if (has_real_control_task_)
            {
                logger_->unregisterDataSource("ControlSystem");
            }

            // Stop and shutdown logger
            logger_->stop();
            logger_->shutdown();
        }

        // Reset state
        test_source_.reset();
        control_source_.reset();
        logger_.reset();
        is_initialized_ = false;
        has_real_control_task_ = false;

        ESP_LOGI(TAG, "Advanced DataLogger demo stopped");
        return ESP_OK;
    }

    void AdvancedDataLoggerDemo::getStatistics(size_t &logger_entries,
                                               uint32_t &test_points,
                                               uint32_t &control_samples,
                                               uint32_t &physics_samples,
                                               uint32_t &brake_events)
    {
        logger_entries = 0;
        test_points = 0;
        control_samples = 0;
        physics_samples = 0;
        brake_events = 0;

        if (logger_)
        {
            logger_entries = logger_->getEntryCount();
        }

        if (test_source_)
        {
            test_points = test_source_->getDataPointsGenerated();
        }

        if (control_source_)
        {
            uint32_t warning_events; // Not used in summary
            control_source_->getStatistics(control_samples, physics_samples,
                                           brake_events, warning_events);
        }
    }

    void AdvancedDataLoggerDemo::printStatus()
    {
        if (!is_initialized_ || !logger_)
        {
            ESP_LOGI(TAG, "Advanced demo not running");
            return;
        }

        size_t logger_entries;
        uint32_t test_points, control_samples, physics_samples, brake_events;
        getStatistics(logger_entries, test_points, control_samples, physics_samples, brake_events);

        std::vector<std::string> sources = logger_->getRegisteredSources();

        ESP_LOGI(TAG, "=== Advanced DataLogger Demo Status ===");
        ESP_LOGI(TAG, "Logger enabled: %s", logger_->isEnabled() ? "YES" : "NO");
        ESP_LOGI(TAG, "Real control task: %s", has_real_control_task_ ? "YES" : "NO");
        ESP_LOGI(TAG, "Active sources: %zu", sources.size());
        for (const auto &source : sources)
        {
            ESP_LOGI(TAG, "  - %s", source.c_str());
        }
        ESP_LOGI(TAG, "Total entries: %zu", logger_entries);
        ESP_LOGI(TAG, "Memory usage: %zu bytes (%.1f%%)",
                 logger_->getMemoryUsage(),
                 (logger_->getMemoryUsage() * 100.0f) / (logger_->getConfig().max_memory_kb * 1024));
        ESP_LOGI(TAG, "--- Data Source Statistics ---");
        ESP_LOGI(TAG, "Test data points: %lu", test_points);
        if (has_real_control_task_)
        {
            ESP_LOGI(TAG, "Control samples: %lu", control_samples);
            ESP_LOGI(TAG, "Physics samples: %lu", physics_samples);
            ESP_LOGI(TAG, "Brake events detected: %lu", brake_events);
        }
        ESP_LOGI(TAG, "=====================================");
    }

    void AdvancedDataLoggerDemo::generatePhysicsReport()
    {
        ESP_LOGI(TAG, "=== PHYSICS ANALYSIS REPORT ===");

        if (!has_real_control_task_ || !control_source_)
        {
            ESP_LOGI(TAG, "No real control data - using test data only");
            ESP_LOGI(TAG, "Physics analysis requires actual ControlTask integration");
            return;
        }

        uint32_t total_samples, physics_samples, brake_events, warning_events;
        control_source_->getStatistics(total_samples, physics_samples, brake_events, warning_events);

        float period = (total_samples * control_source_->getSourceConfig().sample_rate_ms / 1000.0f);
        ESP_LOGI(TAG, "Data Collection Period: %.1fs", period);
        ESP_LOGI(TAG, "Total Control Samples: %lu", total_samples);
        ESP_LOGI(TAG, "Physics Samples: %lu", physics_samples);
        ESP_LOGI(TAG, "Sample Rate: %lu ms", control_source_->getSourceConfig().sample_rate_ms);

        ESP_LOGI(TAG, "=== BRAKING ANALYSIS ===");
        ESP_LOGI(TAG, "Brake Events Detected: %lu", brake_events);
        ESP_LOGI(TAG, "Warning Events: %lu", warning_events);

        if (total_samples > 0)
        {
            float brake_frequency = (brake_events * 100.0f) / total_samples;
            float warning_frequency = (warning_events * 100.0f) / total_samples;

            ESP_LOGI(TAG, "Brake Event Frequency: %.2f%% of samples", brake_frequency);
            ESP_LOGI(TAG, "Warning Event Frequency: %.2f%% of samples", warning_frequency);
        }

        ESP_LOGI(TAG, "=== RECOMMENDATIONS ===");
        if (brake_events > 5)
        {
            ESP_LOGI(TAG, "HIGH brake event frequency detected");
            ESP_LOGI(TAG, "Consider adjusting brake distance thresholds");
        }
        else if (brake_events < 2)
        {
            ESP_LOGI(TAG, "LOW brake event frequency - thresholds may be too conservative");
        }
        else
        {
            ESP_LOGI(TAG, "Brake event frequency appears normal");
        }

        if (physics_samples > 0)
        {
            ESP_LOGI(TAG, "Physics data collection successful");
            ESP_LOGI(TAG, "Use logged data for detailed braking behavior analysis");
        }

        ESP_LOGI(TAG, "=== DATA LOGGER PERFORMANCE ===");
        if (logger_)
        {
            size_t memory_usage = logger_->getMemoryUsage();
            size_t memory_limit = logger_->getConfig().max_memory_kb * 1024;
            float memory_percent = (memory_usage * 100.0f) / memory_limit;

            ESP_LOGI(TAG, "Memory Usage: %zu bytes (%.1f%%)", memory_usage, memory_percent);
            ESP_LOGI(TAG, "Total Entries: %zu", logger_->getEntryCount());

            if (memory_percent > 80.0f)
            {
                ESP_LOGI(TAG, "WARNING: High memory usage - consider increasing limits");
            }
        }

        ESP_LOGI(TAG, "===============================");
    }

} // namespace digitoys::datalogger
