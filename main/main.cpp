// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "WifiMonitor.hpp" // Add our new component
#include "SystemMonitor.hpp"
#include "ControlTask.hpp"
#include "DataLoggerService.hpp" // Add DataLogger integration
#include "DataModeling.hpp" // Add DataModeling component

static const char *TAG = "APP_MAIN";
using namespace lidar;
extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting DigiToys firmware with unified configuration system");

    // --- LiDAR hardware setup using configuration factory ---
    auto lidar_config = lidar::LiDARConfig::createProductionConfig();
    static lidar::LiDAR lidar{lidar_config};
    ESP_ERROR_CHECK(lidar.initialize());
    ESP_ERROR_CHECK(lidar.start());
    ESP_LOGI(TAG, "LiDAR initialized and started with validated configuration");

    // --- PWM passthrough driver using configuration factory ---
    auto throttle_config = adas::PwmChannelConfig::createThrottleConfig();
    std::vector<adas::PwmChannelConfig> configs = {throttle_config};
    static adas::PwmDriver pwm_driver{configs};
    ESP_ERROR_CHECK(pwm_driver.initialize());
    ESP_ERROR_CHECK(pwm_driver.start());
    ESP_LOGI(TAG, "PWM passthrough initialized and started");

    // --- WiFi Monitor (replaces original monitor) ---
    static wifi_monitor::WifiMonitor wifi_mon;
    ESP_ERROR_CHECK(wifi_mon.initialize());
    ESP_ERROR_CHECK(wifi_mon.start());
    ESP_LOGI(TAG, "WiFi Monitor initialized and started");

    // --- Launch ControlTask with WiFi Monitor ---
    static control::ControlContext ctx = {&lidar, &pwm_driver, &wifi_mon};
    static control::ControlTask control_task(&ctx);
    ESP_ERROR_CHECK(control_task.initialize());
    ESP_ERROR_CHECK(control_task.start());
    ESP_LOGI(TAG, "Control task initialized and started with WiFi Monitor");

    // --- DataLogger Service for Hardware Trials ---
    static digitoys::datalogger::DataLoggerService data_logger(&control_task, true); // Physics analysis enabled
    ESP_ERROR_CHECK(data_logger.initialize());
    ESP_ERROR_CHECK(data_logger.start());
    ESP_LOGI(TAG, "DataLogger service initialized and started");

    // --- DataModeling Service for Behavior Analysis ---
    static digitoys::datamodeling::DataModeling data_modeling;
    ESP_ERROR_CHECK(data_modeling.initialize());
    ESP_ERROR_CHECK(data_modeling.start());
    ESP_LOGI(TAG, "DataModeling service initialized and started");

    // --- Connect DataLogger to DataModeling Pipeline ---
    // Enable streaming mode for real-time data processing
    data_logger.getDataLogger()->setStreamingMode(true);
    
    // Set up data stream callback to feed DataModeling
    data_logger.getDataLogger()->setStreamingCallback([&data_modeling](const digitoys::datalogger::DataEntry& entry, const std::string& source_name) {
        // Collect recent data entries for processing
        static std::vector<digitoys::datalogger::DataEntry> batch_buffer;
        batch_buffer.push_back(entry);
        
        // Process in batches every 10 entries for efficiency
        if (batch_buffer.size() >= 10) {
            auto behavior_point = data_modeling.processRawData(batch_buffer);
            batch_buffer.clear();
            
            // Optional: Log processing stats
            ESP_LOGD(TAG, "Processed batch: Session=%lu, Speed=%.2f", 
                    behavior_point.session_data.session_id, 
                    behavior_point.physics_data.calculated_speed);
        }
    });
    ESP_LOGI(TAG, "DataLogger pipeline connected to DataModeling");

    // --- Connect DataLogger to WiFi Monitor for button control ---
    wifi_mon.setDataLoggerService(&data_logger);
    ESP_LOGI(TAG, "DataLogger connected to WiFi Monitor for logging control");

    // --- Connect DataModeling to WiFi Monitor for session control ---
    wifi_mon.setDataModelingService(&data_modeling);
    ESP_LOGI(TAG, "DataModeling connected to WiFi Monitor for session management");

    // Print initial status after all services are running
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for initial data collection
    data_logger.printStatus();

    ESP_LOGI(TAG, "Application startup complete - all services running");

    // Keep app_main idle (this allows other tasks to continue running)
    vTaskDelete(nullptr);
}
