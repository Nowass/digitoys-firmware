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
#include "DataLoggerService.hpp"  // Add DataLogger integration
#include "DataModeling.hpp"       // Add DataModeling component
#include "ModelingDataSource.hpp" // IDataSource adapter for real sensor streaming

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
    static digitoys::datamodeling::DataModeling data_modeling(&lidar, &pwm_driver);
    ESP_ERROR_CHECK(data_modeling.initialize());
    ESP_ERROR_CHECK(data_modeling.start());
    ESP_LOGI(TAG, "DataModeling service initialized and started");

    // Enable real-time sensor data collection (100ms intervals)
    // Note: Temporarily disabled to prevent memory overflow during testing
    // ESP_ERROR_CHECK(data_modeling.setRealTimeCollection(true, 100));

    // (Streaming of modeling data is currently disabled during investigation.)

    // --- Optional: Register ModelingDataSource for real sensor streaming (conservative rate)
    // Keep ControlTask untouched. This source reads LiDAR/PWM directly in parallel.
    {
        static constexpr bool kEnableModelingSource = true; // Flip to false to disable quickly
        static constexpr uint32_t kModelingSampleMs = 500;  // Match ControlTaskDataSource to avoid timer downshift
        if (kEnableModelingSource)
        {
            auto *logger = data_logger.getDataLogger();
            if (logger)
            {
                auto modeling_source = std::make_shared<digitoys::datamodeling::ModelingDataSource>(&data_modeling, kModelingSampleMs, true);
                esp_err_t reg_ret = logger->registerDataSource(modeling_source);
                if (reg_ret == ESP_OK)
                {
                    ESP_LOGI(TAG, "ModelingDataSource registered at %ums", kModelingSampleMs);
                }
                else
                {
                    ESP_LOGW(TAG, "Failed to register ModelingDataSource: %s", esp_err_to_name(reg_ret));
                }
            }
            else
            {
                ESP_LOGW(TAG, "DataLogger not available for ModelingDataSource registration");
            }
        }
    }

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
