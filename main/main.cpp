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
#include "PhysicsAnalyzer.hpp"    // Add Physics Analysis
#include "PhysicsAnalyzerAPI.hpp" // Add HTTP API integration
#include "MultiLoggerAPI.hpp"     // Add Multi-Logger Management

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
    static digitoys::datalogger::DataLoggerService data_logger(&control_task, true); // RE-ENABLE PHYSICS ANALYSIS
    ESP_ERROR_CHECK(data_logger.initialize());
    ESP_ERROR_CHECK(data_logger.start());
    ESP_LOGI(TAG, "DataLogger service initialized and started");

    // --- Physics Analyzer TEMPORARILY DISABLED for debugging ---
    ESP_LOGI(TAG, "Physics Analyzer temporarily disabled for debugging");
    
    /*
    // --- Physics Analyzer with PRODUCTION CONFIG (longer intervals) ---
    ESP_LOGI(TAG, "About to create Physics Analyzer config...");
    auto analyzer_config = digitoys::datalogger::PhysicsAnalyzer::AnalyzerConfig{
        .enabled = true,   
        .analysis_interval_ms = 30000,  // 30 seconds - safe interval for production
        .data_window_size = 50,
        .emergency_decel_threshold = 4.0f,
        .safety_margin_critical = 15.0f,
        .safety_margin_warning = 40.0f
    };
    ESP_LOGI(TAG, "Physics Analyzer config created successfully");

    ESP_LOGI(TAG, "About to create Physics Analyzer instance...");
    static digitoys::datalogger::PhysicsAnalyzer physics_analyzer(data_logger.getDataLogger(), analyzer_config);
    ESP_LOGI(TAG, "Physics Analyzer instance created successfully");
    
    ESP_LOGI(TAG, "About to initialize Physics Analyzer...");
    ESP_ERROR_CHECK(physics_analyzer.initialize());
    ESP_LOGI(TAG, "Physics Analyzer initialized successfully");
    
    ESP_LOGI(TAG, "About to start Physics Analyzer...");
    ESP_ERROR_CHECK(physics_analyzer.start());
    ESP_LOGI(TAG, "Physics analyzer initialized and started (30s analysis interval)");
    */

    // --- HTTP API Integration TEMPORARILY DISABLED ---
    ESP_LOGI(TAG, "HTTP API integration temporarily disabled for debugging");
    
    /*
    // --- HTTP API Integration for Multi-Logger System ---
    static digitoys::datalogger::PhysicsAnalyzerAPI physics_api(&physics_analyzer);
    static digitoys::datalogger::MultiLoggerAPI multi_logger_api(data_logger.getDataLogger(), &physics_analyzer);
    
    httpd_handle_t server_handle = wifi_mon.getHttpServerHandle();
    if (server_handle)
    {
        ESP_ERROR_CHECK(physics_api.registerEndpoints(server_handle));
        ESP_LOGI(TAG, "Physics analysis HTTP API endpoints registered");
        
        ESP_ERROR_CHECK(multi_logger_api.registerEndpoints(server_handle));
        ESP_LOGI(TAG, "Multi-Logger management API endpoints registered");
    }
    else
    {
        ESP_LOGW(TAG, "HTTP server not available for API registration");
    }
    */

    // Print initial status after all services are running
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for initial data collection
    data_logger.printStatus();

    ESP_LOGI(TAG, "Application startup complete - all services running");

    // Keep app_main idle (this allows other tasks to continue running)
    vTaskDelete(nullptr);
}
