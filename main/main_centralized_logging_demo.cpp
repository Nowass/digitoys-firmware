// app_main.cpp - Updated with centralized logging demonstration
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include "ControlTask.hpp"

// Include the new centralized logging system
#include "Logger.hpp"

using namespace lidar;
using namespace digitoys::constants::logging;

extern "C" void app_main()
{
    // Example of new centralized logging usage
    LOG_MAIN(I, "Starting DigiToys firmware with centralized logging system");

    // Traditional ESP_LOG* still works for backward compatibility
    ESP_LOGI("APP_MAIN", "Traditional logging still supported");

    // --- LiDAR hardware setup using configuration factory ---
    auto lidar_config = lidar::LiDARConfig::createProductionConfig();
    static lidar::LiDAR lidar{lidar_config};
    ESP_ERROR_CHECK(lidar.initialize());
    ESP_ERROR_CHECK(lidar.start());
    LOG_MAIN(I, "LiDAR initialized and started with validated configuration");

    // --- PWM passthrough driver using configuration factory ---
    auto throttle_config = adas::PwmChannelConfig::createThrottleConfig();
    std::vector<adas::PwmChannelConfig> configs = {throttle_config};
    static adas::PwmDriver pwm_driver{configs};
    ESP_ERROR_CHECK(pwm_driver.initialize());
    ESP_ERROR_CHECK(pwm_driver.start());
    LOG_MAIN(I, "PWM passthrough initialized and started");

    // --- Telemetry monitor ---
    static monitor::Monitor mon;
    ESP_ERROR_CHECK(mon.initialize());
    ESP_ERROR_CHECK(mon.start());
    LOG_MAIN(I, "Monitor initialized and started");

    // --- System monitor ---
    static monitor::SystemMonitor sys_mon;
    ESP_ERROR_CHECK(sys_mon.initialize());
    ESP_ERROR_CHECK(sys_mon.start());
    LOG_MAIN(I, "System monitor initialized and started");

    // --- Launch ControlTask ---
    static control::ControlContext ctx = {&lidar, &pwm_driver, &mon};
    static control::ControlTask control_task(&ctx);
    ESP_ERROR_CHECK(control_task.initialize());
    ESP_ERROR_CHECK(control_task.start());
    LOG_MAIN(I, "Control task initialized and started");

    // Demonstrate centralized logging features
    auto &logger = digitoys::core::Logger::getInstance();

    LOG_MAIN(I, "=== Centralized Logging System Test ===");

    // Test 1: Show initial component registration
    LOG_MAIN(I, "1. Component Registration Status:");
    auto components = logger.getAllComponents();
    for (const auto &[name, info] : components)
    {
        LOG_MAIN(I, "  Component: %s, TAG: %s, Level: %d, Messages: %lu, Enabled: %s",
                 name.c_str(), info.tag.c_str(), info.level, (unsigned long)info.message_count,
                 info.enabled ? "Yes" : "No");
    }

    // Test 2: Demonstrate runtime debug control (Option A - per component)
    LOG_MAIN(I, "2. Testing per-component debug control...");
    
    // Enable debug for specific component
    std::vector<std::string> debug_components = {"LiDARConfig"};
    logger.enableDebugFor(debug_components);
    LOG_MAIN(I, "  Enabled debug for LiDARConfig component");
    
    // Generate some log messages to test filtering
    DIGITOYS_LOGD("TestComponent", "TEST", "This debug message should appear");
    DIGITOYS_LOGI("TestComponent", "TEST", "This info message should appear");
    
    // Test 3: Demonstrate global debug toggle (Option B)
    LOG_MAIN(I, "3. Testing global debug mode...");
    logger.setDebugMode(true);
    LOG_MAIN(I, "  Global debug mode enabled");
    
    DIGITOYS_LOGD("TestComponent", "TEST", "This debug message should appear (global debug on)");
    
    logger.setDebugMode(false);
    LOG_MAIN(I, "  Global debug mode disabled");
    
    DIGITOYS_LOGD("TestComponent", "TEST", "This debug message should NOT appear (global debug off)");

    // Test 4: Demonstrate runtime log level changes
    LOG_MAIN(I, "4. Testing runtime log level control...");
    
    // Set global log level to WARNING (should suppress INFO messages)
    logger.setGlobalLogLevel(ESP_LOG_WARN);
    LOG_MAIN(W, "  Set global log level to WARNING");
    
    DIGITOYS_LOGI("TestComponent", "TEST", "This INFO message should NOT appear (level too low)");
    DIGITOYS_LOGW("TestComponent", "TEST", "This WARNING message should appear");
    DIGITOYS_LOGE("TestComponent", "TEST", "This ERROR message should appear");
    
    // Reset to INFO level
    logger.setGlobalLogLevel(ESP_LOG_INFO);
    LOG_MAIN(I, "  Reset global log level to INFO");
    
    DIGITOYS_LOGI("TestComponent", "TEST", "This INFO message should appear again");

    // Test 5: Show final component statistics
    LOG_MAIN(I, "5. Final Component Statistics:");
    components = logger.getAllComponents();
    for (const auto &[name, info] : components)
    {
        LOG_MAIN(I, "  Component: %s, Messages logged: %lu",
                 name.c_str(), (unsigned long)info.message_count);
    }

    LOG_MAIN(I, "=== Centralized Logging System Test Complete ===");

    // Keep app_main idle
    vTaskDelete(nullptr);
}
