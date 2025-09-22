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
// DataLogger/DataModeling removed in unified streaming architecture

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

    ESP_LOGI(TAG, "Application startup complete - core services running");

    // Keep app_main idle (this allows other tasks to continue running)
    vTaskDelete(nullptr);
}
