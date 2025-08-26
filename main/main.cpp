// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include "ControlTask.hpp"

static const char *TAG = "APP_MAIN";
using namespace lidar;
extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting DigiToys firmware with unified configuration system");

    // --- LiDAR hardware setup using configuration factory ---
    auto lidar_config = lidar::LiDARConfig::createProductionConfig();
    static lidar::LiDAR lidar{lidar_config};
    ESP_ERROR_CHECK(lidar.initialize());
    ESP_LOGI(TAG, "LiDAR initialized with validated configuration");

    // --- PWM passthrough driver using configuration factory ---
    auto throttle_config = adas::PwmChannelConfig::createThrottleConfig();
    std::vector<adas::PwmChannelConfig> configs = {throttle_config};
    static adas::PwmDriver pwm_driver{configs};
    ESP_ERROR_CHECK(pwm_driver.initialize());
    ESP_LOGI(TAG, "PWM passthrough running");

    // --- Telemetry monitor ---
    static monitor::Monitor mon;
    ESP_ERROR_CHECK(mon.start());
    ESP_LOGI(TAG, "Monitor started");

    // --- System monitor ---
    static monitor::SystemMonitor sys_mon;
    ESP_ERROR_CHECK(sys_mon.start());
    ESP_LOGI(TAG, "System monitor started");

    // --- Launch ControlTask ---
    static control::ControlContext ctx = {&lidar, &pwm_driver, &mon};
    static control::ControlTask control_task(&ctx);

    BaseType_t rc = xTaskCreate(
        ControlTaskWrapper, "ControlTask", 8192,
        &control_task, tskIDLE_PRIORITY + 2, nullptr);
    ESP_ERROR_CHECK(rc == pdPASS ? ESP_OK : ESP_FAIL);

    // Keep app_main idle
    vTaskDelete(nullptr);
}
