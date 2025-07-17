// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include "I2C.hpp"
#include "FrontAssist.hpp"
#include "Actuator.hpp"
#include <limits>

static const char *TAG = "APP_MAIN";
using namespace lidar;

static void ControlTaskEntry(void *pv)
{
    static_cast<ControlTask *>(pv)->run();
}

extern "C" void app_main()
{
    // --- LiDAR hardware setup ---
    LiDARConfig cfg = {
        .uartPort = UART_NUM_1,
        .txPin = GPIO_NUM_10,
        .rxPin = GPIO_NUM_11,
        .dmaBufferLen = 2048,
        .angleMinDeg = 347.5f,
        .angleMaxDeg = 12.5f,
        .motorPin = GPIO_NUM_4,
        .motorChannel = LEDC_CHANNEL_0,
        .motorFreqHz = 50000,
        .motorDutyPct = 50};
    static lidar::LiDAR lidar{cfg};
    ESP_ERROR_CHECK(lidar.initialize());
    ESP_LOGI(TAG, "LiDAR initialized");

    // --- PWM passthrough driver (throttle only) ---
    adas::PwmChannelConfig thr_cfg = {GPIO_NUM_18, GPIO_NUM_19,
                                      LEDC_CHANNEL_0, LEDC_TIMER_0, 62};
    std::vector<adas::PwmChannelConfig> configs = {thr_cfg};
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

    // --- I2C bus for BMI270 ---
    I2C::Config i2c_cfg{I2C_NUM_0, GPIO_NUM_4, GPIO_NUM_5, 400000};
    static I2C i2c_bus{i2c_cfg};

    // --- Actuator wrapper ---
    static Actuator actuator{pwm_driver};

    // --- Launch ControlTask ---
    static ControlTask control{i2c_bus, lidar, actuator, &mon};
    BaseType_t rc = xTaskCreate(
        ControlTaskEntry, "ControlTask", 8192,
        &control, tskIDLE_PRIORITY + 2, nullptr);
    ESP_ERROR_CHECK(rc == pdPASS ? ESP_OK : ESP_FAIL);

    // Keep app_main idle
    vTaskDelete(nullptr);
}
