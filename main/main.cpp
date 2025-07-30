// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include "bmi270-simple.hpp" // Simplified BMI270 production driver
#include <limits>

static const char *TAG = "APP_MAIN";
using namespace lidar;

// Control task context
struct ControlContext
{
    lidar::LiDAR *lidar;
    adas::PwmDriver *pwm_driver;
    monitor::Monitor *mon;
};

// ControlTask: reads LiDAR frames and switches passthrough vs. brake
static void ControlTask(void *pv)
{
    auto *ctx = static_cast<ControlContext *>(pv);
    lidar::LiDAR &lidar = *ctx->lidar;
    adas::PwmDriver &driver = *ctx->pwm_driver;

    bool obstacle_state = false;
    bool warning_state = false;
    float last_distance = std::numeric_limits<float>::infinity();
    float slowdown_duty = 0.0f;

    constexpr float BRAKE = 0.06f;      // full brake duty
    constexpr float ZERO_SPEED = 0.09f; // neutral duty
    constexpr float DUTY_STEP = 0.005f;

    while (true)
    {
        auto info = lidar.getObstacleInfo();
        ctx->mon->updateTelemetry(info.obstacle, info.distance, driver.lastDuty(0), info.warning);
        if (driver.isThrottlePressed(0))
        {
            if (info.obstacle)
            {
                if (!obstacle_state)
                {
                    obstacle_state = true;
                    warning_state = false;
                    ESP_LOGW(TAG, "Obstacle! Applying brake duty");
                    driver.pausePassthrough(0);
                    driver.setDuty(0, BRAKE);
                }
            }
            else if (info.warning)
            {
                obstacle_state = false;
                if (!warning_state)
                {
                    warning_state = true;
                    slowdown_duty = driver.lastDuty(0);
                    last_distance = info.distance;
                    driver.pausePassthrough(0);
                    ESP_LOGI(TAG, "Warning detected. Starting slowdown");
                }
                else
                {
                    if (info.distance < last_distance && slowdown_duty > ZERO_SPEED)
                    {
                        slowdown_duty -= DUTY_STEP;
                        if (slowdown_duty < ZERO_SPEED)
                            slowdown_duty = ZERO_SPEED;
                        driver.setDuty(0, slowdown_duty);
                    }
                    last_distance = info.distance;
                }
            }
            else
            {
                if (obstacle_state || warning_state)
                {
                    obstacle_state = false;
                    warning_state = false;
                    ESP_LOGI(TAG, "Path clear. Resuming passthrough");
                    driver.resumePassthrough(0);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "=== DigiToys Firmware Starting ===");

    // --- BMI270 IMU Sensor with Consolidated I2C HAL ---
    ESP_LOGI(TAG, "Initializing BMI270 with consolidated I2C HAL and automatic priming...");

    static bmi270::BMI270 bmi270Sensor;
    esp_err_t bmi_result = bmi270::initProductionBMI270(bmi270Sensor);

    if (bmi_result == ESP_OK)
    {
        ESP_LOGI(TAG, "🎉 BMI270 production initialization SUCCESSFUL!");
        ESP_LOGI(TAG, "✓ Consolidated I2C HAL with priming enabled reliable sensor communication");

        // Validation: Read BMI270 chip ID to verify the consolidated approach worked
        uint8_t chipId;
        if (bmi270Sensor.readChipId(chipId) == ESP_OK)
        {
            ESP_LOGI(TAG, "✓ Post-initialization validation: BMI270 Chip ID = 0x%02X (expected: 0x24)", chipId);
            if (chipId == 0x24)
            {
                ESP_LOGI(TAG, "✓ Consolidated I2C HAL successfully enabled BMI270 communication!");
            }
            else
            {
                ESP_LOGW(TAG, "⚠ Unexpected chip ID - consolidated HAL may need adjustment");
            }
        }
        else
        {
            ESP_LOGW(TAG, "⚠ Could not read chip ID for validation");
        }

        ESP_LOGI(TAG, "✓ BMI270 is ready for integration with other systems");
    }
    else
    {
        ESP_LOGW(TAG, "❌ BMI270 production initialization FAILED: %s", esp_err_to_name(bmi_result));
        ESP_LOGW(TAG, "Consolidated I2C HAL may need further refinement");
        ESP_LOGW(TAG, "Continuing with other systems - BMI270 will be unavailable");
    }

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Consolidated I2C HAL Integration Completed ===");
    ESP_LOGI(TAG, "=== Proceeding with system integration ===");
    ESP_LOGI(TAG, "");

    // Continue with other systems - BMI270 priming sequence allows this now

    // --- LiDAR hardware setup ---
    LiDARConfig cfg = {
        .uartPort = UART_NUM_1,
        .txPin = GPIO_NUM_10,
        .rxPin = GPIO_NUM_11,
        .dmaBufferLen = 2048,
        .angleMinDeg = 347.5f,
        .angleMaxDeg = 12.5f,
        .motorPin = GPIO_NUM_6,
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

    // --- Launch ControlTask ---
    static ControlContext ctx = {&lidar, &pwm_driver, &mon};
    BaseType_t rc = xTaskCreate(
        ControlTask, "ControlTask", 8192,
        &ctx, tskIDLE_PRIORITY + 2, nullptr);
    ESP_ERROR_CHECK(rc == pdPASS ? ESP_OK : ESP_FAIL);

    // Keep app_main idle
    vTaskDelete(nullptr);
}
