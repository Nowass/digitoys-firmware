// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include "BMI270.hpp"
#include <limits>

static const char *TAG = "APP_MAIN";
using namespace lidar;

// BMI270 I2C bus and sensor instance (GPIO4 = SDA, GPIO5 = SCL)
static I2C::Config bmiBusCfg{I2C_NUM_0, GPIO_NUM_4, GPIO_NUM_5, 400000};
static I2C bmiBus{bmiBusCfg};
static BMI270::Config bmiCfg{};
static BMI270 accel{bmiBus, bmiCfg};

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

// Simple task to validate BMI270 functionality after power-up
static void BmiValidationTask(void *)
{
    ESP_LOGI("BMI270", "Initializing BMI270");
    if (!accel.init())
    {
        ESP_LOGE("BMI270", "Initialization failed");
        vTaskDelete(nullptr);
    }
    ESP_LOGI("BMI270", "Initialization successful");

    for (int i = 0; i < 10; ++i)
    {
        if (accel.dataReady())
        {
            float ax = accel.getAccelLongitudinal();
            ESP_LOGI("BMI270", "Accel X = %.2f m/s^2", ax);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI("BMI270", "Validation finished");
    vTaskDelete(nullptr);
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

    // --- Launch BMI270 validation task ---
    xTaskCreate(BmiValidationTask, "BMI270Test", 4096, nullptr,
                tskIDLE_PRIORITY + 1, nullptr);

    // --- Launch ControlTask ---
    static ControlContext ctx = {&lidar, &pwm_driver, &mon};
    BaseType_t rc = xTaskCreate(
        ControlTask, "ControlTask", 8192,
        &ctx, tskIDLE_PRIORITY + 2, nullptr);
    ESP_ERROR_CHECK(rc == pdPASS ? ESP_OK : ESP_FAIL);

    // Keep app_main idle
    vTaskDelete(nullptr);
}
