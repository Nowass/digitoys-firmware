// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include "mpu6050_example.hpp"
#include <limits>

static const char *TAG = "APP_MAIN";
using namespace lidar;

// Control task context
struct ControlContext
{
    lidar::LiDAR *lidar;
    adas::PwmDriver *pwm_driver;
    monitor::Monitor *mon;
    MPU6050Example::Example *mpu6050; // Add MPU6050 reference
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
        
        // Use raw acceleration magnitude as motion indicator instead of velocity
        // This avoids the drift problem in velocity integration
        float motionIntensity = abs(ctx->mpu6050->getCurrentAcceleration());
        
        ctx->mon->updateTelemetry(info.obstacle, info.distance, motionIntensity, info.warning);
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

    // --- MPU6050 Sensor initialization ---
    static MPU6050Example::Example mpu6050_example;
    
    ESP_LOGI(TAG, "Starting MPU6050 sensor example...");
    
    // Initialize the MPU6050 sensor
    esp_err_t ret = mpu6050_example.initialize();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6050 sensor: %s", esp_err_to_name(ret));
        // Continue without MPU6050 for now
    }
    else
    {
        // Perform self-test
        ret = mpu6050_example.self_test();
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "MPU6050 self-test failed, continuing anyway...");
        }

        // Calibrate the sensor
        ret = mpu6050_example.calibrate();
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "MPU6050 calibration failed, continuing anyway...");
        }
        
        ESP_LOGI(TAG, "MPU6050 sensor initialized successfully");
    }

    // --- MPU6050 Sensor Task ---
    static auto mpu6050_task = [](void *params)
    {
        MPU6050Example::Example* mpu6050_ptr = static_cast<MPU6050Example::Example*>(params);

        // Run continuous sensor data reading
        while (true)
        {
            esp_err_t ret = mpu6050_ptr->read_and_log_data();
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to read MPU6050 data: %s", esp_err_to_name(ret));
            }

            // Check for automotive events that require immediate attention
            if (mpu6050_ptr->isImpactDetected())
            {
                ESP_LOGE(TAG, "COLLISION DETECTED! Peak acceleration: %.2f m/s²",
                         mpu6050_ptr->getPeakAcceleration());
                // Here you could trigger emergency systems, airbags, etc.
            }

            if (mpu6050_ptr->isHardBrakingDetected())
            {
                ESP_LOGW(TAG, "HARD BRAKING DETECTED! Deceleration: %.2f m/s²",
                         -mpu6050_ptr->getCurrentAcceleration());
                // Here you could trigger brake assist, hazard lights, etc.
            }

            // Reset impact detection if needed (e.g., after handling the event)
            // mpu6050_ptr->resetImpactDetection();

            vTaskDelay(pdMS_TO_TICKS(50)); // Read every 50ms for real-time response
        }
    };

    BaseType_t mpu6050_rc = xTaskCreate(
        mpu6050_task, "MPU6050Task", 4096,
        &mpu6050_example, tskIDLE_PRIORITY + 1, nullptr);
    ESP_ERROR_CHECK(mpu6050_rc == pdPASS ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "MPU6050 sensor task started");

    // --- Launch ControlTask ---
    static ControlContext ctx = {&lidar, &pwm_driver, &mon, &mpu6050_example};
    BaseType_t rc = xTaskCreate(
        ControlTask, "ControlTask", 8192,
        &ctx, tskIDLE_PRIORITY + 2, nullptr);
    ESP_ERROR_CHECK(rc == pdPASS ? ESP_OK : ESP_FAIL);

    // Keep app_main idle
    vTaskDelete(nullptr);
}
