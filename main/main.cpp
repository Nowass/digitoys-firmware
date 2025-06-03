#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "LiDARConfig.hpp"
#include "uart-hal.hpp"
#include "motor-hal.hpp"
#include "lidar-frame-parser.hpp"

#define BUF_SIZE 512

static const char *TAG = "HelloWorld";

using namespace lidar;

void hello_task(void *pvParameter)
{
    while (1)
    {
        // Get the current time since startup in milliseconds
        uint32_t uptime = esp_log_timestamp();

        ESP_LOGI(TAG, "Hello, World! System uptime: %lu ms", uptime);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Delay for 2000 ms (2 seconds)
    }
}

__attribute__((optimize("O0"))) void analyzeObstacles(const std::vector<lidar::PointData> &frame,
                                                      float crashThreshold, float obstacleThreshold, float noObstacleThreshold)
{
    float sumLeft = 0, sumFront = 0, sumRight = 0;
    int countLeft = 0, countFront = 0, countRight = 0;
    static bool printedOnce = false;

    bool crashDetected = false;
    float closestDist = std::numeric_limits<float>::max();

    for (const auto &pt : frame)
    {
        float angle = pt.angle;
        float dist = pt.distance / 1000.0f; // mm → m

        if (pt.intensity < 100)
            continue;

        if (!(angle < 12.0f || angle > 347.0f))
            continue;

        if (dist <= crashThreshold)
        {
            crashDetected = true;
        }

        if (dist < closestDist)
            closestDist = dist;

        if (angle >= 360.0f)
            angle -= 360.0f;
        if (angle < 0.0f)
            angle += 360.0f;

        if ((angle >= 347.0f && angle <= 356.0f))
        {
            sumLeft += dist;
            countLeft++;
        }
        else if ((angle > 356.0f && angle < 360.0f) || (angle >= 0.0f && angle <= 4.0f))
        {
            sumFront += dist;
            countFront++;
        }
        else if ((angle > 4.0f && angle <= 12.0f))
        {
            sumRight += dist;
            countRight++;
        }
    }

    if (crashDetected)
    {
        ESP_LOGW("OBSTACLE", "⚠️ CRASH DETECTED! Obstacle too close!");
        printedOnce = false;
        return;
    }

    if (closestDist <= obstacleThreshold)
    {
        bool printed = false;

        if (countLeft > 0 && (sumLeft / countLeft) <= obstacleThreshold)
        {
            ESP_LOGI("OBSTACLE", "Obstacle LEFT at %.2f m", sumLeft / countLeft);
            printed = true;
        }
        if (countFront > 0 && (sumFront / countFront) <= obstacleThreshold)
        {
            ESP_LOGI("OBSTACLE", "Obstacle FRONT at %.2f m", sumFront / countFront);
            printed = true;
        }
        if (countRight > 0 && (sumRight / countRight) <= obstacleThreshold)
        {
            ESP_LOGI("OBSTACLE", "Obstacle RIGHT at %.2f m", sumRight / countRight);
            printed = true;
        }

        if (printed)
            printedOnce = false;
    }
    else if (closestDist >= noObstacleThreshold)
    {
        if (!printedOnce)
        {
            ESP_LOGI("OBSTACLE", "✅ No obstacle in range");
            printedOnce = true;
        }
    }
}

extern "C" void app_main(void)
{
    static const char *TAG = "APP_MAIN";

    UART_HAL uart_hal;
    Motor_HAL motor;
    FramePraser parser;
    uint8_t data[BUF_SIZE];

    LiDARConfig cfg = {
        .uartPort = UART_NUM_1,
        .txPin = GPIO_NUM_10,
        .rxPin = GPIO_NUM_11,
        .dmaBufferLen = 2048,
        .angleMinDeg = 12.5f,
        .angleMaxDeg = 347.5f,
        .motorPin = GPIO_NUM_4,
        .motorChannel = LEDC_CHANNEL_0,
        .motorFreqHz = 50000,
        .motorDutyPct = 50};

    esp_err_t err = uart_hal.init(cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART_HAL init failed");
        return;
    }

    ESP_LOGI(TAG, "UART_HAL initialized successfully");

    // Initialize motor with 50 kHz PWM and 50% duty
    err = motor.init(cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor init failed");
        return;
    }

    // Start the motor
    err = motor.start();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor start failed");
        return;
    }

    ESP_LOGI(TAG, "Motor started");

    while (true)
    {
        // Blocking read from UART (you may want to use ringbuffer for DMA)
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, pdMS_TO_TICKS(20));
        if (len > 0)
        {
            parser.CommReadCallback((const char *)data, len);

            if (parser.IsFrameReady())
            {
                Points2D frame = parser.GetLaserScanData();
                parser.ResetFrameReady();

                // Example thresholds: crash @ 0.3m, warn @ 0.8m, ok above 1.2m
                analyzeObstacles(frame, 0.3f, 0.8f, 1.2f);

                // Use the frame (e.g., find nearest obstacle, draw scan, etc.)
                // for (const auto &pt : frame)
                // {
                //     if (!(pt.angle < cfg.angleMinDeg || pt.angle > cfg.angleMaxDeg))
                //         continue;

                //     ESP_LOGI(TAG, "Angle: %.1f°, Distance: %.2f m, Confidence: %d",
                //              pt.angle, pt.distance / 1000.0, pt.intensity);
                // }
            }
        }
    }
}
