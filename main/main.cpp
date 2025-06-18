// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "uart-hal.hpp"
#include "motor-hal.hpp"
#include "frame-parser.hpp"

static const char *TAG = "APP_MAIN";
#define BUF_SIZE 512

using namespace lidar;

extern "C" __attribute__((optimize("O0"))) void app_main()
{
    // --- LiDAR setup ---
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
    ESP_ERROR_CHECK(uart_hal.init(cfg));
    ESP_ERROR_CHECK(motor.init(cfg));
    ESP_ERROR_CHECK(motor.start());
    ESP_LOGI(TAG, "LiDAR motor started");

    // --- PWM passthrough driver (throttle only) ---
    adas::PwmChannelConfig throttle_cfg = {
        .rx_gpio = GPIO_NUM_18,
        .tx_gpio = GPIO_NUM_19,
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pwm_freq_hz = 62};
    std::vector<adas::PwmChannelConfig> configs = {throttle_cfg};
    adas::PwmDriver pwm_driver{configs};
    ESP_ERROR_CHECK(pwm_driver.initialize());
    ESP_LOGI(TAG, "PWM passthrough running");

    // Control flags
    bool obstacle_state = false;
    static constexpr float BRAKE = 0.12f;
    static constexpr float CRASH_THRESHOLD = 0.3f;
    static constexpr float OBSTACLE_THRESHOLD = 0.8f;
    static constexpr float NO_OBSTACLE_THRESHOLD = 1.2f;

    // Main loop: read LiDAR + control passthrough or brake
    while (true)
    {
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, pdMS_TO_TICKS(20));
        if (len > 0)
        {
            parser.CommReadCallback((const char *)data, len);
            if (parser.IsFrameReady())
            {
                // Simplest: check closest distance
                auto frame = parser.GetLaserScanData();
                parser.ResetFrameReady();

                float closest = std::numeric_limits<float>::infinity();
                for (auto &pt : frame)
                {
                    float angle = pt.angle;

                    if (pt.intensity < 100)
                        continue;

                    if (!(angle < 12.0f || angle > 347.0f))
                        continue;

                    if (angle >= 360.0f)
                        angle -= 360.0f;
                    if (angle < 0.0f)
                        angle += 360.0f;

                    float d = pt.distance / 1000.0f;
                    if (d < closest)
                        closest = d;
                }

                bool obstacle = (closest <= OBSTACLE_THRESHOLD);

                if (obstacle && !obstacle_state)
                {
                    // obstacle just detected: stop passthrough & brake
                    obstacle_state = true;
                    ESP_LOGW(TAG, "Obstacle! Applying brake duty");
                    ESP_ERROR_CHECK(pwm_driver.pausePassthrough(0));
                    pwm_driver.setDuty(0, BRAKE);
                }
                else if (!obstacle && obstacle_state)
                {
                    // obstacle cleared: resume passthrough
                    obstacle_state = false;
                    ESP_LOGI(TAG, "Obstacle cleared. Resuming passthrough");
                    ESP_ERROR_CHECK(pwm_driver.resumePassthrough(0));
                }
            }
        }
    }
}
