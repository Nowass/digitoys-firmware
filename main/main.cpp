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

// Control task context
struct ControlContext
{
    UART_HAL *uart_hal;
    FramePraser *parser;
    adas::PwmDriver *pwm_driver;
};

// ControlTask: reads LiDAR frames and switches passthrough vs. brake
static void ControlTask(void *pv)
{
    auto *ctx = static_cast<ControlContext *>(pv);
    UART_HAL &uart = *ctx->uart_hal;
    FramePraser &parser = *ctx->parser;
    adas::PwmDriver &driver = *ctx->pwm_driver;
    uint8_t buffer[BUF_SIZE];

    bool obstacle_state = false;
    constexpr float BRAKE = 0.12f;
    constexpr float OBSTACLE_THRESHOLD = 0.8f;

    while (true)
    {
        int len = uart_read_bytes(UART_NUM_1, buffer, BUF_SIZE, pdMS_TO_TICKS(20));
        if (len > 0)
        {
            parser.CommReadCallback(reinterpret_cast<const char *>(buffer), len);
            if (parser.IsFrameReady())
            {
                auto frame = parser.GetLaserScanData();
                parser.ResetFrameReady();

                // find closest obstacle
                float closest = std::numeric_limits<float>::infinity();
                for (auto &pt : frame)
                {
                    if (pt.intensity < 100)
                        continue;
                    float angle = pt.angle;
                    if (!(angle < 12.0f || angle > 347.0f))
                        continue;
                    float d = pt.distance / 1000.0f;
                    closest = std::min(closest, d);
                }

                bool obstacle = (closest <= OBSTACLE_THRESHOLD);
                if (obstacle && !obstacle_state)
                {
                    obstacle_state = true;
                    ESP_LOGW(TAG, "Obstacle! Applying brake duty");
                    driver.pausePassthrough(0);
                    driver.setDuty(0, BRAKE);
                }
                else if (!obstacle && obstacle_state)
                {
                    obstacle_state = false;
                    ESP_LOGI(TAG, "Obstacle cleared. Resuming passthrough");
                    driver.resumePassthrough(0);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main()
{
    // --- LiDAR hardware setup ---
    static UART_HAL uart_hal;
    static Motor_HAL motor;
    static FramePraser parser;
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
    adas::PwmChannelConfig thr_cfg = {GPIO_NUM_18, GPIO_NUM_19,
                                      LEDC_CHANNEL_0, LEDC_TIMER_0, 62};
    std::vector<adas::PwmChannelConfig> configs = {thr_cfg};
    static adas::PwmDriver pwm_driver{configs};
    ESP_ERROR_CHECK(pwm_driver.initialize());
    ESP_LOGI(TAG, "PWM passthrough running");

    // --- Launch ControlTask ---
    static ControlContext ctx = {&uart_hal, &parser, &pwm_driver};
    BaseType_t rc = xTaskCreate(
        ControlTask, "ControlTask", 8192,
        &ctx, tskIDLE_PRIORITY + 2, nullptr);
    ESP_ERROR_CHECK(rc == pdPASS ? ESP_OK : ESP_FAIL);

    // Keep app_main idle
    vTaskDelete(nullptr);
}
