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

extern "C" void app_main(void)
{
    static const char *TAG = "APP_MAIN";

    UART_HAL uart_hal;
    MotorHAL motor;
    FramePraser parser;
    uint8_t data[BUF_SIZE];

    LiDARConfig cfg = {
        .uartPort = UART_NUM_1,
        .txPin = GPIO_NUM_10,
        .rxPin = GPIO_NUM_11,
        .dmaBufferLen = 2048,
        .angleMinDeg = 0.0f,
        .angleMaxDeg = 360.0f,
        .motorPin = GPIO_NUM_4,
        .motorChannel = LEDC_CHANNEL_0,
        .motorFreqHz = 50000,
        .motorDutyPct = 50};

    // Example GPIO pin — replace with your actual motor control pin
    gpio_num_t motor_pin = GPIO_NUM_4;

    // LEDC channel 0 is usually free — but check your board usage
    ledc_channel_t motor_channel = LEDC_CHANNEL_0;

    esp_err_t err = uart_hal.init(cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART_HAL init failed");
        return;
    }

    ESP_LOGI(TAG, "UART_HAL initialized successfully");

    // Initialize motor with 50 kHz PWM and 50% duty
    err = motor.init(motor_channel, motor_pin, 50000 /*Hz*/, 50 /*%*/);
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

                // Use the frame (e.g., find nearest obstacle, draw scan, etc.)
                for (const auto &pt : frame)
                {
                    ESP_LOGI(TAG, "Angle: %.1f°, Distance: %.2f m, Confidence: %d",
                             pt.angle, pt.distance / 1000.0, pt.intensity);
                }
            }
        }
    }
}
