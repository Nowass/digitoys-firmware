#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "LiDARConfig.hpp"
#include "uart-hal.hpp"
#include "motor-hal.hpp"
#include "frame-parser.hpp"

static const char *TAG = "HelloWorld";

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

    lidar::UART_HAL uart_hal;
    lidar::MotorHAL motor;
    lidar::FrameParser parser;

    lidar::LiDARConfig cfg = {
        .uartPort = UART_NUM_1,
        .txPin = GPIO_NUM_10,
        .rxPin = GPIO_NUM_11,
        .dmaBufferLen = 512};

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

    uint8_t rx_buf[256];
    size_t out_len = 0;

    // while (true) {
    //     if (uart_hal.read(rx_buf, sizeof(rx_buf), out_len) == ESP_OK && out_len > 0) {
    //         ESP_LOGI(TAG, "Received %d bytes:", out_len);
    //         for (size_t i = 0; i < out_len; ++i) {
    //             printf("%02X ", rx_buf[i]);
    //         }
    //         printf("\n");
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(100));  // Read every 100ms (adjust as needed)
    // }

    while (true)
    {
        if (uart_hal.read(rx_buf, sizeof(rx_buf), out_len) == ESP_OK && out_len > 0)
        {
            // Optional raw dump
            // ESP_LOGI(TAG, "Received %d bytes:", out_len);
            // for (size_t i = 0; i < out_len; ++i) {
            //     printf("%02X ", rx_buf[i]);
            // }
            // printf("\n");

            // Parse
            auto frames = parser.parse(rx_buf, out_len);
            for (const auto &frame : frames)
            {
                char rpm_str[16];
                if (frame.rpm.has_value())
                {
                    snprintf(rpm_str, sizeof(rpm_str), "%u", frame.rpm.value());
                }
                else
                {
                    snprintf(rpm_str, sizeof(rpm_str), "N/A");
                }

                ESP_LOGI(TAG, "✅ Frame with %zu points (RPM: %s)", frame.points.size(), rpm_str);

                for (const auto &pt : frame.points)
                {
                    ESP_LOGI(TAG, "  ➤ Angle: %6.2f°, Dist: %4d mm, Conf: %3d",
                             pt.angle_deg, pt.distance_mm, pt.confidence);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust timing based on sensor rate
    }
}
