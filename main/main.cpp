// app_main.cpp
#include "adas_pwm_driver.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "app_main";

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting PWM passthrough");

    // 1. Configure only throttle passthrough (RX -> TX)
    adas::PwmChannelConfig throttle_cfg = {
        .rx_gpio = GPIO_NUM_18,
        .tx_gpio = GPIO_NUM_19,
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pwm_freq_hz = 62};
    std::vector<adas::PwmChannelConfig> configs;
    configs.push_back(throttle_cfg);

    // 2. Instantiate and start driver
    adas::PwmDriver pwm_driver{configs};
    ESP_ERROR_CHECK(pwm_driver.initialize());

    ESP_LOGI(TAG, "PWM passthrough running. Input on GPIO %d -> Output on GPIO %d",
             throttle_cfg.rx_gpio, throttle_cfg.tx_gpio);

    // 3. Keep app_main alive; all work happens in RMT task
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
