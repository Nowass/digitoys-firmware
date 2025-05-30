#include "motor-hal.hpp"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"

#ifndef SOC_LEDC_SUPPORT_HS_MODE
#define SOC_LEDC_SUPPORT_HS_MODE 0
#endif


static constexpr const char* TAG = "MotorHAL";

namespace lidar {

esp_err_t MotorHAL::init(ledc_channel_t channel,
                         gpio_num_t pin,
                         uint32_t freq_hz,
                         uint8_t duty_pct)
{
    _channel = channel;
    _pin     = pin;

    // Configure timer as before...
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed");
        return err;
    }

    // Compute duty as uint32_t to avoid narrowing
    uint32_t duty_val = (static_cast<uint32_t>(duty_pct) * ((1 << 8) - 1)) / 100;

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = duty_val,  // now no narrowing
        .hpoint     = 0
    };
    err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed");
    }
    return err;
}



esp_err_t MotorHAL::start() {
    // LEDC starts output immediately after channel_config; nothing else needed
    return ESP_OK;
}

esp_err_t MotorHAL::stop() {
    // Set duty to zero
    return ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, 0) == ESP_OK
         ? ESP_OK
         : ESP_FAIL;
}

esp_err_t MotorHAL::setSpeed(uint8_t duty_pct) {
    if (duty_pct > 100) duty_pct = 100;
    uint32_t duty = (duty_pct * ((1 << 8) - 1)) / 100;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "setSpeed failed");
        return err;
    }
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel);
}

} // namespace lidar