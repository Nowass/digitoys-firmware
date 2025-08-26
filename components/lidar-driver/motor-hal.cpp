#include "motor-hal.hpp"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <cmath>

namespace lidar
{

    static uint32_t duty_from_percent(unsigned bits, float pct)
    {
        const uint32_t max = (1u << bits) - 1u;
        if (pct < 0)
            pct = 0;
        if (pct > 100)
            pct = 100;
        return lroundf(max * (pct / 100.f));
    }
    namespace
    {
        constexpr const char *TAG = "Motor_HAL";
        constexpr int kLEDC_FreqHz_Default = 50000;
    }

    Motor_HAL::~Motor_HAL()
    {
        deinit();
    }

    esp_err_t Motor_HAL::init(const LiDARConfig &cfg)
    {
        gpioPin_ = cfg.motorPin;
        pwmChannel_ = cfg.motorChannel;
        pwmDuty_ = cfg.motorDutyPct;
        isInitialized_ = false;

        if (gpioPin_ < 0)
        {
            ESP_LOGE(TAG, "Invalid motor GPIO pin: %d", gpioPin_);
            return ESP_ERR_INVALID_ARG;
        }

        // Configure LEDC timer
        ledc_timer_config_t timerCfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = static_cast<ledc_timer_bit_t>(pwmResolutionBits_),
            .timer_num = LEDC_TIMER_0,
            .freq_hz = cfg.motorFreqHz,
            .clk_cfg = LEDC_USE_PLL_DIV_CLK,
            .deconfigure = false,
        };

        esp_err_t err = ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&timerCfg));

        // Configure LEDC channel
        ledc_channel_config_t channelCfg = {
            .gpio_num = gpioPin_,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = pwmChannel_,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = duty_from_percent(pwmResolutionBits_, 50.0f), // hold ~50% duty for ~300 ms to enter external mode
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags = {.output_invert = 0},
        };

        err = ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&channelCfg));

        isInitialized_ = true;
        ESP_LOGI(TAG, "Motor HAL initialized on pin %d", gpioPin_);
        return err;
    }

    esp_err_t Motor_HAL::start()
    {
        if (!isInitialized_)
            return ESP_ERR_INVALID_STATE;

        const int maxDuty = (1 << pwmResolutionBits_) - 1;
        const int dutyVal = (pwmDuty_ * maxDuty) / 100;

        esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel_, dutyVal);
        if (err != ESP_OK)
            return err;

        return ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel_);
    }

    esp_err_t Motor_HAL::stop()
    {
        if (!isInitialized_)
            return ESP_ERR_INVALID_STATE;

        esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel_, 0);
        if (err != ESP_OK)
            return err;

        return ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel_);
    }

    void Motor_HAL::deinit()
    {
        if (isInitialized_)
        {
            stop();
            isInitialized_ = false;
        }
    }

} // namespace lidar
