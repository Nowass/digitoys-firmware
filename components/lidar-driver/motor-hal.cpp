#include "motor-hal.hpp"
#include <driver/gpio.h>
#include <esp_log.h>

namespace lidar
{

    namespace
    {
        constexpr const char *TAG = "Motor_HAL";
    }

    Motor_HAL::~Motor_HAL()
    {
        deinit();
    }

    esp_err_t Motor_HAL::init(const LiDARConfig &cfg)
    {
        gpioPin_ = cfg.motorPin;

        if (gpioPin_ < 0)
        {
            ESP_LOGE(TAG, "Invalid motor GPIO pin: %d", gpioPin_);
            return ESP_ERR_INVALID_ARG;
        }

        gpio_config_t ioConf = {
            .pin_bit_mask = 1ULL << gpioPin_,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        const esp_err_t err = gpio_config(&ioConf);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "GPIO config failed: %d", err);
            return err;
        }

        isInitialized_ = true;
        ESP_LOGI(TAG, "Motor HAL initialized on pin %d", gpioPin_);
        return start(); // Optional: spin motor by default
    }

    esp_err_t Motor_HAL::start()
    {
        if (!isInitialized_)
            return ESP_ERR_INVALID_STATE;
        return gpio_set_level(static_cast<gpio_num_t>(gpioPin_), 1);
    }

    esp_err_t Motor_HAL::stop()
    {
        if (!isInitialized_)
            return ESP_ERR_INVALID_STATE;
        return gpio_set_level(static_cast<gpio_num_t>(gpioPin_), 0);
    }

    void Motor_HAL::deinit()
    {
        if (isInitialized_)
        {
            stop();
            gpio_reset_pin(static_cast<gpio_num_t>(gpioPin_));
            isInitialized_ = false;
        }
    }

} // namespace lidar
