#include "adas_pwm_driver.hpp"
#include <DigitoysCoreAll.hpp>
#include <Logger.hpp>

using namespace digitoys::core;
using namespace digitoys::constants;

namespace adas
{

    const char *PwmChannelConfig::TAG = "PwmChannelConfig";

    esp_err_t PwmChannelConfig::validate() const
    {
        // Register with centralized logging system (one-time registration)
        static bool registered = false;
        if (!registered) {
            DIGITOYS_REGISTER_COMPONENT("PwmChannelConfig", "CONFIG");
            registered = true;
        }

        // Validate GPIO pins
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateGpio(rx_gpio, "RMT input GPIO"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateGpioOutput(tx_gpio, "LEDC output GPIO"));

        // Ensure RX and TX pins are different
        if (rx_gpio == tx_gpio)
        {
            DIGITOYS_LOGE("PwmChannelConfig", "RX GPIO (%d) and TX GPIO (%d) cannot be the same", rx_gpio, tx_gpio);
            return ESP_ERR_INVALID_ARG;
        }

        // Validate LEDC configuration
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateLedcChannel(ledc_channel, "LEDC channel"));
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateLedcTimer(ledc_timer, "LEDC timer"));

        // Validate PWM frequency (typical RC frequencies: 50-200 Hz)
        DIGITOYS_ERROR_CHECK(TAG, ConfigValidator::validateFrequency(pwm_freq_hz, 20, 1000, "PWM frequency"));

        DIGITOYS_LOGI("PwmChannelConfig", "Configuration validation passed");
        return ESP_OK;
    }

    PwmChannelConfig PwmChannelConfig::getDefault()
    {
        PwmChannelConfig config;

        // Use safe default values - pins must be set by user
        config.rx_gpio = GPIO_NUM_NC;
        config.tx_gpio = GPIO_NUM_NC;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pwm_freq_hz = 62; // Standard RC frequency

        return config;
    }

    PwmChannelConfig PwmChannelConfig::createFromConstants()
    {
        PwmChannelConfig config;

        // Use centralized constants
        config.rx_gpio = pins::PWM_RX;
        config.tx_gpio = pins::PWM_TX;
        config.ledc_channel = hardware::PWM_OUTPUT_CHANNEL;
        config.ledc_timer = hardware::PWM_OUTPUT_TIMER;
        config.pwm_freq_hz = hardware::PWM_OUTPUT_FREQ_HZ;

        return config;
    }

    PwmChannelConfig PwmChannelConfig::createThrottleConfig()
    {
        auto config = createFromConstants();
        esp_err_t result = config.validate();
        digitoys::core::ComponentConfigFactory::validateConfigCreation("Throttle PWM Config", result);
        if (result == ESP_OK)
        {
            DIGITOYS_LOGI("PwmChannelConfig", "Created throttle config: RX:%d, TX:%d, CH:%d, TIMER:%d",
                          config.rx_gpio, config.tx_gpio, config.ledc_channel, config.ledc_timer);
        }
        return config;
    }

    PwmChannelConfig PwmChannelConfig::createSteeringConfig()
    {
        auto config = getDefault();

        // Configure for steering (future use)
        config.rx_gpio = GPIO_NUM_4; // Example steering input pin
        config.tx_gpio = GPIO_NUM_5; // Example steering output pin
        config.ledc_channel = LEDC_CHANNEL_2;
        config.ledc_timer = LEDC_TIMER_2;
        config.pwm_freq_hz = 62;

        esp_err_t result = config.validate();
        digitoys::core::ComponentConfigFactory::validateConfigCreation("Steering PWM Config", result);
        return config;
    }

} // namespace adas
