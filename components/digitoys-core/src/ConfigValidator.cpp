#include "ConfigValidator.hpp"
#include "ComponentError.hpp"
#include "Logger.hpp"
#include <soc/gpio_num.h>

namespace digitoys::core
{

    const char *ConfigValidator::TAG = "ConfigValidator";

    esp_err_t ConfigValidator::validateGpio(gpio_num_t pin, const char *context)
    {
        // Register with centralized logging system (one-time registration)
        static bool registered = false;
        if (!registered) {
            DIGITOYS_REGISTER_COMPONENT("ConfigValidator", "CONFIG");
            registered = true;
        }

        if (pin < 0 || pin >= GPIO_NUM_MAX)
        {
            DIGITOYS_LOGE("ConfigValidator", "Invalid GPIO pin %d for %s (valid range: 0-%d)",
                          pin, context, GPIO_NUM_MAX - 1);
            return ESP_ERR_INVALID_ARG;
        }

        // Check for pins that cannot be used
        if (pin == GPIO_NUM_NC)
        {
            DIGITOYS_LOGE("ConfigValidator", context);
            return ESP_ERR_INVALID_ARG;
        }

        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateGpioOutput(gpio_num_t pin, const char *context)
    {
        DIGITOYS_ERROR_CHECK(TAG, validateGpio(pin, context));

        // Additional checks for output pins could go here
        // (e.g., checking if pin supports output functionality)

        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateRange(float value, float min_val, float max_val, const char *context)
    {
        if (value < min_val || value > max_val)
        {
            DIGITOYS_LOGE("ConfigValidator", "Value %.3f out of range for %s (valid range: %.3f-%.3f)",
                          value, context, min_val, max_val);
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateRange(int value, int min_val, int max_val, const char *context)
    {
        if (value < min_val || value > max_val)
        {
            DIGITOYS_LOGE("ConfigValidator", "Value %d out of range for %s (valid range: %d-%d)",
                          value, context, min_val, max_val);
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateUartPort(uart_port_t port, const char *context)
    {
        if (port < UART_NUM_0 || port >= UART_NUM_MAX)
        {
            DIGITOYS_LOGE("ConfigValidator", "Invalid UART port %d for %s (valid range: %d-%d)",
                          port, context, UART_NUM_0, UART_NUM_MAX - 1);
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateLedcChannel(ledc_channel_t channel, const char *context)
    {
        if (channel < LEDC_CHANNEL_0 || channel >= LEDC_CHANNEL_MAX)
        {
            DIGITOYS_LOGE("ConfigValidator", "Invalid LEDC channel %d for %s (valid range: %d-%d)",
                          channel, context, LEDC_CHANNEL_0, LEDC_CHANNEL_MAX - 1);
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateLedcTimer(ledc_timer_t timer, const char *context)
    {
        if (timer < LEDC_TIMER_0 || timer >= LEDC_TIMER_MAX)
        {
            DIGITOYS_LOGE("ConfigValidator", "Invalid LEDC timer %d for %s (valid range: %d-%d)",
                          timer, context, LEDC_TIMER_0, LEDC_TIMER_MAX - 1);
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateFrequency(uint32_t freq_hz, uint32_t min_freq, uint32_t max_freq, const char *context)
    {
        if (freq_hz < min_freq || freq_hz > max_freq)
        {
            DIGITOYS_LOGE("ConfigValidator", "Frequency %lu Hz out of range for %s (valid range: %lu-%lu Hz)",
                          freq_hz, context, min_freq, max_freq);
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    }

    esp_err_t ConfigValidator::validateDutyPercent(int duty_pct, const char *context)
    {
        return validateRange(duty_pct, 0, 100, context);
    }

    esp_err_t ConfigValidator::validateNormalizedDuty(float duty, const char *context)
    {
        return validateRange(duty, 0.0f, 1.0f, context);
    }

    esp_err_t ConfigValidator::validateBufferSize(size_t size, size_t min_size, size_t max_size, const char *context)
    {
        if (size < min_size || size > max_size)
        {
            DIGITOYS_LOGE("ConfigValidator", "Buffer size %zu out of range for %s (valid range: %zu-%zu)",
                          size, context, min_size, max_size);
            return ESP_ERR_INVALID_ARG;
        }
        return ESP_OK;
    }

} // namespace digitoys::core
