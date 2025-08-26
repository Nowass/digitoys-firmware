#include "component_config.hpp"
#include "digitoys_constants.hpp"
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <sstream>

namespace digitoys
{
    namespace config
    {

        // =============================================================================
        // ConfigValidator Template Specializations
        // =============================================================================

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateRange(float value, float min_val, float max_val, const char *param_name)
        {
            if (value < min_val || value > max_val)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' value %.3f is out of range [%.3f, %.3f]",
                         param_name, value, min_val, max_val);
                return ESP_ERR_INVALID_ARG;
            }
            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateRange(int value, int min_val, int max_val, const char *param_name)
        {
            if (value < min_val || value > max_val)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' value %d is out of range [%d, %d]",
                         param_name, value, min_val, max_val);
                return ESP_ERR_INVALID_ARG;
            }
            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateGpio(gpio_num_t pin, const char *param_name, bool allow_nc)
        {
            if (pin == GPIO_NUM_NC && allow_nc)
            {
                return ESP_OK;
            }

            if (pin < GPIO_NUM_0 || pin >= GPIO_NUM_MAX)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' invalid GPIO pin number: %d", param_name, pin);
                return ESP_ERR_INVALID_ARG;
            }

            // Additional ESP32-C6 specific validation could be added here
            // For now, basic range check is sufficient
            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateUartPort(uart_port_t port, const char *param_name)
        {
            if (port < UART_NUM_0 || port >= UART_NUM_MAX)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' invalid UART port: %d", param_name, port);
                return ESP_ERR_INVALID_ARG;
            }
            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateLedcChannel(ledc_channel_t channel, const char *param_name)
        {
            if (channel >= LEDC_CHANNEL_MAX)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' invalid LEDC channel: %d", param_name, channel);
                return ESP_ERR_INVALID_ARG;
            }
            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateLedcTimer(ledc_timer_t timer, const char *param_name)
        {
            if (timer >= LEDC_TIMER_MAX)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' invalid LEDC timer: %d", param_name, timer);
                return ESP_ERR_INVALID_ARG;
            }
            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateString(const std::string &str, const char *param_name, size_t max_length)
        {
            if (str.empty())
            {
                ESP_LOGE("CONFIG", "Parameter '%s' cannot be empty", param_name);
                return ESP_ERR_INVALID_ARG;
            }

            if (max_length > 0 && str.length() > max_length)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' string too long: %zu > %zu",
                         param_name, str.length(), max_length);
                return ESP_ERR_INVALID_ARG;
            }

            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateAngleRange(float min_angle, float max_angle, const char *param_name)
        {
            // Normalize angles to [0, 360) range
            auto normalizeAngle = [](float angle) -> float
            {
                while (angle < 0.0f)
                    angle += 360.0f;
                while (angle >= 360.0f)
                    angle -= 360.0f;
                return angle;
            };

            float norm_min = normalizeAngle(min_angle);
            float norm_max = normalizeAngle(max_angle);

            // For angle ranges, we allow wrap-around (e.g., 350° to 10°)
            // so we don't enforce min < max in the traditional sense

            ESP_LOGI("CONFIG", "Parameter '%s' angle range: %.1f° to %.1f° (normalized: %.1f° to %.1f°)",
                     param_name, min_angle, max_angle, norm_min, norm_max);

            return ESP_OK;
        }

        template <typename ConfigType>
        esp_err_t ConfigValidator<ConfigType>::validateFrequency(uint32_t frequency, uint32_t min_freq, uint32_t max_freq, const char *param_name)
        {
            if (frequency < min_freq || frequency > max_freq)
            {
                ESP_LOGE("CONFIG", "Parameter '%s' frequency %u Hz is out of range [%u, %u] Hz",
                         param_name, frequency, min_freq, max_freq);
                return ESP_ERR_INVALID_ARG;
            }
            return ESP_OK;
        }

        // =============================================================================
        // ConfigBuilder Template Implementation
        // =============================================================================

        template <typename ConfigType>
        ConfigType ConfigBuilder<ConfigType>::build()
        {
            esp_err_t result = config_.validate();
            if (result != ESP_OK)
            {
                throw std::runtime_error("Configuration validation failed: " + std::to_string(result));
            }
            return config_;
        }

        // =============================================================================
        // Error Handling Functions
        // =============================================================================

        esp_err_t configErrorToEspError(ConfigError error)
        {
            switch (error)
            {
            case ConfigError::INVALID_RANGE:
            case ConfigError::INVALID_GPIO:
            case ConfigError::INVALID_UART_PORT:
            case ConfigError::INVALID_CHANNEL:
            case ConfigError::INVALID_TIMER:
            case ConfigError::INVALID_STRING:
            case ConfigError::INVALID_ANGLE_RANGE:
            case ConfigError::INVALID_FREQUENCY:
                return ESP_ERR_INVALID_ARG;
            case ConfigError::VALIDATION_FAILED:
                return ESP_ERR_INVALID_STATE;
            default:
                return ESP_FAIL;
            }
        }

        const char *getConfigErrorDescription(ConfigError error)
        {
            switch (error)
            {
            case ConfigError::INVALID_RANGE:
                return "Parameter value out of valid range";
            case ConfigError::INVALID_GPIO:
                return "Invalid GPIO pin specification";
            case ConfigError::INVALID_UART_PORT:
                return "Invalid UART port number";
            case ConfigError::INVALID_CHANNEL:
                return "Invalid channel specification";
            case ConfigError::INVALID_TIMER:
                return "Invalid timer specification";
            case ConfigError::INVALID_STRING:
                return "Invalid string parameter";
            case ConfigError::INVALID_ANGLE_RANGE:
                return "Invalid angle range specification";
            case ConfigError::INVALID_FREQUENCY:
                return "Invalid frequency specification";
            case ConfigError::VALIDATION_FAILED:
                return "Configuration validation failed";
            default:
                return "Unknown configuration error";
            }
        }

        // =============================================================================
        // Explicit Template Instantiations
        // =============================================================================

        // We need to explicitly instantiate the template for the types we use
        // This will be expanded as we add more component configuration types

        // Example instantiation - this will be expanded as we create component configs
        template class ConfigValidator<int>; // Placeholder - will be replaced with actual config types

    }
} // namespace digitoys::config
