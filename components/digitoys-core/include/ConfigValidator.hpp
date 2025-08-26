#pragma once

#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>

namespace digitoys::core
{

    /**
     * @brief Configuration validation utilities
     *
     * Provides static methods to validate common configuration parameters
     * used across all components. Ensures consistent validation logic
     * and helpful error messages.
     */
    class ConfigValidator
    {
    public:
        /**
         * @brief Validate GPIO pin number
         * @param pin GPIO pin to validate
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateGpio(gpio_num_t pin, const char *context);

        /**
         * @brief Validate GPIO pin for output use
         * @param pin GPIO pin to validate
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateGpioOutput(gpio_num_t pin, const char *context);

        /**
         * @brief Validate numeric value within range
         * @param value Value to validate
         * @param min_val Minimum allowed value (inclusive)
         * @param max_val Maximum allowed value (inclusive)
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateRange(float value, float min_val, float max_val, const char *context);

        /**
         * @brief Validate integer value within range
         * @param value Value to validate
         * @param min_val Minimum allowed value (inclusive)
         * @param max_val Maximum allowed value (inclusive)
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateRange(int value, int min_val, int max_val, const char *context);

        /**
         * @brief Validate UART port number
         * @param port UART port to validate
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateUartPort(uart_port_t port, const char *context);

        /**
         * @brief Validate LEDC channel
         * @param channel LEDC channel to validate
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateLedcChannel(ledc_channel_t channel, const char *context);

        /**
         * @brief Validate LEDC timer
         * @param timer LEDC timer to validate
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateLedcTimer(ledc_timer_t timer, const char *context);

        /**
         * @brief Validate frequency value
         * @param freq_hz Frequency in Hz to validate
         * @param min_freq Minimum allowed frequency
         * @param max_freq Maximum allowed frequency
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateFrequency(uint32_t freq_hz, uint32_t min_freq, uint32_t max_freq, const char *context);

        /**
         * @brief Validate duty cycle percentage
         * @param duty_pct Duty cycle percentage (0-100)
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateDutyPercent(int duty_pct, const char *context);

        /**
         * @brief Validate PWM duty cycle (normalized 0.0-1.0)
         * @param duty Normalized duty cycle
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateNormalizedDuty(float duty, const char *context);

        /**
         * @brief Validate buffer size
         * @param size Buffer size to validate
         * @param min_size Minimum required size
         * @param max_size Maximum allowed size
         * @param context Context description for error messages
         * @return ESP_OK if valid, error code otherwise
         */
        static esp_err_t validateBufferSize(size_t size, size_t min_size, size_t max_size, const char *context);

    private:
        static const char *TAG;
    };

    /**
     * @brief Base interface for configurable components
     *
     * Components that accept configuration should implement this interface
     * to provide consistent validation and default value handling.
     */
    template <typename ConfigType>
    class IConfigurable
    {
    public:
        virtual ~IConfigurable() = default;

        /**
         * @brief Validate configuration parameters
         * @param config Configuration to validate
         * @return ESP_OK if valid, error code otherwise
         */
        virtual esp_err_t validateConfig(const ConfigType &config) const = 0;

        /**
         * @brief Get default configuration
         * @return Default configuration with safe values
         */
        virtual ConfigType getDefaultConfig() const = 0;
    };

} // namespace digitoys::core
