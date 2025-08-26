#pragma once

#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>
#include <string>
#include <memory>

/**
 * @file component_config.hpp
 * @brief Base configuration interface and validation framework
 *
 * This file defines the unified configuration interface that all components
 * should implement. It provides validation, default value management,
 * and serialization capabilities.
 */

namespace digitoys
{
    namespace config
    {

        /**
         * @brief Base interface for all component configurations
         *
         * All component configuration classes should inherit from this interface
         * to ensure consistent validation and management across the system.
         */
        class IComponentConfig
        {
        public:
            virtual ~IComponentConfig() = default;

            /**
             * @brief Validate the configuration parameters
             * @return ESP_OK if configuration is valid, error code otherwise
             */
            virtual esp_err_t validate() const = 0;

            /**
             * @brief Set default values for all configuration parameters
             *
             * This method should populate all configuration fields with sensible
             * default values, ensuring the component can operate even without
             * explicit configuration.
             */
            virtual void setDefaults() = 0;

            /**
             * @brief Convert configuration to string representation
             * @return String representation for logging and debugging
             */
            virtual std::string toString() const = 0;

            /**
             * @brief Get the configuration schema version
             * @return Version string for configuration compatibility
             */
            virtual const char *getSchemaVersion() const { return "1.0.0"; }

            /**
             * @brief Check if configuration has been modified from defaults
             * @return true if configuration differs from default values
             */
            virtual bool isModified() const { return true; }
        };

        /**
         * @brief Template class for configuration validation utilities
         *
         * Provides common validation functions that can be used by all
         * component configurations to ensure consistent parameter checking.
         */
        template <typename ConfigType>
        class ConfigValidator
        {
        public:
            /**
             * @brief Validate a numeric value is within specified range
             * @param value Value to validate
             * @param min_val Minimum allowed value (inclusive)
             * @param max_val Maximum allowed value (inclusive)
             * @param param_name Parameter name for error reporting
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateRange(float value, float min_val, float max_val, const char *param_name);

            /**
             * @brief Validate a numeric value is within specified range
             * @param value Value to validate
             * @param min_val Minimum allowed value (inclusive)
             * @param max_val Maximum allowed value (inclusive)
             * @param param_name Parameter name for error reporting
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateRange(int value, int min_val, int max_val, const char *param_name);

            /**
             * @brief Validate a GPIO pin number
             * @param pin GPIO pin to validate
             * @param param_name Parameter name for error reporting
             * @param allow_nc Whether GPIO_NUM_NC is allowed
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateGpio(gpio_num_t pin, const char *param_name, bool allow_nc = false);

            /**
             * @brief Validate a UART port number
             * @param port UART port to validate
             * @param param_name Parameter name for error reporting
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateUartPort(uart_port_t port, const char *param_name);

            /**
             * @brief Validate a LEDC channel
             * @param channel LEDC channel to validate
             * @param param_name Parameter name for error reporting
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateLedcChannel(ledc_channel_t channel, const char *param_name);

            /**
             * @brief Validate a LEDC timer
             * @param timer LEDC timer to validate
             * @param param_name Parameter name for error reporting
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateLedcTimer(ledc_timer_t timer, const char *param_name);

            /**
             * @brief Validate a string parameter is not empty
             * @param str String to validate
             * @param param_name Parameter name for error reporting
             * @param max_length Maximum allowed length (0 = no limit)
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateString(const std::string &str, const char *param_name, size_t max_length = 0);

            /**
             * @brief Validate angle range (handles wrap-around)
             * @param min_angle Minimum angle in degrees
             * @param max_angle Maximum angle in degrees
             * @param param_name Parameter name for error reporting
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateAngleRange(float min_angle, float max_angle, const char *param_name);

            /**
             * @brief Validate frequency value
             * @param frequency Frequency in Hz
             * @param min_freq Minimum allowed frequency
             * @param max_freq Maximum allowed frequency
             * @param param_name Parameter name for error reporting
             * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise
             */
            static esp_err_t validateFrequency(uint32_t frequency, uint32_t min_freq, uint32_t max_freq, const char *param_name);
        };

        /**
         * @brief Configuration builder pattern helper
         *
         * Provides a fluent interface for building configurations with validation
         * at each step to catch errors early in the development process.
         */
        template <typename ConfigType>
        class ConfigBuilder
        {
        public:
            ConfigBuilder() { config_.setDefaults(); }

            /**
             * @brief Build and validate the final configuration
             * @return Validated configuration object
             * @throws std::runtime_error if validation fails
             */
            ConfigType build();

            /**
             * @brief Get the current configuration being built
             * @return Reference to configuration under construction
             */
            ConfigType &getConfig() { return config_; }

        protected:
            ConfigType config_;
        };

        // =============================================================================
        // Configuration Error Handling
        // =============================================================================

        /**
         * @brief Configuration-specific error codes
         */
        enum class ConfigError
        {
            INVALID_RANGE = 1,
            INVALID_GPIO,
            INVALID_UART_PORT,
            INVALID_CHANNEL,
            INVALID_TIMER,
            INVALID_STRING,
            INVALID_ANGLE_RANGE,
            INVALID_FREQUENCY,
            VALIDATION_FAILED
        };

        /**
         * @brief Convert configuration error to ESP error code
         * @param error Configuration error code
         * @return Corresponding ESP error code
         */
        esp_err_t configErrorToEspError(ConfigError error);

        /**
         * @brief Get human-readable description of configuration error
         * @param error Configuration error code
         * @return Error description string
         */
        const char *getConfigErrorDescription(ConfigError error);

    }
} // namespace digitoys::config
