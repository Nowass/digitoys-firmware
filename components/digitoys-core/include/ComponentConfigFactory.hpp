#pragma once

#include "Constants.hpp"
#include "ConfigValidator.hpp"
#include <esp_err.h>

namespace digitoys::core
{

    /**
     * @brief Base configuration factory utilities
     *
     * Provides common utilities for configuration validation and creation.
     * Specific component factories should be implemented in their respective components.
     */
    class ComponentConfigFactory
    {
    public:
        /**
         * @brief Validate and log configuration creation
         * @param config_name Name of the configuration for logging
         * @param validation_result Result of configuration validation
         * @return ESP_OK if validation passed, error code otherwise
         */
        static esp_err_t validateConfigCreation(const char *config_name, esp_err_t validation_result);

    private:
        static const char *TAG;
    };

} // namespace digitoys::core
