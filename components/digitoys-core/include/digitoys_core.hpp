#pragma once

/**
 * @file digitoys_core.hpp
 * @brief Main include header for DigiToys core framework
 *
 * This file provides a single include point for all DigiToys core
 * framework components, making it easy to use the unified system.
 */

// Core framework components
#include "digitoys_constants.hpp"
#include "component_config.hpp"
#include "component_interface.hpp"
#include "component_logging.hpp"
#include "component_error.hpp"

/**
 * @brief Main DigiToys namespace
 *
 * All DigiToys firmware components should be implemented within
 * sub-namespaces of this main namespace for organization.
 */
namespace digitoys
{

    /**
     * @brief Framework version information
     */
    namespace framework
    {
        constexpr const char *VERSION = "1.0.0";
        constexpr const char *BUILD_DATE = __DATE__;
        constexpr const char *BUILD_TIME = __TIME__;
    }

    /**
     * @brief Initialize the DigiToys core framework
     *
     * This function should be called early in the application startup
     * to initialize logging and other core services.
     *
     * @return ESP_OK on success, error code on failure
     */
    esp_err_t initializeFramework();

    /**
     * @brief Get framework version string
     * @return Version information string
     */
    std::string getFrameworkVersion();

    /**
     * @brief Get framework build information
     * @return Build date and time string
     */
    std::string getFrameworkBuildInfo();

} // namespace digitoys
