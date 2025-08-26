#pragma once

#include <esp_err.h>
#include <esp_log.h>

namespace digitoys::core
{

    /**
     * @brief Unified error handling and logging utilities
     *
     * Provides consistent error handling patterns across all components
     * with proper logging and error context preservation.
     */
    class ComponentError
    {
    public:
        /**
         * @brief Log error and return the error code
         * @param err Error code to log and return
         * @param component Component name for logging context
         * @param operation Operation that failed
         * @return The original error code
         */
        static esp_err_t logAndReturn(esp_err_t err, const char *component, const char *operation);

        /**
         * @brief Validate error code and log if not ESP_OK
         * @param err Error code to validate
         * @param component Component name for logging context
         * @param context Context description for the error
         * @return The original error code
         */
        static esp_err_t validateAndLog(esp_err_t err, const char *component, const char *context);

        /**
         * @brief Convert ESP error code to human-readable string
         * @param err Error code
         * @return Error description string
         */
        static const char *errorToString(esp_err_t err);
    };

} // namespace digitoys::core

/**
 * @brief Unified logging macros with component context
 *
 * These macros provide consistent logging format across all components
 * with function name and component context.
 */
#define DIGITOYS_LOGI(component, format, ...) \
    ESP_LOGI(component, "[%s] " format, __func__, ##__VA_ARGS__)

#define DIGITOYS_LOGW(component, format, ...) \
    ESP_LOGW(component, "[%s] " format, __func__, ##__VA_ARGS__)

#define DIGITOYS_LOGE(component, format, ...) \
    ESP_LOGE(component, "[%s] " format, __func__, ##__VA_ARGS__)

#define DIGITOYS_LOGD(component, format, ...) \
    ESP_LOGD(component, "[%s] " format, __func__, ##__VA_ARGS__)

/**
 * @brief Component-aware error checking macro
 *
 * Checks error code and logs with component context if not ESP_OK.
 * Returns the error code if not ESP_OK.
 */
#define DIGITOYS_ERROR_CHECK(component, operation)                                             \
    do                                                                                         \
    {                                                                                          \
        esp_err_t __err = (operation);                                                         \
        if (__err != ESP_OK)                                                                   \
        {                                                                                      \
            return digitoys::core::ComponentError::logAndReturn(__err, component, #operation); \
        }                                                                                      \
    } while (0)

/**
 * @brief Component-aware error checking without return
 *
 * Similar to DIGITOYS_ERROR_CHECK but doesn't return, just logs.
 * Useful for cleanup operations or when you want to continue despite errors.
 */
#define DIGITOYS_ERROR_LOG(component, operation)                                          \
    do                                                                                    \
    {                                                                                     \
        esp_err_t __err = (operation);                                                    \
        if (__err != ESP_OK)                                                              \
        {                                                                                 \
            digitoys::core::ComponentError::validateAndLog(__err, component, #operation); \
        }                                                                                 \
    } while (0)
