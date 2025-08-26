#pragma once

#include <esp_log.h>
#include <esp_err.h>
#include <unordered_map>
#include <string>
#include <memory>
#include <vector>

namespace digitoys::core
{

    /**
     * @brief Centralized logging system for all DigiToys components
     *
     * Provides unified logging with component registration, consistent formatting,
     * and centralized log level management. Integrates with ESP-IDF logging
     * while adding component lifecycle awareness and structured metadata.
     */
    class Logger
    {
    public:
        /**
         * @brief Component information for logging context
         */
        struct ComponentInfo
        {
            std::string name;       ///< Component name
            std::string tag;        ///< Logging TAG
            esp_log_level_t level;  ///< Component-specific log level
            bool enabled;           ///< Logging enabled flag
            uint32_t message_count; ///< Total messages logged
        };

        /**
         * @brief Get the singleton logger instance
         * @return Reference to the logger instance
         */
        static Logger &getInstance();

        /**
         * @brief Register a component for centralized logging
         * @param component_name Unique component name
         * @param tag Logging TAG (will be prefixed with "DT_")
         * @param default_level Default log level for this component
         * @return ESP_OK if registered successfully
         */
        esp_err_t registerComponent(const char *component_name,
                                    const char *tag,
                                    esp_log_level_t default_level = ESP_LOG_INFO);

        /**
         * @brief Set log level for a specific component
         * @param component_name Component name
         * @param level New log level
         * @return ESP_OK if level set successfully
         */
        esp_err_t setComponentLogLevel(const char *component_name, esp_log_level_t level);

        /**
         * @brief Get component TAG for logging
         * @param component_name Component name
         * @return Component TAG or nullptr if not registered
         */
        const char *getComponentTag(const char *component_name) const;

        /**
         * @brief Check if logging is enabled for component at given level
         * @param component_name Component name
         * @param level Log level to check
         * @return true if logging should proceed
         */
        bool isLoggingEnabled(const char *component_name, esp_log_level_t level) const;

        /**
         * @brief Log a message with component context
         * @param component_name Component name
         * @param level Log level
         * @param function_name Function name (from __func__)
         * @param format Format string
         * @param ... Format arguments
         */
        void logMessage(const char *component_name,
                        esp_log_level_t level,
                        const char *function_name,
                        const char *format,
                        ...) __attribute__((format(printf, 5, 6)));

        /**
         * @brief Get component information for diagnostics
         * @param component_name Component name
         * @return ComponentInfo pointer or nullptr if not found
         */
        const ComponentInfo *getComponentInfo(const char *component_name) const;

        /**
         * @brief Enable/disable logging for a component
         * @param component_name Component name
         * @param enabled Enable flag
         * @return ESP_OK if successful
         */
        esp_err_t setComponentEnabled(const char *component_name, bool enabled);

        /**
         * @brief Set global debug mode for all components
         * @param enabled Enable debug mode flag
         * @return ESP_OK if successful
         */
        esp_err_t setDebugMode(bool enabled);

        /**
         * @brief Enable debug logging for specific components
         * @param component_names Vector of component names to enable debug for
         * @return ESP_OK if successful
         */
        esp_err_t enableDebugFor(const std::vector<std::string> &component_names);

        /**
         * @brief Set global log level for all components
         * @param level Global log level
         * @return ESP_OK if successful
         */
        esp_err_t setGlobalLogLevel(esp_log_level_t level);

        /**
         * @brief Check if debug mode is currently enabled
         * @return true if debug mode is active
         */
        bool isDebugMode() const;

        /**
         * @brief Get current global log level
         * @return Current global log level
         */
        esp_log_level_t getGlobalLogLevel() const;

        /**
         * @brief Get all registered components (for diagnostics)
         * @return Map of component names to info
         */
        const std::unordered_map<std::string, ComponentInfo> &getAllComponents() const;

    private:
        Logger() = default;
        ~Logger() = default;
        Logger(const Logger &) = delete;
        Logger &operator=(const Logger &) = delete;

        std::unordered_map<std::string, ComponentInfo> components_;
        bool debug_mode_ = false;
        esp_log_level_t global_level_ = ESP_LOG_INFO;
        static constexpr const char *TAG = "LOGGER";
    };

} // namespace digitoys::core

/**
 * @brief Centralized logging macros with component registration
 *
 * These macros automatically register components on first use and provide
 * consistent logging format with function context and component metadata.
 */

#define DIGITOYS_LOG_REGISTER(component_name, tag)                                        \
    do                                                                                    \
    {                                                                                     \
        static bool __registered = false;                                                 \
        if (!__registered)                                                                \
        {                                                                                 \
            digitoys::core::Logger::getInstance().registerComponent(component_name, tag); \
            __registered = true;                                                          \
        }                                                                                 \
    } while (0)

#define DIGITOYS_LOG_IMPL(component_name, tag, level, format, ...)                                                    \
    do                                                                                                                \
    {                                                                                                                 \
        DIGITOYS_LOG_REGISTER(component_name, tag);                                                                   \
        if (digitoys::core::Logger::getInstance().isLoggingEnabled(component_name, level))                            \
        {                                                                                                             \
            digitoys::core::Logger::getInstance().logMessage(component_name, level, __func__, format, ##__VA_ARGS__); \
        }                                                                                                             \
    } while (0)

// Component-aware logging macros
#define DIGITOYS_LOGI(component_name, tag, format, ...) \
    DIGITOYS_LOG_IMPL(component_name, tag, ESP_LOG_INFO, format, ##__VA_ARGS__)

#define DIGITOYS_LOGW(component_name, tag, format, ...) \
    DIGITOYS_LOG_IMPL(component_name, tag, ESP_LOG_WARN, format, ##__VA_ARGS__)

#define DIGITOYS_LOGE(component_name, tag, format, ...) \
    DIGITOYS_LOG_IMPL(component_name, tag, ESP_LOG_ERROR, format, ##__VA_ARGS__)

#define DIGITOYS_LOGD(component_name, tag, format, ...) \
    DIGITOYS_LOG_IMPL(component_name, tag, ESP_LOG_DEBUG, format, ##__VA_ARGS__)

#define DIGITOYS_LOGV(component_name, tag, format, ...) \
    DIGITOYS_LOG_IMPL(component_name, tag, ESP_LOG_VERBOSE, format, ##__VA_ARGS__)

// Convenience macros for common components
#define LOG_LIDAR(level, format, ...) DIGITOYS_LOG##level("LiDAR", "LIDAR", format, ##__VA_ARGS__)
#define LOG_PWM(level, format, ...) DIGITOYS_LOG##level("PWM", "PWM", format, ##__VA_ARGS__)
#define LOG_CONTROL(level, format, ...) DIGITOYS_LOG##level("Control", "CONTROL", format, ##__VA_ARGS__)
#define LOG_MONITOR(level, format, ...) DIGITOYS_LOG##level("Monitor", "MONITOR", format, ##__VA_ARGS__)
#define LOG_SYSTEM(level, format, ...) DIGITOYS_LOG##level("System", "SYSTEM", format, ##__VA_ARGS__)
#define LOG_MAIN(level, format, ...) DIGITOYS_LOG##level("Main", "MAIN", format, ##__VA_ARGS__)
