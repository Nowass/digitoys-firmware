#pragma once

#include <esp_log.h>
#include <esp_err.h>
#include <string>
#include <cstdarg>
#include <memory>

/**
 * @file component_logging.hpp
 * @brief Unified logging framework for DigiToys components
 *
 * This file provides a consistent logging interface across all components,
 * with standardized formatting, log levels, and diagnostic capabilities.
 */

namespace digitoys
{
    namespace logging
    {

        /**
         * @brief Log levels (extending ESP-IDF log levels)
         */
        enum class LogLevel
        {
            NONE = ESP_LOG_NONE,      ///< No output
            ERROR = ESP_LOG_ERROR,    ///< Critical errors, system may be unstable
            WARN = ESP_LOG_WARN,      ///< Error conditions from which recovery is possible
            INFO = ESP_LOG_INFO,      ///< Information that highlights system behavior
            DEBUG = ESP_LOG_DEBUG,    ///< Information that is diagnostically helpful
            VERBOSE = ESP_LOG_VERBOSE ///< Information that is only needed when diagnosing problems
        };

        /**
         * @brief Component-specific logger class
         *
         * Provides structured logging with component identification, consistent
         * formatting, and integration with ESP-IDF logging system.
         */
        class ComponentLogger
        {
        public:
            /**
             * @brief Constructor
             * @param component_name Name of the component (used as log tag)
             * @param default_level Default log level for this component
             */
            explicit ComponentLogger(std::string component_name,
                                     LogLevel default_level = LogLevel::INFO);

            /**
             * @brief Destructor
             */
            ~ComponentLogger() = default;

            // =========================================================================
            // Basic Logging Methods
            // =========================================================================

            /**
             * @brief Log an error message
             * @param format Printf-style format string
             * @param ... Format arguments
             */
            void error(const char *format, ...) const;

            /**
             * @brief Log a warning message
             * @param format Printf-style format string
             * @param ... Format arguments
             */
            void warning(const char *format, ...) const;

            /**
             * @brief Log an info message
             * @param format Printf-style format string
             * @param ... Format arguments
             */
            void info(const char *format, ...) const;

            /**
             * @brief Log a debug message
             * @param format Printf-style format string
             * @param ... Format arguments
             */
            void debug(const char *format, ...) const;

            /**
             * @brief Log a verbose message
             * @param format Printf-style format string
             * @param ... Format arguments
             */
            void verbose(const char *format, ...) const;

            // =========================================================================
            // Structured Logging Methods
            // =========================================================================

            /**
             * @brief Log component initialization
             * @param success Whether initialization was successful
             * @param details Additional details about initialization
             */
            void logInitialization(bool success, const std::string &details = "");

            /**
             * @brief Log component state change
             * @param from_state Previous state name
             * @param to_state New state name
             * @param reason Reason for state change
             */
            void logStateChange(const std::string &from_state,
                                const std::string &to_state,
                                const std::string &reason = "");

            /**
             * @brief Log configuration information
             * @param config_info Configuration details
             */
            void logConfiguration(const std::string &config_info);

            /**
             * @brief Log performance metrics
             * @param metric_name Name of the metric
             * @param value Metric value
             * @param unit Unit of measurement
             */
            void logMetric(const std::string &metric_name,
                           float value,
                           const std::string &unit = "");

            /**
             * @brief Log hardware operation
             * @param operation Operation name (e.g., "UART_INIT", "GPIO_SET")
             * @param success Whether operation was successful
             * @param details Additional operation details
             */
            void logHardwareOperation(const std::string &operation,
                                      bool success,
                                      const std::string &details = "");

            /**
             * @brief Log error with error code
             * @param operation Operation that failed
             * @param error_code ESP error code
             * @param details Additional error details
             */
            void logError(const std::string &operation,
                          esp_err_t error_code,
                          const std::string &details = "");

            // =========================================================================
            // Configuration and Control
            // =========================================================================

            /**
             * @brief Set log level for this component
             * @param level New log level
             */
            void setLogLevel(LogLevel level);

            /**
             * @brief Get current log level
             * @return Current log level
             */
            LogLevel getLogLevel() const;

            /**
             * @brief Check if a log level is enabled
             * @param level Log level to check
             * @return true if messages at this level will be output
             */
            bool isLevelEnabled(LogLevel level) const;

            /**
             * @brief Get component name used for logging
             * @return Component name (log tag)
             */
            const std::string &getComponentName() const { return component_name_; }

            /**
             * @brief Convert LogLevel to esp_log_level_t
             * @param level DigiToys log level
             * @return ESP-IDF log level
             */
            static esp_log_level_t toEspLogLevel(LogLevel level);

            // =========================================================================
            // Diagnostic Helpers
            // =========================================================================

            /**
             * @brief Log system memory information
             */
            void logMemoryInfo() const;

            /**
             * @brief Log task information
             */
            void logTaskInfo() const;

            /**
             * @brief Log ESP error description
             * @param error_code ESP error code to describe
             */
            void logEspError(esp_err_t error_code) const;

        private:
            std::string component_name_; ///< Component name used as log tag
            LogLevel current_level_;     ///< Current log level for this component

            /**
             * @brief Internal logging method
             * @param level Log level
             * @param format Format string
             * @param args Variable arguments
             */
            void logInternal(LogLevel level, const char *format, va_list args) const;

            /**
             * @brief Format timestamp for logging
             * @return Formatted timestamp string
             */
            std::string getTimestamp() const;
        };

        // =============================================================================
        // Logging Macros for Convenience
        // =============================================================================

        /**
         * @brief Convenience macros for component logging
         *
         * These macros provide a shorthand for common logging operations
         * and ensure consistent formatting across components.
         */

#define COMPONENT_LOG_ERROR(logger, ...) (logger).error(__VA_ARGS__)
#define COMPONENT_LOG_WARN(logger, ...) (logger).warning(__VA_ARGS__)
#define COMPONENT_LOG_INFO(logger, ...) (logger).info(__VA_ARGS__)
#define COMPONENT_LOG_DEBUG(logger, ...) (logger).debug(__VA_ARGS__)
#define COMPONENT_LOG_VERBOSE(logger, ...) (logger).verbose(__VA_ARGS__)

// Structured logging macros
#define COMPONENT_LOG_INIT_SUCCESS(logger, details) (logger).logInitialization(true, details)
#define COMPONENT_LOG_INIT_FAILURE(logger, details) (logger).logInitialization(false, details)
#define COMPONENT_LOG_STATE_CHANGE(logger, from, to, reason) (logger).logStateChange(from, to, reason)
#define COMPONENT_LOG_CONFIG(logger, config) (logger).logConfiguration(config)
#define COMPONENT_LOG_METRIC(logger, name, value, unit) (logger).logMetric(name, value, unit)
#define COMPONENT_LOG_HW_OP(logger, op, success, details) (logger).logHardwareOperation(op, success, details)
#define COMPONENT_LOG_ESP_ERROR(logger, op, err, details) (logger).logError(op, err, details)

        // =============================================================================
        // Global Logging Utilities
        // =============================================================================

        /**
         * @brief Global logging utility functions
         */
        namespace utils
        {

            /**
             * @brief Set global log level for all DigiToys components
             * @param level Log level to set
             */
            void setGlobalLogLevel(LogLevel level);

            /**
             * @brief Initialize logging subsystem
             * @return ESP_OK on success
             */
            esp_err_t initializeLogging();

            /**
             * @brief Format ESP error code as string
             * @param error_code ESP error code
             * @return Human-readable error description
             */
            std::string formatEspError(esp_err_t error_code);

            /**
             * @brief Format memory usage information
             * @return Formatted memory usage string
             */
            std::string formatMemoryInfo();

            /**
             * @brief Format task information
             * @return Formatted task status string
             */
            std::string formatTaskInfo();

        } // namespace utils

    }
} // namespace digitoys::logging
