#pragma once

#include <esp_err.h>
#include <string>
#include <exception>
#include <memory>

/**
 * @file component_error.hpp
 * @brief Unified error handling framework for DigiToys components
 *
 * This file provides structured error handling with detailed error
 * information, recovery strategies, and integration with ESP-IDF
 * error system.
 */

namespace digitoys
{
    namespace error
    {

        /**
         * @brief DigiToys-specific error codes
         *
         * These error codes extend the ESP-IDF error system with
         * component-specific error information.
         */
        enum class ErrorCode
        {
            SUCCESS = 0, ///< Operation completed successfully

            // Configuration errors (0x1000-0x1FFF)
            INVALID_CONFIG = 0x1000, ///< Configuration validation failed
            MISSING_CONFIG,          ///< Required configuration missing
            CONFIG_OUT_OF_RANGE,     ///< Configuration parameter out of valid range
            CONFIG_CONFLICT,         ///< Configuration parameters conflict

            // Hardware errors (0x2000-0x2FFF)
            HARDWARE_FAILURE = 0x2000, ///< Hardware initialization/operation failed
            GPIO_ERROR,                ///< GPIO configuration/operation error
            UART_ERROR,                ///< UART communication error
            I2C_ERROR,                 ///< I2C communication error
            SPI_ERROR,                 ///< SPI communication error
            PWM_ERROR,                 ///< PWM configuration/operation error
            ADC_ERROR,                 ///< ADC operation error

            // Communication errors (0x3000-0x3FFF)
            COMMUNICATION_ERROR = 0x3000, ///< General communication failure
            TIMEOUT,                      ///< Operation timed out
            BUFFER_OVERFLOW,              ///< Buffer overflow detected
            BUFFER_UNDERFLOW,             ///< Buffer underflow detected
            CHECKSUM_ERROR,               ///< Data checksum verification failed
            PROTOCOL_ERROR,               ///< Protocol violation detected

            // Resource errors (0x4000-0x4FFF)
            MEMORY_ERROR = 0x4000, ///< Memory allocation/access error
            RESOURCE_EXHAUSTED,    ///< System resources exhausted
            RESOURCE_BUSY,         ///< Resource is busy/locked
            RESOURCE_NOT_FOUND,    ///< Requested resource not found
            PERMISSION_DENIED,     ///< Access permission denied

            // State errors (0x5000-0x5FFF)
            STATE_ERROR = 0x5000, ///< Invalid component state for operation
            NOT_INITIALIZED,      ///< Component not initialized
            ALREADY_INITIALIZED,  ///< Component already initialized
            NOT_STARTED,          ///< Component not started
            ALREADY_STARTED,      ///< Component already started
            SHUTTING_DOWN,        ///< Component is shutting down

            // Data errors (0x6000-0x6FFF)
            DATA_ERROR = 0x6000, ///< Data validation/processing error
            INVALID_DATA,        ///< Invalid data format/content
            DATA_CORRUPTION,     ///< Data corruption detected
            PARSING_ERROR,       ///< Data parsing failed
            CONVERSION_ERROR,    ///< Data type conversion failed

            // Operation errors (0x7000-0x7FFF)
            OPERATION_ERROR = 0x7000, ///< General operation failure
            NOT_SUPPORTED,            ///< Operation not supported
            OPERATION_ABORTED,        ///< Operation was aborted
            RETRY_EXHAUSTED,          ///< Maximum retry attempts exceeded
            DEPENDENCY_ERROR,         ///< Dependency component error

            // Unknown/Other errors (0x8000+)
            UNKNOWN_ERROR = 0x8000 ///< Unknown or unclassified error
        };

        /**
         * @brief Error severity levels
         */
        enum class ErrorSeverity
        {
            INFO = 0, ///< Informational, no action required
            WARNING,  ///< Warning, operation may continue with degraded functionality
            ERROR,    ///< Error, operation failed but system can recover
            CRITICAL, ///< Critical error, system functionality severely impacted
            FATAL     ///< Fatal error, system cannot continue operation
        };

        /**
         * @brief Error recovery strategies
         */
        enum class RecoveryStrategy
        {
            NONE = 0, ///< No recovery possible
            RETRY,    ///< Retry the operation
            RESET,    ///< Reset the component
            RESTART,  ///< Restart the component
            FALLBACK, ///< Use fallback mechanism
            IGNORE,   ///< Ignore the error and continue
            SHUTDOWN  ///< Shutdown the component/system
        };

        /**
         * @brief Detailed error information
         *
         * Provides comprehensive error information including context,
         * recovery options, and debugging details.
         */
        class ComponentError
        {
        public:
            /**
             * @brief Constructor
             * @param code Error code
             * @param component Component name where error occurred
             * @param operation Operation that failed
             * @param message Detailed error message
             * @param severity Error severity level
             * @param recovery Suggested recovery strategy
             */
            ComponentError(ErrorCode code,
                           std::string component,
                           std::string operation,
                           std::string message,
                           ErrorSeverity severity = ErrorSeverity::ERROR,
                           RecoveryStrategy recovery = RecoveryStrategy::NONE);

            /**
             * @brief Constructor with ESP error code
             * @param esp_error ESP-IDF error code
             * @param component Component name where error occurred
             * @param operation Operation that failed
             * @param message Additional error message
             */
            ComponentError(esp_err_t esp_error,
                           std::string component,
                           std::string operation,
                           std::string message = "");

            // =========================================================================
            // Error Information Access
            // =========================================================================

            ErrorCode getCode() const { return code_; }
            const std::string &getComponent() const { return component_; }
            const std::string &getOperation() const { return operation_; }
            const std::string &getMessage() const { return message_; }
            ErrorSeverity getSeverity() const { return severity_; }
            RecoveryStrategy getRecoveryStrategy() const { return recovery_strategy_; }
            esp_err_t getEspError() const { return esp_error_; }
            uint32_t getTimestamp() const { return timestamp_; }

            /**
             * @brief Get formatted error string
             * @return Complete error description
             */
            std::string toString() const;

            /**
             * @brief Get short error description
             * @return Brief error summary
             */
            std::string getShortDescription() const;

            /**
             * @brief Convert to ESP-IDF error code
             * @return Corresponding ESP error code
             */
            esp_err_t toEspError() const;

            // =========================================================================
            // Error Classification
            // =========================================================================

            bool isRecoverable() const;
            bool isRetryable() const;
            bool isCritical() const;
            bool isHardwareRelated() const;
            bool isConfigurationRelated() const;

        private:
            ErrorCode code_;                     ///< DigiToys error code
            std::string component_;              ///< Component name
            std::string operation_;              ///< Failed operation
            std::string message_;                ///< Detailed error message
            ErrorSeverity severity_;             ///< Error severity
            RecoveryStrategy recovery_strategy_; ///< Suggested recovery
            esp_err_t esp_error_;                ///< ESP-IDF error code (if applicable)
            uint32_t timestamp_;                 ///< Error occurrence timestamp
        };

        /**
         * @brief Exception class for DigiToys errors
         *
         * Provides exception-based error handling for C++ code
         * while maintaining compatibility with ESP-IDF C-style errors.
         */
        class ComponentException : public std::exception
        {
        public:
            explicit ComponentException(const ComponentError &error)
                : error_(error) {}

            explicit ComponentException(ErrorCode code,
                                        const std::string &component,
                                        const std::string &operation,
                                        const std::string &message)
                : error_(code, component, operation, message) {}

            const char *what() const noexcept override
            {
                what_str_ = error_.toString();
                return what_str_.c_str();
            }

            const ComponentError &getError() const { return error_; }

        private:
            ComponentError error_;
            mutable std::string what_str_;
        };

        // =============================================================================
        // Error Handling Utilities
        // =============================================================================

        /**
         * @brief Error handling utility functions
         */
        namespace utils
        {

            /**
             * @brief Convert ErrorCode to string
             * @param code Error code to convert
             * @return String representation
             */
            std::string errorCodeToString(ErrorCode code);

            /**
             * @brief Convert ErrorSeverity to string
             * @param severity Severity level to convert
             * @return String representation
             */
            std::string severityToString(ErrorSeverity severity);

            /**
             * @brief Convert RecoveryStrategy to string
             * @param strategy Recovery strategy to convert
             * @return String representation
             */
            std::string recoveryStrategyToString(RecoveryStrategy strategy);

            /**
             * @brief Convert ESP error to DigiToys error code
             * @param esp_error ESP-IDF error code
             * @return Corresponding DigiToys error code
             */
            ErrorCode espErrorToDigiToysError(esp_err_t esp_error);

            /**
             * @brief Convert DigiToys error code to ESP error
             * @param code DigiToys error code
             * @return Corresponding ESP-IDF error code
             */
            esp_err_t digitoysErrorToEspError(ErrorCode code);

            /**
             * @brief Check if ESP error indicates success
             * @param error ESP error code
             * @return true if error indicates success
             */
            inline bool isSuccess(esp_err_t error)
            {
                return error == ESP_OK;
            }

            /**
             * @brief Check if DigiToys error indicates success
             * @param error DigiToys error code
             * @return true if error indicates success
             */
            inline bool isSuccess(ErrorCode error)
            {
                return error == ErrorCode::SUCCESS;
            }

        } // namespace utils

// =============================================================================
// Error Handling Macros
// =============================================================================

/**
 * @brief Macro to check ESP error and create ComponentError
 */
#define DIGITOYS_CHECK_ESP_ERROR(esp_err, component, operation, message)           \
    do                                                                             \
    {                                                                              \
        if ((esp_err) != ESP_OK)                                                   \
        {                                                                          \
            return ComponentError((esp_err), (component), (operation), (message)); \
        }                                                                          \
    } while (0)

/**
 * @brief Macro to check condition and create ComponentError
 */
#define DIGITOYS_CHECK_CONDITION(condition, code, component, operation, message) \
    do                                                                           \
    {                                                                            \
        if (!(condition))                                                        \
        {                                                                        \
            return ComponentError((code), (component), (operation), (message));  \
        }                                                                        \
    } while (0)

/**
 * @brief Macro to propagate ComponentError
 */
#define DIGITOYS_PROPAGATE_ERROR(error_result)           \
    do                                                   \
    {                                                    \
        if (!utils::isSuccess((error_result).getCode())) \
        {                                                \
            return (error_result);                       \
        }                                                \
    } while (0)

    }
} // namespace digitoys::error
