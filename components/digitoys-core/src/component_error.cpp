#include "component_error.hpp"
#include <esp_timer.h>
#include <sstream>
#include <iomanip>

namespace digitoys
{
    namespace error
    {

        // =============================================================================
        // ComponentError Implementation
        // =============================================================================

        ComponentError::ComponentError(ErrorCode code,
                                       std::string component,
                                       std::string operation,
                                       std::string message,
                                       ErrorSeverity severity,
                                       RecoveryStrategy recovery)
            : code_(code), component_(std::move(component)), operation_(std::move(operation)), message_(std::move(message)), severity_(severity), recovery_strategy_(recovery), esp_error_(utils::digitoysErrorToEspError(code)), timestamp_(esp_timer_get_time() / 1000) // Convert to milliseconds
        {
        }

        ComponentError::ComponentError(esp_err_t esp_error,
                                       std::string component,
                                       std::string operation,
                                       std::string message)
            : code_(utils::espErrorToDigiToysError(esp_error)), component_(std::move(component)), operation_(std::move(operation)), message_(std::move(message)), severity_(ErrorSeverity::ERROR), recovery_strategy_(RecoveryStrategy::RETRY), esp_error_(esp_error), timestamp_(esp_timer_get_time() / 1000)
        {
            // Adjust severity based on ESP error code
            if (esp_error == ESP_ERR_NO_MEM || esp_error == ESP_ERR_INVALID_STATE)
            {
                severity_ = ErrorSeverity::CRITICAL;
            }
            else if (esp_error == ESP_ERR_TIMEOUT || esp_error == ESP_ERR_INVALID_ARG)
            {
                severity_ = ErrorSeverity::WARNING;
            }

            // Set appropriate recovery strategy
            if (esp_error == ESP_ERR_TIMEOUT)
            {
                recovery_strategy_ = RecoveryStrategy::RETRY;
            }
            else if (esp_error == ESP_ERR_INVALID_STATE)
            {
                recovery_strategy_ = RecoveryStrategy::RESET;
            }
            else if (esp_error == ESP_ERR_NO_MEM)
            {
                recovery_strategy_ = RecoveryStrategy::RESTART;
            }
        }

        std::string ComponentError::toString() const
        {
            std::ostringstream oss;

            oss << "[" << component_ << "] ";
            oss << utils::severityToString(severity_) << ": ";
            oss << operation_ << " failed - ";
            oss << utils::errorCodeToString(code_);

            if (!message_.empty())
            {
                oss << " (" << message_ << ")";
            }

            if (esp_error_ != ESP_OK)
            {
                oss << " [ESP: 0x" << std::hex << esp_error_ << " (" << esp_err_to_name(esp_error_) << ")]";
            }

            oss << " | Recovery: " << utils::recoveryStrategyToString(recovery_strategy_);
            oss << " | Time: " << timestamp_ << "ms";

            return oss.str();
        }

        std::string ComponentError::getShortDescription() const
        {
            std::ostringstream oss;
            oss << component_ << ": " << operation_ << " - " << utils::errorCodeToString(code_);
            return oss.str();
        }

        esp_err_t ComponentError::toEspError() const
        {
            return esp_error_;
        }

        bool ComponentError::isRecoverable() const
        {
            return recovery_strategy_ != RecoveryStrategy::NONE &&
                   severity_ < ErrorSeverity::FATAL;
        }

        bool ComponentError::isRetryable() const
        {
            return recovery_strategy_ == RecoveryStrategy::RETRY;
        }

        bool ComponentError::isCritical() const
        {
            return severity_ >= ErrorSeverity::CRITICAL;
        }

        bool ComponentError::isHardwareRelated() const
        {
            return static_cast<int>(code_) >= 0x2000 && static_cast<int>(code_) < 0x3000;
        }

        bool ComponentError::isConfigurationRelated() const
        {
            return static_cast<int>(code_) >= 0x1000 && static_cast<int>(code_) < 0x2000;
        }

        // =============================================================================
        // Utility Functions Implementation
        // =============================================================================

        namespace utils
        {

            std::string errorCodeToString(ErrorCode code)
            {
                switch (code)
                {
                case ErrorCode::SUCCESS:
                    return "Success";

                // Configuration errors
                case ErrorCode::INVALID_CONFIG:
                    return "Invalid Configuration";
                case ErrorCode::MISSING_CONFIG:
                    return "Missing Configuration";
                case ErrorCode::CONFIG_OUT_OF_RANGE:
                    return "Configuration Out of Range";
                case ErrorCode::CONFIG_CONFLICT:
                    return "Configuration Conflict";

                // Hardware errors
                case ErrorCode::HARDWARE_FAILURE:
                    return "Hardware Failure";
                case ErrorCode::GPIO_ERROR:
                    return "GPIO Error";
                case ErrorCode::UART_ERROR:
                    return "UART Error";
                case ErrorCode::I2C_ERROR:
                    return "I2C Error";
                case ErrorCode::SPI_ERROR:
                    return "SPI Error";
                case ErrorCode::PWM_ERROR:
                    return "PWM Error";
                case ErrorCode::ADC_ERROR:
                    return "ADC Error";

                // Communication errors
                case ErrorCode::COMMUNICATION_ERROR:
                    return "Communication Error";
                case ErrorCode::TIMEOUT:
                    return "Timeout";
                case ErrorCode::BUFFER_OVERFLOW:
                    return "Buffer Overflow";
                case ErrorCode::BUFFER_UNDERFLOW:
                    return "Buffer Underflow";
                case ErrorCode::CHECKSUM_ERROR:
                    return "Checksum Error";
                case ErrorCode::PROTOCOL_ERROR:
                    return "Protocol Error";

                // Resource errors
                case ErrorCode::MEMORY_ERROR:
                    return "Memory Error";
                case ErrorCode::RESOURCE_EXHAUSTED:
                    return "Resource Exhausted";
                case ErrorCode::RESOURCE_BUSY:
                    return "Resource Busy";
                case ErrorCode::RESOURCE_NOT_FOUND:
                    return "Resource Not Found";
                case ErrorCode::PERMISSION_DENIED:
                    return "Permission Denied";

                // State errors
                case ErrorCode::STATE_ERROR:
                    return "State Error";
                case ErrorCode::NOT_INITIALIZED:
                    return "Not Initialized";
                case ErrorCode::ALREADY_INITIALIZED:
                    return "Already Initialized";
                case ErrorCode::NOT_STARTED:
                    return "Not Started";
                case ErrorCode::ALREADY_STARTED:
                    return "Already Started";
                case ErrorCode::SHUTTING_DOWN:
                    return "Shutting Down";

                // Data errors
                case ErrorCode::DATA_ERROR:
                    return "Data Error";
                case ErrorCode::INVALID_DATA:
                    return "Invalid Data";
                case ErrorCode::DATA_CORRUPTION:
                    return "Data Corruption";
                case ErrorCode::PARSING_ERROR:
                    return "Parsing Error";
                case ErrorCode::CONVERSION_ERROR:
                    return "Conversion Error";

                // Operation errors
                case ErrorCode::OPERATION_ERROR:
                    return "Operation Error";
                case ErrorCode::NOT_SUPPORTED:
                    return "Not Supported";
                case ErrorCode::OPERATION_ABORTED:
                    return "Operation Aborted";
                case ErrorCode::RETRY_EXHAUSTED:
                    return "Retry Exhausted";
                case ErrorCode::DEPENDENCY_ERROR:
                    return "Dependency Error";

                case ErrorCode::UNKNOWN_ERROR:
                default:
                    return "Unknown Error";
                }
            }

            std::string severityToString(ErrorSeverity severity)
            {
                switch (severity)
                {
                case ErrorSeverity::INFO:
                    return "INFO";
                case ErrorSeverity::WARNING:
                    return "WARNING";
                case ErrorSeverity::ERROR:
                    return "ERROR";
                case ErrorSeverity::CRITICAL:
                    return "CRITICAL";
                case ErrorSeverity::FATAL:
                    return "FATAL";
                default:
                    return "UNKNOWN";
                }
            }

            std::string recoveryStrategyToString(RecoveryStrategy strategy)
            {
                switch (strategy)
                {
                case RecoveryStrategy::NONE:
                    return "None";
                case RecoveryStrategy::RETRY:
                    return "Retry";
                case RecoveryStrategy::RESET:
                    return "Reset";
                case RecoveryStrategy::RESTART:
                    return "Restart";
                case RecoveryStrategy::FALLBACK:
                    return "Fallback";
                case RecoveryStrategy::IGNORE:
                    return "Ignore";
                case RecoveryStrategy::SHUTDOWN:
                    return "Shutdown";
                default:
                    return "Unknown";
                }
            }

            ErrorCode espErrorToDigiToysError(esp_err_t esp_error)
            {
                switch (esp_error)
                {
                case ESP_OK:
                    return ErrorCode::SUCCESS;
                case ESP_ERR_NO_MEM:
                    return ErrorCode::MEMORY_ERROR;
                case ESP_ERR_INVALID_ARG:
                    return ErrorCode::INVALID_CONFIG;
                case ESP_ERR_INVALID_STATE:
                    return ErrorCode::STATE_ERROR;
                case ESP_ERR_INVALID_SIZE:
                    return ErrorCode::DATA_ERROR;
                case ESP_ERR_NOT_FOUND:
                    return ErrorCode::RESOURCE_NOT_FOUND;
                case ESP_ERR_NOT_SUPPORTED:
                    return ErrorCode::NOT_SUPPORTED;
                case ESP_ERR_TIMEOUT:
                    return ErrorCode::TIMEOUT;
                case ESP_ERR_INVALID_RESPONSE:
                    return ErrorCode::PROTOCOL_ERROR;
                case ESP_ERR_INVALID_CRC:
                    return ErrorCode::CHECKSUM_ERROR;
                case ESP_ERR_INVALID_VERSION:
                    return ErrorCode::DATA_ERROR;
                case ESP_ERR_INVALID_MAC:
                    return ErrorCode::HARDWARE_FAILURE;
                default:
                    return ErrorCode::UNKNOWN_ERROR;
                }
            }

            esp_err_t digitoysErrorToEspError(ErrorCode code)
            {
                switch (code)
                {
                case ErrorCode::SUCCESS:
                    return ESP_OK;
                case ErrorCode::INVALID_CONFIG:
                case ErrorCode::CONFIG_OUT_OF_RANGE:
                case ErrorCode::CONFIG_CONFLICT:
                    return ESP_ERR_INVALID_ARG;
                case ErrorCode::MISSING_CONFIG:
                    return ESP_ERR_NOT_FOUND;
                case ErrorCode::HARDWARE_FAILURE:
                case ErrorCode::GPIO_ERROR:
                case ErrorCode::UART_ERROR:
                case ErrorCode::I2C_ERROR:
                case ErrorCode::SPI_ERROR:
                case ErrorCode::PWM_ERROR:
                case ErrorCode::ADC_ERROR:
                    return ESP_ERR_INVALID_STATE;
                case ErrorCode::TIMEOUT:
                    return ESP_ERR_TIMEOUT;
                case ErrorCode::MEMORY_ERROR:
                    return ESP_ERR_NO_MEM;
                case ErrorCode::BUFFER_OVERFLOW:
                case ErrorCode::BUFFER_UNDERFLOW:
                    return ESP_ERR_INVALID_SIZE;
                case ErrorCode::CHECKSUM_ERROR:
                    return ESP_ERR_INVALID_CRC;
                case ErrorCode::PROTOCOL_ERROR:
                    return ESP_ERR_INVALID_RESPONSE;
                case ErrorCode::RESOURCE_NOT_FOUND:
                    return ESP_ERR_NOT_FOUND;
                case ErrorCode::NOT_SUPPORTED:
                    return ESP_ERR_NOT_SUPPORTED;
                case ErrorCode::STATE_ERROR:
                case ErrorCode::NOT_INITIALIZED:
                case ErrorCode::ALREADY_INITIALIZED:
                case ErrorCode::NOT_STARTED:
                case ErrorCode::ALREADY_STARTED:
                case ErrorCode::SHUTTING_DOWN:
                    return ESP_ERR_INVALID_STATE;
                case ErrorCode::DATA_ERROR:
                case ErrorCode::INVALID_DATA:
                case ErrorCode::DATA_CORRUPTION:
                case ErrorCode::PARSING_ERROR:
                case ErrorCode::CONVERSION_ERROR:
                    return ESP_ERR_INVALID_ARG;
                default:
                    return ESP_FAIL;
                }
            }

        } // namespace utils

    }
} // namespace digitoys::error
