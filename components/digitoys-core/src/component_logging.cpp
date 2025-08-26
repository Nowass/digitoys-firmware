#include "component_logging.hpp"
#include "digitoys_constants.hpp"
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdio>
#include <cstring>

namespace digitoys
{
    namespace logging
    {

        // =============================================================================
        // ComponentLogger Implementation
        // =============================================================================

        ComponentLogger::ComponentLogger(std::string component_name, LogLevel default_level)
            : component_name_(std::move(component_name)), current_level_(default_level)
        {
            // Set the ESP-IDF log level for this component
            esp_log_level_set(component_name_.c_str(), toEspLogLevel(default_level));
        }

        // Basic logging methods
        void ComponentLogger::error(const char *format, ...) const
        {
            if (!isLevelEnabled(LogLevel::ERROR))
                return;

            va_list args;
            va_start(args, format);
            logInternal(LogLevel::ERROR, format, args);
            va_end(args);
        }

        void ComponentLogger::warning(const char *format, ...) const
        {
            if (!isLevelEnabled(LogLevel::WARN))
                return;

            va_list args;
            va_start(args, format);
            logInternal(LogLevel::WARN, format, args);
            va_end(args);
        }

        void ComponentLogger::info(const char *format, ...) const
        {
            if (!isLevelEnabled(LogLevel::INFO))
                return;

            va_list args;
            va_start(args, format);
            logInternal(LogLevel::INFO, format, args);
            va_end(args);
        }

        void ComponentLogger::debug(const char *format, ...) const
        {
            if (!isLevelEnabled(LogLevel::DEBUG))
                return;

            va_list args;
            va_start(args, format);
            logInternal(LogLevel::DEBUG, format, args);
            va_end(args);
        }

        void ComponentLogger::verbose(const char *format, ...) const
        {
            if (!isLevelEnabled(LogLevel::VERBOSE))
                return;

            va_list args;
            va_start(args, format);
            logInternal(LogLevel::VERBOSE, format, args);
            va_end(args);
        }

        // Structured logging methods
        void ComponentLogger::logInitialization(bool success, const std::string &details)
        {
            if (success)
            {
                info("Initialization successful%s%s",
                     details.empty() ? "" : ": ",
                     details.c_str());
            }
            else
            {
                error("Initialization failed%s%s",
                      details.empty() ? "" : ": ",
                      details.c_str());
            }
        }

        void ComponentLogger::logStateChange(const std::string &from_state,
                                             const std::string &to_state,
                                             const std::string &reason)
        {
            info("State change: %s -> %s%s%s",
                 from_state.c_str(),
                 to_state.c_str(),
                 reason.empty() ? "" : " (",
                 reason.empty() ? "" : (reason + ")").c_str());
        }

        void ComponentLogger::logConfiguration(const std::string &config_info)
        {
            info("Configuration: %s", config_info.c_str());
        }

        void ComponentLogger::logMetric(const std::string &metric_name,
                                        float value,
                                        const std::string &unit)
        {
            debug("Metric %s: %.3f%s%s",
                  metric_name.c_str(),
                  value,
                  unit.empty() ? "" : " ",
                  unit.c_str());
        }

        void ComponentLogger::logHardwareOperation(const std::string &operation,
                                                   bool success,
                                                   const std::string &details)
        {
            if (success)
            {
                debug("Hardware operation '%s' successful%s%s",
                      operation.c_str(),
                      details.empty() ? "" : ": ",
                      details.c_str());
            }
            else
            {
                error("Hardware operation '%s' failed%s%s",
                      operation.c_str(),
                      details.empty() ? "" : ": ",
                      details.c_str());
            }
        }

        void ComponentLogger::logError(const std::string &operation,
                                       esp_err_t error_code,
                                       const std::string &details)
        {
            error("Operation '%s' failed with error 0x%x (%s)%s%s",
                  operation.c_str(),
                  error_code,
                  esp_err_to_name(error_code),
                  details.empty() ? "" : ": ",
                  details.c_str());
        }

        // Configuration and control
        void ComponentLogger::setLogLevel(LogLevel level)
        {
            current_level_ = level;
            esp_log_level_set(component_name_.c_str(), toEspLogLevel(level));
        }

        LogLevel ComponentLogger::getLogLevel() const
        {
            return current_level_;
        }

        bool ComponentLogger::isLevelEnabled(LogLevel level) const
        {
            return level <= current_level_;
        }

        // Diagnostic helpers
        void ComponentLogger::logMemoryInfo() const
        {
            if (!isLevelEnabled(LogLevel::DEBUG))
                return;

            multi_heap_info_t heap_info;
            heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);

            debug("Memory - Free: %u bytes, Largest block: %u bytes, Min free: %u bytes",
                  heap_info.total_free_bytes,
                  heap_info.largest_free_block,
                  heap_info.minimum_free_bytes);
        }

        void ComponentLogger::logTaskInfo() const
        {
            if (!isLevelEnabled(LogLevel::DEBUG))
                return;

            TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
            UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(current_task);

            debug("Task info - High water mark: %u bytes", stack_high_water * sizeof(StackType_t));
        }

        void ComponentLogger::logEspError(esp_err_t error_code) const
        {
            if (error_code == ESP_OK)
            {
                debug("ESP operation successful");
            }
            else
            {
                error("ESP error: 0x%x (%s)", error_code, esp_err_to_name(error_code));
            }
        }

        // Private methods
        void ComponentLogger::logInternal(LogLevel level, const char *format, va_list args) const
        {
            esp_log_level_t esp_level = toEspLogLevel(level);

            // Use ESP-IDF logging system directly
            esp_log_writev(esp_level, component_name_.c_str(), format, args);
        }

        esp_log_level_t ComponentLogger::toEspLogLevel(LogLevel level)
        {
            return static_cast<esp_log_level_t>(level);
        }

        std::string ComponentLogger::getTimestamp() const
        {
            int64_t timestamp_us = esp_timer_get_time();
            uint32_t timestamp_ms = timestamp_us / 1000;

            char buffer[32];
            snprintf(buffer, sizeof(buffer), "[%lu.%03lu]",
                     (unsigned long)(timestamp_ms / 1000),
                     (unsigned long)(timestamp_ms % 1000));

            return std::string(buffer);
        }

        // =============================================================================
        // Global Logging Utilities
        // =============================================================================

        namespace utils
        {

            void setGlobalLogLevel(LogLevel level)
            {
                esp_log_level_t esp_level = ComponentLogger::toEspLogLevel(level);
                esp_log_level_set("*", esp_level);
            }

            esp_err_t initializeLogging()
            {
                // Set default log levels for DigiToys components
                esp_log_level_set("DIGITOYS", ESP_LOG_INFO);
                esp_log_level_set("LIDAR", ESP_LOG_INFO);
                esp_log_level_set("PWM", ESP_LOG_INFO);
                esp_log_level_set("CONTROL", ESP_LOG_INFO);
                esp_log_level_set("MONITOR", ESP_LOG_INFO);

                // Set verbose logging for debugging components
                esp_log_level_set("CONFIG", ESP_LOG_DEBUG);
                esp_log_level_set("SYSTEM", ESP_LOG_DEBUG);

                return ESP_OK;
            }

            namespace utils
            {

                std::string formatEspError(esp_err_t error_code)
                {
                    if (error_code == ESP_OK)
                    {
                        return "Success";
                    }

                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "Error 0x%x (%s)",
                             error_code, esp_err_to_name(error_code));

                    return std::string(buffer);
                }

                std::string formatMemoryInfo()
                {
                    multi_heap_info_t heap_info;
                    heap_caps_get_info(&heap_info, MALLOC_CAP_DEFAULT);

                    char buffer[256];
                    snprintf(buffer, sizeof(buffer),
                             "Memory: Free=%u bytes, Largest=%u bytes, MinFree=%u bytes, Allocated=%u bytes",
                             heap_info.total_free_bytes,
                             heap_info.largest_free_block,
                             heap_info.minimum_free_bytes,
                             heap_info.total_allocated_bytes);

                    return std::string(buffer);
                }

                std::string formatTaskInfo()
                {
                    UBaseType_t task_count = uxTaskGetNumberOfTasks();
                    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
                    const char *task_name = pcTaskGetName(current_task);
                    UBaseType_t stack_high_water = uxTaskGetStackHighWaterMark(current_task);

                    char buffer[256];
                    snprintf(buffer, sizeof(buffer),
                             "Tasks: Count=%u, Current='%s', StackFree=%u bytes",
                             task_count,
                             task_name,
                             stack_high_water * sizeof(StackType_t));

                    return std::string(buffer);
                }

            } // namespace utils

        } // namespace utils

    }
} // namespace digitoys::logging
