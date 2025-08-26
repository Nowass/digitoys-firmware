#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>
#include <mutex>
#include <memory>
#include "component_config.hpp"

/**
 * @file component_interface.hpp
 * @brief Unified component interface and base classes
 *
 * This file defines the standard interface that all DigiToys firmware
 * components should implement, providing consistent lifecycle management,
 * state tracking, and health monitoring across the system.
 */

namespace digitoys
{
    namespace core
    {

        /**
         * @brief Component lifecycle states
         */
        enum class ComponentState
        {
            UNINITIALIZED = 0, ///< Component created but not initialized
            INITIALIZING,      ///< Component is being initialized
            INITIALIZED,       ///< Component initialized but not running
            STARTING,          ///< Component is starting up
            RUNNING,           ///< Component is running normally
            STOPPING,          ///< Component is shutting down
            STOPPED,           ///< Component stopped but can be restarted
            ERROR,             ///< Component encountered an error
            FATAL_ERROR        ///< Component encountered unrecoverable error
        };

        /**
         * @brief Component health status
         */
        enum class HealthStatus
        {
            HEALTHY = 0, ///< Component operating normally
            WARNING,     ///< Component has minor issues but functional
            DEGRADED,    ///< Component functionality is reduced
            CRITICAL,    ///< Component has serious issues
            FAILED       ///< Component has failed completely
        };

        /**
         * @brief Base interface for all system components
         *
         * All DigiToys firmware components should implement this interface
         * to ensure consistent behavior and management across the system.
         */
        class IComponent
        {
        public:
            virtual ~IComponent() = default;

            // =========================================================================
            // Lifecycle Management
            // =========================================================================

            /**
             * @brief Initialize the component
             *
             * Perform one-time initialization including hardware setup,
             * resource allocation, and configuration validation.
             *
             * @return ESP_OK on success, error code on failure
             */
            virtual esp_err_t initialize() = 0;

            /**
             * @brief Start the component operation
             *
             * Begin normal operation including starting tasks, enabling
             * interrupts, and beginning data processing.
             *
             * @return ESP_OK on success, error code on failure
             */
            virtual esp_err_t start() = 0;

            /**
             * @brief Stop the component operation
             *
             * Gracefully stop operation while preserving the ability
             * to restart. Resources should remain allocated.
             *
             * @return ESP_OK on success, error code on failure
             */
            virtual esp_err_t stop() = 0;

            /**
             * @brief Shutdown the component completely
             *
             * Release all resources and perform final cleanup.
             * Component cannot be restarted after shutdown.
             *
             * @return ESP_OK on success, error code on failure
             */
            virtual esp_err_t shutdown() = 0;

            // =========================================================================
            // State and Health Management
            // =========================================================================

            /**
             * @brief Get current component state
             * @return Current lifecycle state
             */
            virtual ComponentState getState() const = 0;

            /**
             * @brief Get component health status
             * @return Current health status
             */
            virtual HealthStatus getHealthStatus() const = 0;

            /**
             * @brief Check if component is healthy and operational
             * @return true if component is healthy and running
             */
            virtual bool isHealthy() const
            {
                return getHealthStatus() == HealthStatus::HEALTHY &&
                       getState() == ComponentState::RUNNING;
            }

            /**
             * @brief Check if component is ready for operation
             * @return true if component can be started
             */
            virtual bool isReady() const
            {
                ComponentState state = getState();
                return state == ComponentState::INITIALIZED ||
                       state == ComponentState::STOPPED;
            }

            // =========================================================================
            // Information and Diagnostics
            // =========================================================================

            /**
             * @brief Get component name
             * @return Human-readable component name
             */
            virtual std::string getComponentName() const = 0;

            /**
             * @brief Get component version
             * @return Component version string
             */
            virtual std::string getComponentVersion() const = 0;

            /**
             * @brief Get detailed status information
             * @return Multi-line status string for debugging
             */
            virtual std::string getStatusInfo() const = 0;

            /**
             * @brief Get component uptime in milliseconds
             * @return Time since component was started
             */
            virtual uint32_t getUptimeMs() const = 0;

            /**
             * @brief Perform self-test
             * @return ESP_OK if self-test passes
             */
            virtual esp_err_t selfTest() { return ESP_OK; }

            /**
             * @brief Reset component to initial state
             *
             * Attempt to recover from error states by resetting
             * the component without full shutdown/restart.
             *
             * @return ESP_OK if reset successful
             */
            virtual esp_err_t reset() { return ESP_ERR_NOT_SUPPORTED; }
        };

        /**
         * @brief Base implementation for components with configuration
         *
         * Provides common functionality for components that use configuration
         * objects, including state management, logging, and basic lifecycle.
         */
        template <typename ConfigType>
        class ComponentBase : public IComponent
        {
        public:
            /**
             * @brief Constructor with configuration
             * @param config Component configuration
             * @param name Component name
             * @param version Component version
             */
            explicit ComponentBase(const ConfigType &config,
                                   std::string name,
                                   std::string version = "1.0.0")
                : config_(config), component_name_(std::move(name)), component_version_(std::move(version)), state_(ComponentState::UNINITIALIZED), health_status_(HealthStatus::HEALTHY), start_time_ms_(0)
            {
            }

            virtual ~ComponentBase() = default;

            // =========================================================================
            // IComponent Implementation
            // =========================================================================

            ComponentState getState() const override
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return state_;
            }

            HealthStatus getHealthStatus() const override
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return health_status_;
            }

            std::string getComponentName() const override
            {
                return component_name_;
            }

            std::string getComponentVersion() const override
            {
                return component_version_;
            }

            uint32_t getUptimeMs() const override
            {
                if (start_time_ms_ == 0)
                {
                    return 0;
                }
                return (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time_ms_;
            }

            std::string getStatusInfo() const override
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                return formatStatusInfo();
            }

            // =========================================================================
            // Configuration Access
            // =========================================================================

            /**
             * @brief Get the component configuration
             * @return Reference to configuration object
             */
            const ConfigType &getConfig() const
            {
                return config_;
            }

        protected:
            // =========================================================================
            // Protected Helper Methods
            // =========================================================================

            /**
             * @brief Validate the component configuration
             * @return ESP_OK if configuration is valid
             */
            esp_err_t validateConfig() const
            {
                return config_.validate();
            }

            /**
             * @brief Set component state (thread-safe)
             * @param new_state New state to set
             */
            void setState(ComponentState new_state)
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                ComponentState old_state = state_;
                state_ = new_state;

                // Record start time when transitioning to RUNNING
                if (new_state == ComponentState::RUNNING && old_state != ComponentState::RUNNING)
                {
                    start_time_ms_ = xTaskGetTickCount() * portTICK_PERIOD_MS;
                }

                onStateChanged(old_state, new_state);
            }

            /**
             * @brief Set component health status (thread-safe)
             * @param new_status New health status to set
             */
            void setHealthStatus(HealthStatus new_status)
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                HealthStatus old_status = health_status_;
                health_status_ = new_status;
                onHealthStatusChanged(old_status, new_status);
            }

            /**
             * @brief Called when component state changes
             * @param old_state Previous state
             * @param new_state New state
             */
            virtual void onStateChanged(ComponentState old_state, ComponentState new_state)
            {
                // Override in derived classes for custom behavior
            }

            /**
             * @brief Called when health status changes
             * @param old_status Previous health status
             * @param new_status New health status
             */
            virtual void onHealthStatusChanged(HealthStatus old_status, HealthStatus new_status)
            {
                // Override in derived classes for custom behavior
            }

            /**
             * @brief Format status information for debugging
             * @return Formatted status string
             */
            virtual std::string formatStatusInfo() const
            {
                return component_name_ + " v" + component_version_ +
                       " State: " + stateToString(state_) +
                       " Health: " + healthStatusToString(health_status_) +
                       " Uptime: " + std::to_string(getUptimeMs()) + "ms";
            }

            // =========================================================================
            // Utility Methods
            // =========================================================================

            /**
             * @brief Convert state enum to string
             * @param state State to convert
             * @return String representation
             */
            static std::string stateToString(ComponentState state)
            {
                switch (state)
                {
                case ComponentState::UNINITIALIZED:
                    return "UNINITIALIZED";
                case ComponentState::INITIALIZING:
                    return "INITIALIZING";
                case ComponentState::INITIALIZED:
                    return "INITIALIZED";
                case ComponentState::STARTING:
                    return "STARTING";
                case ComponentState::RUNNING:
                    return "RUNNING";
                case ComponentState::STOPPING:
                    return "STOPPING";
                case ComponentState::STOPPED:
                    return "STOPPED";
                case ComponentState::ERROR:
                    return "ERROR";
                case ComponentState::FATAL_ERROR:
                    return "FATAL_ERROR";
                default:
                    return "UNKNOWN";
                }
            }

            /**
             * @brief Convert health status enum to string
             * @param status Health status to convert
             * @return String representation
             */
            static std::string healthStatusToString(HealthStatus status)
            {
                switch (status)
                {
                case HealthStatus::HEALTHY:
                    return "HEALTHY";
                case HealthStatus::WARNING:
                    return "WARNING";
                case HealthStatus::DEGRADED:
                    return "DEGRADED";
                case HealthStatus::CRITICAL:
                    return "CRITICAL";
                case HealthStatus::FAILED:
                    return "FAILED";
                default:
                    return "UNKNOWN";
                }
            }

        protected:
            ConfigType config_;             ///< Component configuration
            std::string component_name_;    ///< Component name
            std::string component_version_; ///< Component version

        private:
            mutable std::mutex state_mutex_; ///< Mutex for state access
            ComponentState state_;           ///< Current component state
            HealthStatus health_status_;     ///< Current health status
            uint32_t start_time_ms_;         ///< Time when component started running
        };

    }
} // namespace digitoys::core
