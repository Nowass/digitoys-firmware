#pragma once

#include <esp_err.h>

namespace digitoys::core {

/**
 * @brief Component lifecycle states
 */
enum class ComponentState {
    UNINITIALIZED,  ///< Component not yet initialized
    INITIALIZED,    ///< Component initialized but not running
    RUNNING,        ///< Component is running
    STOPPED,        ///< Component was running but now stopped
    ERROR          ///< Component is in error state
};

/**
 * @brief Base interface for all components in the system
 * 
 * Provides a unified lifecycle management interface for all components.
 * All components should implement this interface to ensure consistent
 * behavior across the system.
 */
class IComponent {
public:
    virtual ~IComponent() = default;

    /**
     * @brief Initialize the component
     * @return ESP_OK on success, error code otherwise
     */
    virtual esp_err_t initialize() = 0;

    /**
     * @brief Start the component operation
     * @return ESP_OK on success, error code otherwise
     */
    virtual esp_err_t start() = 0;

    /**
     * @brief Stop the component operation
     * @return ESP_OK on success, error code otherwise
     */
    virtual esp_err_t stop() = 0;

    /**
     * @brief Shutdown and cleanup the component
     * @return ESP_OK on success, error code otherwise
     */
    virtual esp_err_t shutdown() = 0;

    /**
     * @brief Check if component is initialized
     * @return true if initialized, false otherwise
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief Check if component is running
     * @return true if running, false otherwise
     */
    virtual bool isRunning() const = 0;

    /**
     * @brief Get component name for logging and debugging
     * @return Component name string
     */
    virtual const char* getName() const = 0;

    /**
     * @brief Get current component state
     * @return Current ComponentState
     */
    virtual ComponentState getState() const = 0;
};

} // namespace digitoys::core
