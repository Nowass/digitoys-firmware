#pragma once

#include "IComponent.hpp"
#include <esp_log.h>

namespace digitoys::core
{

    /**
     * @brief Base implementation of IComponent interface
     *
     * Provides common functionality for component lifecycle management,
     * state tracking, and logging. Components should inherit from this
     * class and implement the pure virtual methods.
     */
    class ComponentBase : public IComponent
    {
    public:
        /**
         * @brief Constructor
         * @param name Component name for logging and identification
         */
        explicit ComponentBase(const char *name);

        virtual ~ComponentBase() = default;

        // IComponent interface
        bool isInitialized() const override;
        bool isRunning() const override;
        const char *getName() const override;
        ComponentState getState() const override;

    protected:
        /**
         * @brief Set component state with logging
         * @param new_state New state to set
         * @return ESP_OK on success, error code on invalid transition
         */
        esp_err_t setState(ComponentState new_state);

        /**
         * @brief Log state change
         * @param from Previous state
         * @param to New state
         */
        void logStateChange(ComponentState from, ComponentState to);

        /**
         * @brief Check if state transition is valid
         * @param from Current state
         * @param to Desired state
         * @return true if transition is valid, false otherwise
         */
        bool isValidTransition(ComponentState from, ComponentState to) const;

    protected:
        ComponentState state_ = ComponentState::UNINITIALIZED;
        const char *component_name_;
    };

    /**
     * @brief Convert ComponentState to string for logging
     * @param state ComponentState to convert
     * @return String representation of the state
     */
    const char *componentStateToString(ComponentState state);

} // namespace digitoys::core
