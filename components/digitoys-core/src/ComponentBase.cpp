#include "ComponentBase.hpp"
#include "ComponentError.hpp"
#include "Logger.hpp"

namespace digitoys::core
{

    static const char *TAG = "ComponentBase";

    ComponentBase::ComponentBase(const char *name)
        : component_name_(name)
    {
        // Register with centralized logging system
        DIGITOYS_REGISTER_COMPONENT("ComponentBase", "CORE");
        
        DIGITOYS_LOGD("ComponentBase", "Created component: %s", name);
    }

    bool ComponentBase::isInitialized() const
    {
        return state_ != ComponentState::UNINITIALIZED;
    }

    bool ComponentBase::isRunning() const
    {
        return state_ == ComponentState::RUNNING;
    }

    const char *ComponentBase::getName() const
    {
        return component_name_;
    }

    ComponentState ComponentBase::getState() const
    {
        return state_;
    }

    esp_err_t ComponentBase::setState(ComponentState new_state)
    {
        if (!isValidTransition(state_, new_state))
        {
            DIGITOYS_LOGE("ComponentBase", "Invalid state transition from %s to %s",
                          componentStateToString(state_), componentStateToString(new_state));
            return ESP_ERR_INVALID_STATE;
        }

        ComponentState old_state = state_;
        state_ = new_state;
        logStateChange(old_state, new_state);

        return ESP_OK;
    }

    void ComponentBase::logStateChange(ComponentState from, ComponentState to)
    {
        DIGITOYS_LOGI("ComponentBase", "State change: %s -> %s",
                      componentStateToString(from), componentStateToString(to));
    }

    bool ComponentBase::isValidTransition(ComponentState from, ComponentState to) const
    {
        // Define valid state transitions
        switch (from)
        {
        case ComponentState::UNINITIALIZED:
            return (to == ComponentState::INITIALIZED || to == ComponentState::ERROR);

        case ComponentState::INITIALIZED:
            return (to == ComponentState::RUNNING || to == ComponentState::ERROR);

        case ComponentState::RUNNING:
            return (to == ComponentState::STOPPED || to == ComponentState::ERROR);

        case ComponentState::STOPPED:
            return (to == ComponentState::RUNNING || to == ComponentState::UNINITIALIZED || to == ComponentState::ERROR);

        case ComponentState::ERROR:
            return (to == ComponentState::UNINITIALIZED); // Allow recovery
        }

        return false;
    }

    const char *componentStateToString(ComponentState state)
    {
        switch (state)
        {
        case ComponentState::UNINITIALIZED:
            return "UNINITIALIZED";
        case ComponentState::INITIALIZED:
            return "INITIALIZED";
        case ComponentState::RUNNING:
            return "RUNNING";
        case ComponentState::STOPPED:
            return "STOPPED";
        case ComponentState::ERROR:
            return "ERROR";
        }
        return "UNKNOWN";
    }

} // namespace digitoys::core
