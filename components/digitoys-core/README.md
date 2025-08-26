# DigiToys Core Framework

This component provides the unified foundation for all DigiToys firmware components, establishing consistent patterns for configuration, logging, error handling, and component lifecycle management.

## Features

### 🔧 Unified Configuration Management
- **Base configuration interface** (`IComponentConfig`) for all components
- **Validation framework** with type-safe parameter checking
- **Default value management** ensuring components work out-of-the-box
- **Configuration builders** for fluent, validated configuration construction

### 📝 Structured Logging System
- **Component-specific loggers** with consistent formatting
- **Multiple log levels** (Error, Warning, Info, Debug, Verbose)
- **Structured logging methods** for common operations (initialization, state changes, metrics)
- **ESP-IDF integration** leveraging existing logging infrastructure
- **Diagnostic helpers** for memory and task information

### ⚠️ Comprehensive Error Handling
- **Unified error codes** extending ESP-IDF error system
- **Error severity levels** (Info, Warning, Error, Critical, Fatal)
- **Recovery strategies** for automated error handling
- **Detailed error context** including component, operation, and timestamp
- **Exception support** for C++ code with ESP-IDF compatibility

### 🔄 Component Lifecycle Management
- **Standardized interface** (`IComponent`) for all components
- **State management** with thread-safe state transitions
- **Health monitoring** with status reporting
- **Base implementation** (`ComponentBase`) reducing boilerplate code

### 📊 Centralized Constants
- **All magic numbers** moved to centralized location
- **Categorized constants** (PWM, Timing, Memory, LiDAR, Safety, etc.)
- **Type-safe definitions** preventing accidental misuse
- **Documentation** explaining the purpose of each constant

## Usage

### Basic Component Implementation

```cpp
#include "digitoys_core.hpp"

namespace my_component {

// 1. Define configuration
struct MyComponentConfig : public digitoys::config::IComponentConfig {
    gpio_num_t pin{GPIO_NUM_2};
    uint32_t frequency{1000};
    
    esp_err_t validate() const override {
        auto validator = digitoys::config::ConfigValidator<MyComponentConfig>{};
        ESP_RETURN_ON_ERROR(validator.validateGpio(pin, "pin"), "CONFIG", "Invalid pin");
        ESP_RETURN_ON_ERROR(validator.validateFrequency(frequency, 100, 10000, "frequency"), "CONFIG", "Invalid frequency");
        return ESP_OK;
    }
    
    void setDefaults() override {
        pin = GPIO_NUM_2;
        frequency = 1000;
    }
    
    std::string toString() const override {
        return "MyComponent: pin=" + std::to_string(pin) + ", freq=" + std::to_string(frequency);
    }
};

// 2. Implement component
class MyComponent : public digitoys::core::ComponentBase<MyComponentConfig> {
public:
    explicit MyComponent(const MyComponentConfig& config)
        : ComponentBase(config, "MyComponent", "1.0.0")
        , logger_("MY_COMPONENT") {}
    
    esp_err_t initialize() override {
        setState(digitoys::core::ComponentState::INITIALIZING);
        
        // Validate configuration
        ESP_RETURN_ON_ERROR(validateConfig(), "INIT", "Configuration validation failed");
        
        // Initialize hardware
        // ... hardware setup code ...
        
        setState(digitoys::core::ComponentState::INITIALIZED);
        logger_.logInitialization(true, getConfig().toString());
        return ESP_OK;
    }
    
    esp_err_t start() override {
        if (getState() != digitoys::core::ComponentState::INITIALIZED) {
            return ESP_ERR_INVALID_STATE;
        }
        
        setState(digitoys::core::ComponentState::STARTING);
        
        // Start operation
        // ... startup code ...
        
        setState(digitoys::core::ComponentState::RUNNING);
        logger_.info("Component started successfully");
        return ESP_OK;
    }
    
    esp_err_t stop() override {
        setState(digitoys::core::ComponentState::STOPPING);
        // ... stop code ...
        setState(digitoys::core::ComponentState::STOPPED);
        return ESP_OK;
    }
    
    esp_err_t shutdown() override {
        stop();
        // ... cleanup code ...
        setState(digitoys::core::ComponentState::UNINITIALIZED);
        return ESP_OK;
    }
    
private:
    digitoys::logging::ComponentLogger logger_;
};

} // namespace my_component
```

### Application Integration

```cpp
#include "digitoys_core.hpp"

extern "C" void app_main() {
    // Initialize framework
    ESP_ERROR_CHECK(digitoys::initializeFramework());
    
    // Create and use components
    my_component::MyComponentConfig config;
    config.setDefaults();
    
    my_component::MyComponent component(config);
    ESP_ERROR_CHECK(component.initialize());
    ESP_ERROR_CHECK(component.start());
    
    // Component is now running...
}
```

## Constants Usage

Instead of magic numbers scattered throughout the code:

```cpp
// Old way (inconsistent, hard to maintain)
#define THROTTLE_NEUTRAL 0.0856f
const int STACK_SIZE = 4096;
uint32_t freq = 62;

// New way (centralized, documented, type-safe)
using namespace digitoys::constants;
float neutral = pwm::NEUTRAL_DUTY;
uint32_t stack = memory::DEFAULT_TASK_STACK_SIZE;
uint32_t frequency = pwm::HIGH_FREQUENCY_HZ;
```

## Error Handling

```cpp
// Structured error handling
digitoys::error::ComponentError error(
    digitoys::error::ErrorCode::HARDWARE_FAILURE,
    "MyComponent",
    "GPIO_INIT",
    "Failed to configure GPIO pin",
    digitoys::error::ErrorSeverity::CRITICAL,
    digitoys::error::RecoveryStrategy::RESET
);

logger_.error("Operation failed: %s", error.toString().c_str());

// ESP error integration
esp_err_t esp_result = some_esp_function();
if (esp_result != ESP_OK) {
    digitoys::error::ComponentError esp_error(esp_result, "MyComponent", "ESP_FUNCTION");
    return esp_error.toEspError();
}
```

## Logging

```cpp
digitoys::logging::ComponentLogger logger("MY_COMPONENT");

// Basic logging
logger.info("Component starting...");
logger.warning("Parameter out of optimal range: %.2f", value);
logger.error("Hardware initialization failed");

// Structured logging
logger.logInitialization(true, "All systems nominal");
logger.logStateChange("IDLE", "RUNNING", "User request");
logger.logMetric("Temperature", 25.4f, "°C");
logger.logHardwareOperation("GPIO_CONFIG", true, "Pin 2 configured as output");
```

## Integration with Existing Code

This framework is designed to be gradually adopted:

1. **Phase 1**: Use constants and logging in new code
2. **Phase 2**: Refactor existing components to use the base interfaces
3. **Phase 3**: Full adoption with configuration management

The framework is backward compatible and doesn't require immediate changes to existing code.
