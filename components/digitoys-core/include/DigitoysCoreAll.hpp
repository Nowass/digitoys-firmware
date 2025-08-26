#pragma once

/**
 * @file DigitoysCoreAll.hpp
 * @brief Convenience header that includes all digitoys-core functionality
 *
 * This header provides easy access to all core infrastructure components:
 * - Component lifecycle management (IComponent, ComponentBase)
 * - Unified error handling (ComponentError)
 * - System constants (Constants)
 * - Logging macros (DIGITOYS_LOG*)
 */

#include "IComponent.hpp"
#include "ComponentBase.hpp"
#include "ComponentError.hpp"
#include "Constants.hpp"
#include "ConfigValidator.hpp"
#include "ComponentConfigFactory.hpp"

namespace digitoys::core
{
    // Version information
    constexpr const char *VERSION = "1.0.0";
    constexpr int VERSION_MAJOR = 1;
    constexpr int VERSION_MINOR = 0;
    constexpr int VERSION_PATCH = 0;
}

/**
 * @brief Quick reference for common patterns:
 *
 * 1. Creating a new component:
 *    ```cpp
 *    class MyComponent : public digitoys::core::ComponentBase {
 *    public:
 *        explicit MyComponent() : ComponentBase("MyComponent") {}
 *        esp_err_t initialize() override { ... }
 *        esp_err_t start() override { ... }
 *        esp_err_t stop() override { ... }
 *        esp_err_t shutdown() override { ... }
 *    };
 *    ```
 *
 * 2. Error handling:
 *    ```cpp
 *    DIGITOYS_ERROR_CHECK("MyComponent", some_esp_function());
 *    ```
 *
 * 3. Logging:
 *    ```cpp
 *    DIGITOYS_LOGI("MyComponent", "Operation completed successfully");
 *    DIGITOYS_LOGE("MyComponent", "Failed with error: %d", error_code);
 *    ```
 *
 * 4. Using constants:
 *    ```cpp
 *    using namespace digitoys::constants;
 *    float brake_duty = pwm::BRAKE_DUTY;
 *    gpio_num_t lidar_pin = pins::LIDAR_TX;
 *    ```
 */
