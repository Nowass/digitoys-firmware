#include "ComponentConfigFactory.hpp"
#include "ComponentError.hpp"
#include "Logger.hpp"

namespace digitoys::core
{

    const char *ComponentConfigFactory::TAG = "ConfigFactory";

    esp_err_t ComponentConfigFactory::validateConfigCreation(const char *config_name, esp_err_t validation_result)
    {
        // Register with centralized logging system (one-time registration)
        static bool registered = false;
        if (!registered) {
            DIGITOYS_REGISTER_COMPONENT("ConfigFactory", "CONFIG");
            registered = true;
        }

        if (validation_result == ESP_OK)
        {
            DIGITOYS_LOGI("ConfigFactory", "%s created and validated successfully", config_name);
        }
        else
        {
            DIGITOYS_LOGE("ConfigFactory", "%s validation failed: %s", config_name,
                          ComponentError::errorToString(validation_result));
        }
        return validation_result;
    }

} // namespace digitoys::core
