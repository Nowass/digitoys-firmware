#include "ComponentError.hpp"

namespace digitoys::core
{

    esp_err_t ComponentError::logAndReturn(esp_err_t err, const char *component, const char *operation)
    {
        if (err != ESP_OK)
        {
            ESP_LOGE(component, "Operation failed: %s - Error: %s (0x%x)",
                     operation, errorToString(err), err);
        }
        return err;
    }

    esp_err_t ComponentError::validateAndLog(esp_err_t err, const char *component, const char *context)
    {
        if (err != ESP_OK)
        {
            ESP_LOGW(component, "Error in %s: %s (0x%x)",
                     context, errorToString(err), err);
        }
        return err;
    }

    const char *ComponentError::errorToString(esp_err_t err)
    {
        switch (err)
        {
        case ESP_OK:
            return "No error";
        case ESP_FAIL:
            return "Generic error";
        case ESP_ERR_NO_MEM:
            return "Out of memory";
        case ESP_ERR_INVALID_ARG:
            return "Invalid argument";
        case ESP_ERR_INVALID_STATE:
            return "Invalid state";
        case ESP_ERR_INVALID_SIZE:
            return "Invalid size";
        case ESP_ERR_NOT_FOUND:
            return "Resource not found";
        case ESP_ERR_NOT_SUPPORTED:
            return "Operation not supported";
        case ESP_ERR_TIMEOUT:
            return "Operation timeout";
        case ESP_ERR_INVALID_RESPONSE:
            return "Invalid response";
        case ESP_ERR_INVALID_CRC:
            return "Invalid CRC";
        case ESP_ERR_INVALID_VERSION:
            return "Invalid version";
        case ESP_ERR_INVALID_MAC:
            return "Invalid MAC address";
        default:
            return "Unknown error";
        }
    }

} // namespace digitoys::core
