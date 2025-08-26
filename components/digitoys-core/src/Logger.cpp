#include "Logger.hpp"
#include <esp_log.h>
#include <cstdarg>
#include <cstring>

namespace digitoys::core
{

    Logger &Logger::getInstance()
    {
        static Logger instance;
        return instance;
    }

    esp_err_t Logger::registerComponent(const char *component_name,
                                        const char *tag,
                                        esp_log_level_t default_level)
    {
        if (!component_name || !tag)
        {
            ESP_LOGE(TAG, "Invalid component name or tag");
            return ESP_ERR_INVALID_ARG;
        }

        std::string name(component_name);

        // Check if already registered
        if (components_.find(name) != components_.end())
        {
            ESP_LOGD(TAG, "Component '%s' already registered", component_name);
            return ESP_OK;
        }

        // Create component info
        ComponentInfo info;
        info.name = name;
        info.tag = tag;
        info.level = default_level;
        info.enabled = true;
        info.message_count = 0;

        // Register with ESP-IDF logging system
        esp_log_level_set(info.tag.c_str(), default_level);

        // Store in our registry
        components_[name] = std::move(info);

        ESP_LOGI(TAG, "Registered component '%s' with tag '%s' at level %d",
                 component_name, info.tag.c_str(), default_level);

        return ESP_OK;
    }

    esp_err_t Logger::setComponentLogLevel(const char *component_name, esp_log_level_t level)
    {
        if (!component_name)
        {
            return ESP_ERR_INVALID_ARG;
        }

        auto it = components_.find(component_name);
        if (it == components_.end())
        {
            ESP_LOGW(TAG, "Component '%s' not registered", component_name);
            return ESP_ERR_NOT_FOUND;
        }

        // Update our registry
        it->second.level = level;

        // Update ESP-IDF log level
        esp_log_level_set(it->second.tag.c_str(), level);

        ESP_LOGI(TAG, "Set log level for '%s' to %d", component_name, level);
        return ESP_OK;
    }

    const char *Logger::getComponentTag(const char *component_name) const
    {
        if (!component_name)
        {
            return nullptr;
        }

        auto it = components_.find(component_name);
        if (it == components_.end())
        {
            return nullptr;
        }

        return it->second.tag.c_str();
    }

    bool Logger::isLoggingEnabled(const char *component_name, esp_log_level_t level) const
    {
        if (!component_name)
        {
            return false;
        }

        auto it = components_.find(component_name);
        if (it == components_.end())
        {
            return false;
        }

        return it->second.enabled && (level <= it->second.level);
    }

    void Logger::logMessage(const char *component_name,
                            esp_log_level_t level,
                            const char *function_name,
                            const char *format,
                            ...)
    {
        if (!component_name || !function_name || !format)
        {
            return;
        }

        auto it = components_.find(component_name);
        if (it == components_.end())
        {
            ESP_LOGW(TAG, "Logging from unregistered component: %s", component_name);
            return;
        }

        // Update message count
        const_cast<ComponentInfo &>(it->second).message_count++;

        // Format the message with function context
        char formatted_msg[256];
        va_list args;
        va_start(args, format);
        vsnprintf(formatted_msg, sizeof(formatted_msg), format, args);
        va_end(args);

        // Log using ESP-IDF with component context
        const char *tag = it->second.tag.c_str();
        switch (level)
        {
        case ESP_LOG_ERROR:
            ESP_LOGE(tag, "[%s] %s", function_name, formatted_msg);
            break;
        case ESP_LOG_WARN:
            ESP_LOGW(tag, "[%s] %s", function_name, formatted_msg);
            break;
        case ESP_LOG_INFO:
            ESP_LOGI(tag, "[%s] %s", function_name, formatted_msg);
            break;
        case ESP_LOG_DEBUG:
            ESP_LOGD(tag, "[%s] %s", function_name, formatted_msg);
            break;
        case ESP_LOG_VERBOSE:
            ESP_LOGV(tag, "[%s] %s", function_name, formatted_msg);
            break;
        default:
            ESP_LOGI(tag, "[%s] %s", function_name, formatted_msg);
            break;
        }
    }

    const Logger::ComponentInfo *Logger::getComponentInfo(const char *component_name) const
    {
        if (!component_name)
        {
            return nullptr;
        }

        auto it = components_.find(component_name);
        if (it == components_.end())
        {
            return nullptr;
        }

        return &it->second;
    }

    esp_err_t Logger::setComponentEnabled(const char *component_name, bool enabled)
    {
        if (!component_name)
        {
            return ESP_ERR_INVALID_ARG;
        }

        auto it = components_.find(component_name);
        if (it == components_.end())
        {
            ESP_LOGW(TAG, "Component '%s' not registered", component_name);
            return ESP_ERR_NOT_FOUND;
        }

        it->second.enabled = enabled;

        ESP_LOGI(TAG, "Component '%s' logging %s",
                 component_name, enabled ? "enabled" : "disabled");

        return ESP_OK;
    }

    esp_err_t Logger::setDebugMode(bool enabled)
    {
        debug_mode_ = enabled;

        ESP_LOGI(TAG, "Debug mode %s", enabled ? "enabled" : "disabled");

        // If enabling debug mode, set all components to debug level
        if (enabled)
        {
            for (auto &[name, info] : components_)
            {
                esp_log_level_set(info.tag.c_str(), ESP_LOG_DEBUG);
            }
        }
        else
        {
            // Restore original levels
            for (auto &[name, info] : components_)
            {
                esp_log_level_set(info.tag.c_str(), info.level);
            }
        }

        return ESP_OK;
    }

    esp_err_t Logger::enableDebugFor(const std::vector<std::string> &component_names)
    {
        for (const auto &component_name : component_names)
        {
            auto it = components_.find(component_name);
            if (it != components_.end())
            {
                esp_log_level_set(it->second.tag.c_str(), ESP_LOG_DEBUG);
                ESP_LOGI(TAG, "Debug enabled for component '%s'", component_name.c_str());
            }
            else
            {
                ESP_LOGW(TAG, "Component '%s' not found for debug enable", component_name.c_str());
            }
        }
        return ESP_OK;
    }

    esp_err_t Logger::setGlobalLogLevel(esp_log_level_t level)
    {
        global_level_ = level;

        // Apply to all registered components
        for (auto &[name, info] : components_)
        {
            info.level = level;
            esp_log_level_set(info.tag.c_str(), level);
        }

        ESP_LOGI(TAG, "Global log level set to %d", level);
        return ESP_OK;
    }

    bool Logger::isDebugMode() const
    {
        return debug_mode_;
    }

    esp_log_level_t Logger::getGlobalLogLevel() const
    {
        return global_level_;
    }

    const std::unordered_map<std::string, Logger::ComponentInfo> &Logger::getAllComponents() const
    {
        return components_;
    }

} // namespace digitoys::core
