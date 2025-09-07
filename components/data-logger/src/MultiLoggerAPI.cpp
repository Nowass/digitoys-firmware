#include "MultiLoggerAPI.hpp"
#include "DataLogger.hpp"
#include "PhysicsAnalyzer.hpp"
#include <esp_log.h>
#include <esp_http_server.h>
#include <esp_timer.h>
#include <sstream>
#include <iomanip>

namespace digitoys::datalogger
{
    const char* MultiLoggerAPI::TAG = "MultiLoggerAPI";

    // HTTP endpoint definitions
    static const httpd_uri_t loggers_list_uri = {
        .uri = "/api/loggers/list",
        .method = HTTP_GET,
        .handler = MultiLoggerAPI::handleLoggersList,
        .user_ctx = nullptr
    };

    static const httpd_uri_t loggers_control_uri = {
        .uri = "/api/loggers/control",
        .method = HTTP_POST,
        .handler = MultiLoggerAPI::handleLoggersControl,
        .user_ctx = nullptr
    };

    static const httpd_uri_t data_export_uri = {
        .uri = "/api/loggers/export",
        .method = HTTP_GET,
        .handler = MultiLoggerAPI::handleDataExport,
        .user_ctx = nullptr
    };

    static const httpd_uri_t logger_status_uri = {
        .uri = "/api/loggers/status",
        .method = HTTP_GET,
        .handler = MultiLoggerAPI::handleLoggerStatus,
        .user_ctx = nullptr
    };

    MultiLoggerAPI::MultiLoggerAPI(DataLogger* data_logger, PhysicsAnalyzer* physics_analyzer)
        : data_logger_(data_logger), physics_analyzer_(physics_analyzer)
    {
        if (!data_logger_)
        {
            ESP_LOGE(TAG, "DataLogger instance is required");
        }
        ESP_LOGI(TAG, "MultiLoggerAPI created with %s", 
                 physics_analyzer_ ? "Physics Analyzer" : "DataLogger only");
    }

    esp_err_t MultiLoggerAPI::registerEndpoints(httpd_handle_t server)
    {
        if (!server)
        {
            ESP_LOGE(TAG, "HTTP server handle is null");
            return ESP_ERR_INVALID_ARG;
        }

        // Store this instance in user context for all endpoints
        httpd_uri_t list_uri = loggers_list_uri;
        list_uri.user_ctx = this;
        
        httpd_uri_t control_uri = loggers_control_uri;
        control_uri.user_ctx = this;
        
        httpd_uri_t export_uri = data_export_uri;
        export_uri.user_ctx = this;
        
        httpd_uri_t status_uri = logger_status_uri;
        status_uri.user_ctx = this;

        esp_err_t ret = httpd_register_uri_handler(server, &list_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register loggers list endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = httpd_register_uri_handler(server, &control_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register loggers control endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = httpd_register_uri_handler(server, &export_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register data export endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = httpd_register_uri_handler(server, &status_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register logger status endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGI(TAG, "Multi-Logger API endpoints registered successfully");
        return ESP_OK;
    }

    esp_err_t MultiLoggerAPI::unregisterEndpoints(httpd_handle_t server)
    {
        if (!server) return ESP_ERR_INVALID_ARG;

        httpd_unregister_uri_handler(server, loggers_list_uri.uri, loggers_list_uri.method);
        httpd_unregister_uri_handler(server, loggers_control_uri.uri, loggers_control_uri.method);
        httpd_unregister_uri_handler(server, data_export_uri.uri, data_export_uri.method);
        httpd_unregister_uri_handler(server, logger_status_uri.uri, logger_status_uri.method);

        ESP_LOGI(TAG, "Multi-Logger API endpoints unregistered");
        return ESP_OK;
    }

    std::vector<MultiLoggerAPI::LoggerInfo> MultiLoggerAPI::getAvailableLoggers() const
    {
        std::vector<LoggerInfo> loggers;

        // Always include the main data logger
        if (data_logger_)
        {
            loggers.push_back(getDataLoggerInfo());
        }

        // Include physics analyzer if available
        if (physics_analyzer_)
        {
            loggers.push_back(getPhysicsAnalyzerInfo());
        }

        return loggers;
    }

    esp_err_t MultiLoggerAPI::startLoggers(const std::vector<std::string>& logger_ids)
    {
        for (const auto& id : logger_ids)
        {
            ESP_LOGI(TAG, "Starting logger: %s", id.c_str());
            
            if (id == "control_system" && data_logger_)
            {
                if (!data_logger_->isRunning())
                {
                    esp_err_t ret = data_logger_->start();
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to start data logger: %s", esp_err_to_name(ret));
                        return ret;
                    }
                }
            }
            else if (id == "physics_analyzer" && physics_analyzer_)
            {
                if (!physics_analyzer_->isRunning())
                {
                    esp_err_t ret = physics_analyzer_->start();
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to start physics analyzer: %s", esp_err_to_name(ret));
                        return ret;
                    }
                }
            }
            else
            {
                ESP_LOGW(TAG, "Unknown logger ID: %s", id.c_str());
            }
        }

        return ESP_OK;
    }

    esp_err_t MultiLoggerAPI::stopLoggers(const std::vector<std::string>& logger_ids)
    {
        for (const auto& id : logger_ids)
        {
            ESP_LOGI(TAG, "Stopping logger: %s", id.c_str());
            
            if (id == "control_system" && data_logger_)
            {
                if (data_logger_->isRunning())
                {
                    esp_err_t ret = data_logger_->stop();
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to stop data logger: %s", esp_err_to_name(ret));
                        return ret;
                    }
                }
            }
            else if (id == "physics_analyzer" && physics_analyzer_)
            {
                if (physics_analyzer_->isRunning())
                {
                    esp_err_t ret = physics_analyzer_->stop();
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to stop physics analyzer: %s", esp_err_to_name(ret));
                        return ret;
                    }
                }
            }
        }

        return ESP_OK;
    }

    // HTTP endpoint handlers
    esp_err_t MultiLoggerAPI::handleLoggersList(httpd_req_t* req)
    {
        MultiLoggerAPI* api = getAPIFromReq(req);
        if (!api)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "MultiLoggerAPI not available");
            return ESP_FAIL;
        }

        std::string json = api->generateLoggerListJSON();
        return sendJSONResponse(req, json);
    }

    esp_err_t MultiLoggerAPI::handleLoggersControl(httpd_req_t* req)
    {
        MultiLoggerAPI* api = getAPIFromReq(req);
        if (!api)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "MultiLoggerAPI not available");
            return ESP_FAIL;
        }

        // Get query parameters for action and logger IDs
        char query[256];
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        {
            char action[32] = {0};
            char logger_ids[128] = {0};
            
            httpd_query_key_value(query, "action", action, sizeof(action));
            httpd_query_key_value(query, "loggers", logger_ids, sizeof(logger_ids));
            
            // Parse logger IDs (comma-separated)
            std::vector<std::string> ids;
            std::string ids_str(logger_ids);
            size_t pos = 0;
            while ((pos = ids_str.find(',')) != std::string::npos)
            {
                ids.push_back(ids_str.substr(0, pos));
                ids_str.erase(0, pos + 1);
            }
            if (!ids_str.empty())
            {
                ids.push_back(ids_str);
            }
            
            esp_err_t ret = ESP_OK;
            if (strcmp(action, "start") == 0)
            {
                ret = api->startLoggers(ids);
            }
            else if (strcmp(action, "stop") == 0)
            {
                ret = api->stopLoggers(ids);
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid action");
                return ESP_FAIL;
            }
            
            if (ret == ESP_OK)
            {
                return sendJSONResponse(req, "{\"status\":\"success\"}");
            }
            else
            {
                return sendJSONResponse(req, "{\"status\":\"error\"}");
            }
        }
        
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing parameters");
        return ESP_FAIL;
    }

    esp_err_t MultiLoggerAPI::handleDataExport(httpd_req_t* req)
    {
        MultiLoggerAPI* api = getAPIFromReq(req);
        if (!api)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "MultiLoggerAPI not available");
            return ESP_FAIL;
        }

        char query[256];
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        {
            char logger_id[32] = {0};
            char format[16] = "csv";
            char max_entries_str[16] = "0";
            
            httpd_query_key_value(query, "logger", logger_id, sizeof(logger_id));
            httpd_query_key_value(query, "format", format, sizeof(format));
            httpd_query_key_value(query, "max", max_entries_str, sizeof(max_entries_str));
            
            uint32_t max_entries = atoi(max_entries_str);
            
            if (strcmp(logger_id, "control_system") == 0)
            {
                std::string csv = api->generateDataLoggerCSV(max_entries);
                return sendCSVResponse(req, csv, "control_system_data.csv");
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown logger ID");
                return ESP_FAIL;
            }
        }
        
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing parameters");
        return ESP_FAIL;
    }

    esp_err_t MultiLoggerAPI::handleLoggerStatus(httpd_req_t* req)
    {
        MultiLoggerAPI* api = getAPIFromReq(req);
        if (!api)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "MultiLoggerAPI not available");
            return ESP_FAIL;
        }

        return sendJSONResponse(req, api->generateLoggerListJSON());
    }

    // Helper methods
    MultiLoggerAPI* MultiLoggerAPI::getAPIFromReq(httpd_req_t* req)
    {
        return static_cast<MultiLoggerAPI*>(req->user_ctx);
    }

    esp_err_t MultiLoggerAPI::sendJSONResponse(httpd_req_t* req, const std::string& json)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, json.c_str(), json.length());
    }

    esp_err_t MultiLoggerAPI::sendCSVResponse(httpd_req_t* req, const std::string& csv, const std::string& filename)
    {
        httpd_resp_set_type(req, "text/csv");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        std::string disposition = "attachment; filename=\"" + filename + "\"";
        httpd_resp_set_hdr(req, "Content-Disposition", disposition.c_str());
        return httpd_resp_send(req, csv.c_str(), csv.length());
    }

    MultiLoggerAPI::LoggerInfo MultiLoggerAPI::getDataLoggerInfo() const
    {
        LoggerInfo info;
        info.id = "control_system";
        info.name = "Control System Logger";
        info.type = "control_data";
        info.enabled = data_logger_ != nullptr;
        info.running = data_logger_ ? data_logger_->isRunning() : false;
        info.total_entries = data_logger_ ? data_logger_->getEntryCount() : 0;
        info.memory_usage_percent = data_logger_ ? data_logger_->getMemoryUsage() : 0.0f;
        info.status = info.running ? "Running" : "Stopped";
        return info;
    }

    MultiLoggerAPI::LoggerInfo MultiLoggerAPI::getPhysicsAnalyzerInfo() const
    {
        LoggerInfo info;
        info.id = "physics_analyzer";
        info.name = "Physics Analyzer";
        info.type = "physics_analysis";
        info.enabled = physics_analyzer_ != nullptr;
        info.running = physics_analyzer_ ? physics_analyzer_->isRunning() : false;
        info.total_entries = 0; // Physics analyzer doesn't store entries directly
        info.memory_usage_percent = 0.0f; // Minimal memory usage
        info.status = info.running ? "Analyzing" : "Stopped";
        return info;
    }

    std::string MultiLoggerAPI::generateDataLoggerCSV(uint32_t max_entries) const
    {
        if (!data_logger_)
        {
            return "Error: DataLogger not available\n";
        }

        auto data = data_logger_->getCollectedData(max_entries);
        if (data.empty())
        {
            return "Error: No data available\n";
        }

        std::ostringstream csv;
        csv << "Timestamp_us,Key,Value\n";

        for (const auto& entry : data)
        {
            csv << entry.timestamp_us << ","
                << entry.key << ","
                << std::fixed << std::setprecision(3) << entry.value << "\n";
        }

        return csv.str();
    }

    std::string MultiLoggerAPI::generateLoggerListJSON() const
    {
        auto loggers = getAvailableLoggers();
        
        std::ostringstream json;
        json << "{\n  \"loggers\": [\n";
        
        for (size_t i = 0; i < loggers.size(); ++i)
        {
            const auto& logger = loggers[i];
            json << "    {\n";
            json << "      \"id\": \"" << logger.id << "\",\n";
            json << "      \"name\": \"" << logger.name << "\",\n";
            json << "      \"type\": \"" << logger.type << "\",\n";
            json << "      \"enabled\": " << (logger.enabled ? "true" : "false") << ",\n";
            json << "      \"running\": " << (logger.running ? "true" : "false") << ",\n";
            json << "      \"total_entries\": " << logger.total_entries << ",\n";
            json << "      \"memory_usage_percent\": " << std::fixed << std::setprecision(1) << logger.memory_usage_percent << ",\n";
            json << "      \"status\": \"" << logger.status << "\"\n";
            json << "    }";
            if (i < loggers.size() - 1) json << ",";
            json << "\n";
        }
        
        json << "  ],\n";
        json << "  \"timestamp\": " << esp_timer_get_time() << "\n";
        json << "}\n";
        
        return json.str();
    }
}
