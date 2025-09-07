#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <vector>
#include <string>
#include <memory>

namespace digitoys::datalogger
{
    class DataLogger;
    class PhysicsAnalyzer;

    /**
     * @brief Multi-Logger Management API
     * 
     * Provides HTTP endpoints for managing multiple data loggers
     * and exporting their data in various formats.
     */
    class MultiLoggerAPI
    {
    public:
        /**
         * @brief Logger information structure
         */
        struct LoggerInfo
        {
            std::string id;
            std::string name;
            std::string type;
            bool enabled;
            bool running;
            uint32_t total_entries;
            float memory_usage_percent;
            std::string status;
        };

        /**
         * @brief Logger export configuration
         */
        struct ExportConfig
        {
            std::string logger_id;
            std::string format; // "csv", "json"
            uint32_t max_entries;
            bool include_physics;
        };

        /**
         * @brief Constructor
         * @param data_logger Primary data logger instance
         * @param physics_analyzer Physics analyzer instance (optional)
         */
        MultiLoggerAPI(DataLogger* data_logger, PhysicsAnalyzer* physics_analyzer = nullptr);

        /**
         * @brief Register HTTP endpoints with the server
         * @param server HTTP server handle
         * @return ESP_OK on success
         */
        esp_err_t registerEndpoints(httpd_handle_t server);

        /**
         * @brief Unregister HTTP endpoints from the server
         * @param server HTTP server handle
         * @return ESP_OK on success
         */
        esp_err_t unregisterEndpoints(httpd_handle_t server);

        /**
         * @brief Get list of available loggers
         * @return Vector of logger information
         */
        std::vector<LoggerInfo> getAvailableLoggers() const;

        /**
         * @brief Start logging for selected loggers
         * @param logger_ids List of logger IDs to start
         * @return ESP_OK on success
         */
        esp_err_t startLoggers(const std::vector<std::string>& logger_ids);

        /**
         * @brief Stop logging for selected loggers
         * @param logger_ids List of logger IDs to stop
         * @return ESP_OK on success
         */
        esp_err_t stopLoggers(const std::vector<std::string>& logger_ids);

    private:
        static const char* TAG;
        
        DataLogger* data_logger_;
        PhysicsAnalyzer* physics_analyzer_;

    public:
        // HTTP endpoint handlers (made public for C-style callbacks)
        static esp_err_t handleLoggersList(httpd_req_t* req);
        static esp_err_t handleLoggersControl(httpd_req_t* req);
        static esp_err_t handleDataExport(httpd_req_t* req);
        static esp_err_t handleLoggerStatus(httpd_req_t* req);

    private:

        // Helper methods
        static MultiLoggerAPI* getAPIFromReq(httpd_req_t* req);
        static esp_err_t sendJSONResponse(httpd_req_t* req, const std::string& json);
        static esp_err_t sendCSVResponse(httpd_req_t* req, const std::string& csv, const std::string& filename);
        
        LoggerInfo getDataLoggerInfo() const;
        LoggerInfo getPhysicsAnalyzerInfo() const;
        std::string generateDataLoggerCSV(uint32_t max_entries) const;
        std::string generateLoggerListJSON() const;
    };
}
