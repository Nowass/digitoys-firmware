#pragma once

#include "PhysicsAnalyzer.hpp"
#include <esp_http_server.h>
#include <string>

namespace digitoys::datalogger
{
    /**
     * @brief HTTP API handler for PhysicsAnalyzer data export
     *
     * Provides HTTP endpoints for accessing physics analysis data:
     * - CSV export of analysis results
     * - Real-time physics metrics
     * - Braking event history
     * - Configuration management
     */
    class PhysicsAnalyzerAPI
    {
    public:
        /**
         * @brief Constructor
         * @param analyzer Pointer to PhysicsAnalyzer instance
         */
        explicit PhysicsAnalyzerAPI(PhysicsAnalyzer *analyzer);

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

    private:
        static const char *TAG;
        PhysicsAnalyzer *analyzer_;

        // HTTP endpoint handlers
        static esp_err_t handleAnalysisResultsCSV(httpd_req_t *req);
        static esp_err_t handleBrakingEventsCSV(httpd_req_t *req);
        static esp_err_t handleRealTimeMetricsCSV(httpd_req_t *req);
        static esp_err_t handleAnalysisConfig(httpd_req_t *req);
        static esp_err_t handleAnalysisStatus(httpd_req_t *req);

        // CSV generation helpers
        static std::string generateAnalysisResultsCSV(const PhysicsAnalyzer::AnalysisResults &results);
        static std::string generateBrakingEventsCSV(const std::vector<PhysicsAnalyzer::BrakingEvent> &events);
        static std::string generateRealTimeMetricsCSV(PhysicsAnalyzer *analyzer);

        // Utility functions
        static PhysicsAnalyzer *getAnalyzerFromReq(httpd_req_t *req);
        static esp_err_t sendCSVResponse(httpd_req_t *req, const std::string &csv_data, const char *filename);
        static esp_err_t sendJSONResponse(httpd_req_t *req, const std::string &json_data);

        // URI handlers
        static const httpd_uri_t analysis_results_uri_;
        static const httpd_uri_t braking_events_uri_;
        static const httpd_uri_t realtime_metrics_uri_;
        static const httpd_uri_t analysis_config_uri_;
        static const httpd_uri_t analysis_status_uri_;
    };

} // namespace digitoys::datalogger
