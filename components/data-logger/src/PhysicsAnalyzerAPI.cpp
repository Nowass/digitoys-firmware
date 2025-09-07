#include "PhysicsAnalyzerAPI.hpp"
#include <esp_log.h>
#include <sstream>
#include <iomanip>

namespace digitoys::datalogger
{
    const char *PhysicsAnalyzerAPI::TAG = "PhysicsAnalyzerAPI";

    // URI handler definitions
    const httpd_uri_t PhysicsAnalyzerAPI::analysis_results_uri_ = {
        .uri = "/api/physics/analysis.csv",
        .method = HTTP_GET,
        .handler = handleAnalysisResultsCSV,
        .user_ctx = nullptr};

    const httpd_uri_t PhysicsAnalyzerAPI::braking_events_uri_ = {
        .uri = "/api/physics/braking-events.csv",
        .method = HTTP_GET,
        .handler = handleBrakingEventsCSV,
        .user_ctx = nullptr};

    const httpd_uri_t PhysicsAnalyzerAPI::realtime_metrics_uri_ = {
        .uri = "/api/physics/realtime.csv",
        .method = HTTP_GET,
        .handler = handleRealTimeMetricsCSV,
        .user_ctx = nullptr};

    const httpd_uri_t PhysicsAnalyzerAPI::analysis_config_uri_ = {
        .uri = "/api/physics/config",
        .method = HTTP_GET,
        .handler = handleAnalysisConfig,
        .user_ctx = nullptr};

    const httpd_uri_t PhysicsAnalyzerAPI::analysis_status_uri_ = {
        .uri = "/api/physics/status",
        .method = HTTP_GET,
        .handler = handleAnalysisStatus,
        .user_ctx = nullptr};

    PhysicsAnalyzerAPI::PhysicsAnalyzerAPI(PhysicsAnalyzer *analyzer)
        : analyzer_(analyzer)
    {
        if (!analyzer_)
        {
            ESP_LOGE(TAG, "PhysicsAnalyzer instance is required");
        }
        ESP_LOGI(TAG, "PhysicsAnalyzerAPI created");
    }

    esp_err_t PhysicsAnalyzerAPI::registerEndpoints(httpd_handle_t server)
    {
        if (!server)
        {
            ESP_LOGE(TAG, "HTTP server handle is required");
            return ESP_ERR_INVALID_ARG;
        }

        ESP_LOGI(TAG, "Registering physics analysis HTTP endpoints...");

        // Store analyzer instance in user context for all handlers
        httpd_uri_t analysis_results_uri = analysis_results_uri_;
        analysis_results_uri.user_ctx = analyzer_;

        httpd_uri_t braking_events_uri = braking_events_uri_;
        braking_events_uri.user_ctx = analyzer_;

        httpd_uri_t realtime_metrics_uri = realtime_metrics_uri_;
        realtime_metrics_uri.user_ctx = analyzer_;

        httpd_uri_t analysis_config_uri = analysis_config_uri_;
        analysis_config_uri.user_ctx = analyzer_;

        httpd_uri_t analysis_status_uri = analysis_status_uri_;
        analysis_status_uri.user_ctx = analyzer_;

        // Register all endpoints
        esp_err_t ret = httpd_register_uri_handler(server, &analysis_results_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register analysis results endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = httpd_register_uri_handler(server, &braking_events_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register braking events endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = httpd_register_uri_handler(server, &realtime_metrics_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register realtime metrics endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = httpd_register_uri_handler(server, &analysis_config_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register analysis config endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = httpd_register_uri_handler(server, &analysis_status_uri);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to register analysis status endpoint: %s", esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGI(TAG, "Physics analysis HTTP endpoints registered successfully");
        ESP_LOGI(TAG, "Available endpoints:");
        ESP_LOGI(TAG, "  GET /api/physics/analysis.csv - Analysis results CSV");
        ESP_LOGI(TAG, "  GET /api/physics/braking-events.csv - Braking events CSV");
        ESP_LOGI(TAG, "  GET /api/physics/realtime.csv - Real-time metrics CSV");
        ESP_LOGI(TAG, "  GET /api/physics/config - Analysis configuration JSON");
        ESP_LOGI(TAG, "  GET /api/physics/status - Analysis status JSON");

        return ESP_OK;
    }

    esp_err_t PhysicsAnalyzerAPI::unregisterEndpoints(httpd_handle_t server)
    {
        if (!server)
        {
            return ESP_OK;
        }

        ESP_LOGI(TAG, "Unregistering physics analysis HTTP endpoints...");

        httpd_unregister_uri_handler(server, analysis_results_uri_.uri, analysis_results_uri_.method);
        httpd_unregister_uri_handler(server, braking_events_uri_.uri, braking_events_uri_.method);
        httpd_unregister_uri_handler(server, realtime_metrics_uri_.uri, realtime_metrics_uri_.method);
        httpd_unregister_uri_handler(server, analysis_config_uri_.uri, analysis_config_uri_.method);
        httpd_unregister_uri_handler(server, analysis_status_uri_.uri, analysis_status_uri_.method);

        ESP_LOGI(TAG, "Physics analysis HTTP endpoints unregistered");
        return ESP_OK;
    }

    esp_err_t PhysicsAnalyzerAPI::handleAnalysisResultsCSV(httpd_req_t *req)
    {
        PhysicsAnalyzer *analyzer = getAnalyzerFromReq(req);
        if (!analyzer)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "PhysicsAnalyzer not available");
            return ESP_FAIL;
        }

        const auto &results = analyzer->getLatestResults();
        std::string csv_data = generateAnalysisResultsCSV(results);

        return sendCSVResponse(req, csv_data, "physics_analysis.csv");
    }

    esp_err_t PhysicsAnalyzerAPI::handleBrakingEventsCSV(httpd_req_t *req)
    {
        PhysicsAnalyzer *analyzer = getAnalyzerFromReq(req);
        if (!analyzer)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "PhysicsAnalyzer not available");
            return ESP_FAIL;
        }

        std::vector<PhysicsAnalyzer::BrakingEvent> events = analyzer->getBrakingEvents();
        std::string csv_data = generateBrakingEventsCSV(events);

        return sendCSVResponse(req, csv_data, "braking_events.csv");
    }

    esp_err_t PhysicsAnalyzerAPI::handleRealTimeMetricsCSV(httpd_req_t *req)
    {
        PhysicsAnalyzer *analyzer = getAnalyzerFromReq(req);
        if (!analyzer)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "PhysicsAnalyzer not available");
            return ESP_FAIL;
        }

        std::string csv_data = generateRealTimeMetricsCSV(analyzer);

        return sendCSVResponse(req, csv_data, "realtime_metrics.csv");
    }

    esp_err_t PhysicsAnalyzerAPI::handleAnalysisConfig(httpd_req_t *req)
    {
        PhysicsAnalyzer *analyzer = getAnalyzerFromReq(req);
        if (!analyzer)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "PhysicsAnalyzer not available");
            return ESP_FAIL;
        }

        const auto &config = analyzer->getConfig();

        std::ostringstream json;
        json << std::fixed << std::setprecision(2);
        json << "{\n";
        json << "  \"enabled\": " << (config.enabled ? "true" : "false") << ",\n";
        json << "  \"analysis_interval_ms\": " << config.analysis_interval_ms << ",\n";
        json << "  \"data_window_size\": " << config.data_window_size << ",\n";
        json << "  \"emergency_decel_threshold\": " << config.emergency_decel_threshold << ",\n";
        json << "  \"safety_margin_critical\": " << config.safety_margin_critical << ",\n";
        json << "  \"safety_margin_warning\": " << config.safety_margin_warning << ",\n";
        json << "  \"g_force_warning\": " << config.g_force_warning << ",\n";
        json << "  \"g_force_critical\": " << config.g_force_critical << "\n";
        json << "}";

        return sendJSONResponse(req, json.str());
    }

    esp_err_t PhysicsAnalyzerAPI::handleAnalysisStatus(httpd_req_t *req)
    {
        PhysicsAnalyzer *analyzer = getAnalyzerFromReq(req);
        if (!analyzer)
        {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "PhysicsAnalyzer not available");
            return ESP_FAIL;
        }

        uint32_t total_analyses;
        float avg_analysis_time;
        uint64_t last_analysis_time;
        analyzer->getStatistics(total_analyses, avg_analysis_time, last_analysis_time);

        std::ostringstream json;
        json << std::fixed << std::setprecision(2);
        json << "{\n";
        json << "  \"running\": " << (analyzer->isRunning() ? "true" : "false") << ",\n";
        json << "  \"analyzing\": " << (analyzer->isAnalyzing() ? "true" : "false") << ",\n";
        json << "  \"total_analyses\": " << total_analyses << ",\n";
        json << "  \"avg_analysis_time_ms\": " << avg_analysis_time << ",\n";
        json << "  \"last_analysis_timestamp\": " << last_analysis_time << ",\n";
        json << "  \"current_timestamp\": " << esp_timer_get_time() << "\n";
        json << "}";

        return sendJSONResponse(req, json.str());
    }

    std::string PhysicsAnalyzerAPI::generateAnalysisResultsCSV(const PhysicsAnalyzer::AnalysisResults &results)
    {
        std::ostringstream csv;
        csv << std::fixed << std::setprecision(3);

        // CSV Header
        csv << "Metric,Value,Unit,Timestamp\n";

        // Braking Performance
        csv << "Average Deceleration," << results.average_deceleration << ",m/s²," << results.analysis_timestamp << "\n";
        csv << "Peak Deceleration," << results.peak_deceleration << ",m/s²," << results.analysis_timestamp << "\n";
        csv << "Braking Efficiency," << results.braking_efficiency << ",ratio," << results.analysis_timestamp << "\n";
        csv << "Total Brake Events," << results.total_brake_events << ",count," << results.analysis_timestamp << "\n";

        // Safety Analysis
        csv << "Average Safety Margin," << results.average_safety_margin << ",cm," << results.analysis_timestamp << "\n";
        csv << "Minimum Safety Margin," << results.minimum_safety_margin << ",cm," << results.analysis_timestamp << "\n";
        csv << "Safety Violations," << results.safety_violations << ",count," << results.analysis_timestamp << "\n";
        csv << "Near Miss Events," << results.near_miss_events << ",count," << results.analysis_timestamp << "\n";

        // Performance Trends
        csv << "Reaction Time Average," << results.reaction_time_avg << ",ms," << results.analysis_timestamp << "\n";
        csv << "Stopping Distance Average," << results.stopping_distance_avg << ",cm," << results.analysis_timestamp << "\n";
        csv << "G-Force Peak," << results.g_force_peak << ",G," << results.analysis_timestamp << "\n";

        // Risk Assessment
        csv << "Risk Score," << results.risk_score << ",score," << results.analysis_timestamp << "\n";
        csv << "Emergency Pattern Detected," << (results.emergency_pattern_detected ? "1" : "0") << ",boolean," << results.analysis_timestamp << "\n";

        return csv.str();
    }

    std::string PhysicsAnalyzerAPI::generateBrakingEventsCSV(const std::vector<PhysicsAnalyzer::BrakingEvent> &events)
    {
        std::ostringstream csv;
        csv << std::fixed << std::setprecision(3);

        // CSV Header
        csv << "Event_ID,Start_Timestamp,End_Timestamp,Duration_ms,Initial_Speed,Final_Speed,Peak_Deceleration,Average_Deceleration,Stopping_Distance,Safety_Margin_Start,Safety_Margin_End,Emergency_Brake,Reaction_Time\n";

        // Events data
        for (size_t i = 0; i < events.size(); ++i)
        {
            const auto &event = events[i];
            uint64_t duration_ms = (event.end_timestamp - event.start_timestamp) / 1000;

            csv << i + 1 << ",";
            csv << event.start_timestamp << ",";
            csv << event.end_timestamp << ",";
            csv << duration_ms << ",";
            csv << event.initial_speed << ",";
            csv << event.final_speed << ",";
            csv << event.peak_deceleration << ",";
            csv << event.average_deceleration << ",";
            csv << event.stopping_distance << ",";
            csv << event.safety_margin_start << ",";
            csv << event.safety_margin_end << ",";
            csv << (event.emergency_brake ? "1" : "0") << ",";
            csv << event.reaction_time << "\n";
        }

        return csv.str();
    }

    std::string PhysicsAnalyzerAPI::generateRealTimeMetricsCSV(PhysicsAnalyzer *analyzer)
    {
        std::ostringstream csv;
        csv << std::fixed << std::setprecision(3);

        uint64_t current_time = esp_timer_get_time();
        const auto &results = analyzer->getLatestResults();

        uint32_t total_analyses;
        float avg_analysis_time;
        uint64_t last_analysis_time;
        analyzer->getStatistics(total_analyses, avg_analysis_time, last_analysis_time);

        // CSV Header
        csv << "Timestamp,Metric,Value,Unit\n";

        // Current metrics
        csv << current_time << ",Current Risk Score," << results.risk_score << ",score\n";
        csv << current_time << ",Peak Deceleration," << results.peak_deceleration << ",m/s²\n";
        csv << current_time << ",Safety Margin," << results.minimum_safety_margin << ",cm\n";
        csv << current_time << ",G-Force Peak," << results.g_force_peak << ",G\n";
        csv << current_time << ",Total Brake Events," << results.total_brake_events << ",count\n";
        csv << current_time << ",Safety Violations," << results.safety_violations << ",count\n";
        csv << current_time << ",Analysis Running," << (analyzer->isRunning() ? "1" : "0") << ",boolean\n";
        csv << current_time << ",Emergency Pattern," << (results.emergency_pattern_detected ? "1" : "0") << ",boolean\n";

        return csv.str();
    }

    PhysicsAnalyzer *PhysicsAnalyzerAPI::getAnalyzerFromReq(httpd_req_t *req)
    {
        return static_cast<PhysicsAnalyzer *>(req->user_ctx);
    }

    esp_err_t PhysicsAnalyzerAPI::sendCSVResponse(httpd_req_t *req, const std::string &csv_data, const char *filename)
    {
        // Set CSV content type and download headers
        httpd_resp_set_type(req, "text/csv");

        std::string content_disposition = "attachment; filename=\"";
        content_disposition += filename;
        content_disposition += "\"";
        httpd_resp_set_hdr(req, "Content-Disposition", content_disposition.c_str());
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

        // Send CSV data
        esp_err_t ret = httpd_resp_send(req, csv_data.c_str(), csv_data.length());
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send CSV response: %s", esp_err_to_name(ret));
        }

        return ret;
    }

    esp_err_t PhysicsAnalyzerAPI::sendJSONResponse(httpd_req_t *req, const std::string &json_data)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

        esp_err_t ret = httpd_resp_send(req, json_data.c_str(), json_data.length());
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send JSON response: %s", esp_err_to_name(ret));
        }

        return ret;
    }

} // namespace digitoys::datalogger
