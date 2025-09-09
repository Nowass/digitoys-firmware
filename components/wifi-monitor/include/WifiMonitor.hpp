#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <ComponentBase.hpp>
#include <IMonitor.hpp>
#include <Constants.hpp>
#include <vector>
#include <string>

// Forward declare to avoid circular dependency
namespace digitoys::datalogger {
    class DataLoggerService;
}

namespace wifi_monitor
{
    /**
     * @brief Telemetry data structure for real-time monitoring
     */
    struct Telemetry
    {
        bool obstacle = false;  ///< Obstacle detected flag
        bool warning = false;   ///< Warning condition flag
        float distance = 0.0f;  ///< Distance to obstacle (meters)
        float speed_est = 0.0f; ///< Estimated speed
        uint32_t timestamp = 0; ///< Timestamp of the data
    };

    /**
     * @brief Diagnostic log entry capturing control system state
     */
    struct DiagnosticEntry
    {
        uint32_t timestamp = 0;         ///< Timestamp of the entry
        float cached_duty = 0.0f;       ///< Cached duty cycle reading
        float direct_duty = 0.0f;       ///< Direct duty cycle reading
        float current_input = 0.0f;     ///< Current RC input duty cycle
        float distance = 0.0f;          ///< LiDAR distance reading
        float brake_distance = 0.0f;    ///< Dynamic brake distance
        float warning_distance = 0.0f;  ///< Dynamic warning distance
        bool cached_throttle = false;   ///< Cached throttle state
        bool throttle_pressed = false;  ///< Current throttle state
        bool driving_forward = false;   ///< Forward driving flag
        bool wants_reverse = false;     ///< Reverse input flag
        bool obstacle_detected = false; ///< Obstacle detection flag
        bool warning_active = false;    ///< Warning condition flag
    };

    /**
     * @brief WiFi Monitor component providing AP mode with real-time telemetry
     *
     * This component creates a WiFi Access Point with built-in DHCP server
     * and provides both HTTP REST endpoints and WebSocket streaming for
     * real-time telemetry data. It's designed to be a replacement for the
     * original monitor component with improved performance and reliability.
     *
     * Features:
     * - WiFi AP mode (192.168.4.1) with DHCP (192.168.4.100-110)
     * - HTTP REST API (/telemetry, /system, /)
     * - WebSocket streaming for real-time updates (20Hz)
     * - Thread-safe telemetry data access
     * - Integrated with digitoys-core framework
     */
    class WifiMonitor : public digitoys::core::ComponentBase, public digitoys::core::IMonitor
    {
    public:
        /**
         * @brief Constructor
         */
        /**
         * @brief Construct the WiFi Monitor component
         * @param data_logger_service Optional DataLogger service for logging control
         */
        WifiMonitor(digitoys::datalogger::DataLoggerService* data_logger_service = nullptr) 
            : ComponentBase("WifiMonitor"), data_logger_service_(data_logger_service) {}

        /**
         * @brief Set the DataLogger service for logging control (can be called after construction)
         */
        void setDataLoggerService(digitoys::datalogger::DataLoggerService* data_logger_service) {
            data_logger_service_ = data_logger_service;
            setupDataLoggerStreaming();
        }

        /**
         * @brief Destructor
         */
        ~WifiMonitor() override = default;

        // Disable copy constructor and assignment operator
        WifiMonitor(const WifiMonitor &) = delete;
        WifiMonitor &operator=(const WifiMonitor &) = delete;

        // IComponent interface implementation
        esp_err_t initialize() override;
        esp_err_t start() override;
        esp_err_t stop() override;
        esp_err_t shutdown() override;

        /**
         * @brief Update telemetry data for monitoring
         * @param obstacle Obstacle detection flag
         * @param distance Distance to obstacle in meters
         * @param speed_est Estimated speed
         * @param warning Warning condition flag
         */
        void updateTelemetry(bool obstacle, float distance, float speed_est, bool warning);

        /**
         * @brief Get current telemetry data (thread-safe)
         * @param data Output telemetry structure
         * @return ESP_OK on success, ESP_ERR_TIMEOUT if mutex unavailable
         */
        esp_err_t getTelemetry(Telemetry &data) const;

        /**
         * @brief Add diagnostic log entry from control system
         * @param entry Diagnostic data to log
         */
        void addDiagnosticEntry(const DiagnosticEntry &entry);

        /**
         * @brief Convenience method to log control system diagnostics
         * @param cached_duty Cached duty cycle reading
         * @param direct_duty Direct duty cycle reading
         * @param current_input Current RC input
         * @param distance LiDAR distance reading
         * @param brake_distance Dynamic brake distance
         * @param warning_distance Dynamic warning distance
         * @param cached_throttle Cached throttle state
         * @param throttle_pressed Current throttle state
         * @param driving_forward Forward driving flag
         * @param wants_reverse Reverse input flag
         * @param obstacle_detected Obstacle detection flag
         * @param warning_active Warning condition flag
         */
        void logControlDiagnostics(float cached_duty, float direct_duty, float current_input,
                                   float distance, float brake_distance, float warning_distance,
                                   bool cached_throttle, bool throttle_pressed, bool driving_forward,
                                   bool wants_reverse, bool obstacle_detected, bool warning_active);

        /**
         * @brief Start diagnostic logging
         * @return ESP_OK on success
         */
        esp_err_t startLogging();

        /**
         * @brief Stop diagnostic logging
         * @return ESP_OK on success
         */
        esp_err_t stopLogging();

        /**
         * @brief Check if logging is active
         * @return true if logging is active
         */
        bool isLoggingActive() const;

        /**
         * @brief Get logged diagnostic data as JSON
         * @return JSON string with diagnostic entries
         */
        std::string getDiagnosticDataJSON() const;

        /**
         * @brief Get logged diagnostic data as CSV
         * @return CSV string with diagnostic entries
         */
        std::string getDiagnosticDataCSV() const;

        /**
         * @brief Get DataLogger physics data as JSON
         * @return JSON string with physics data entries
         */
        std::string getDataLoggerJSON() const;

        /**
         * @brief Get DataLogger physics data as CSV
         * @return CSV string with physics data entries
         */
        std::string getDataLoggerCSV() const;

        /**
         * @brief Clear all logged diagnostic data
         */
        void clearDiagnosticData();

        /**
         * @brief Get HTTP server handle for external endpoint registration
         * @return HTTP server handle (nullptr if not started)
         */
        httpd_handle_t getHttpServerHandle() const { return server_; }

    private:
        // WiFi and Network setup
        esp_err_t setupWifiAP();
        esp_err_t configureDHCP();
        esp_err_t teardownWifi();

        // HTTP Server management
        esp_err_t startHttpServer();
        esp_err_t stopHttpServer();
        esp_err_t registerHttpHandlers();

        // WebSocket management
        esp_err_t startWebSocketTask();
        esp_err_t stopWebSocketTask();
        static void webSocketTaskFunction(void *param);
        void webSocketBroadcastLoop();
        esp_err_t broadcastTelemetry();
        esp_err_t broadcastDataEntry(const std::string& data_json);
        void cleanupDisconnectedClients();
        void setupDataLoggerStreaming();

        // HTTP request handlers
        static esp_err_t telemetryGetHandler(httpd_req_t *req);
        static esp_err_t systemGetHandler(httpd_req_t *req);
        static esp_err_t indexGetHandler(httpd_req_t *req);
        static esp_err_t websocketHandler(httpd_req_t *req);
        static esp_err_t websocketDataHandler(httpd_req_t *req);
        static esp_err_t loggingControlHandler(httpd_req_t *req);
        static esp_err_t loggingDataHandler(httpd_req_t *req);

        // Helper methods
        static esp_err_t addCorsHeaders(httpd_req_t *req);
        bool isWebSocketFrame(httpd_req_t *req);

    private:
        // DataLogger integration
        digitoys::datalogger::DataLoggerService* data_logger_service_ = nullptr;

        // Network interfaces
        esp_netif_t *ap_netif_ = nullptr;

        // HTTP server
        httpd_handle_t server_ = nullptr;

        // WebSocket clients management
        std::vector<int> websocket_clients_;
        std::vector<int> websocket_data_clients_;  // Separate list for data streaming clients
        SemaphoreHandle_t ws_clients_mutex_ = nullptr;

        // Telemetry data protection
        Telemetry telemetry_data_{};
        SemaphoreHandle_t telemetry_mutex_ = nullptr;

        // WebSocket broadcast task
        TaskHandle_t ws_task_handle_ = nullptr;
        volatile bool ws_task_running_ = false;

        // Diagnostic logging
        std::vector<DiagnosticEntry> diagnostic_log_;
        SemaphoreHandle_t diagnostic_mutex_ = nullptr;
        bool logging_active_ = false;
        static constexpr size_t MAX_LOG_ENTRIES = 1000; // Limit log size

        // Static instance for HTTP handlers
        static WifiMonitor *instance_;
    };

} // namespace wifi_monitor
