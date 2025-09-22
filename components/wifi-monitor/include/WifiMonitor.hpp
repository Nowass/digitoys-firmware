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

// No external logger/modeling dependencies anymore

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
     * @brief Unified telemetry frame for high-fidelity streaming
     * Flat schema; single source of truth for real-time dashboard & logging.
     * Times are microseconds since boot (esp_timer_get_time). Sequence wraps on overflow.
     */
    struct TelemetryFrame
    {
        uint64_t ts_us = 0;               ///< Monotonic timestamp (µs)
        uint32_t seq = 0;                 ///< Incrementing sequence id (wraps)
        float rc_duty_raw = 0.0f;         ///< Raw RC duty (e.g. 0.06-0.12)
        bool rc_throttle_pressed = false; ///< Any non-neutral throttle pressed
        bool rc_forward = false;          ///< Forward intent flag
        bool rc_reverse = false;          ///< Reverse intent flag
        float lidar_distance_m = 0.0f;    ///< Latest LiDAR distance (meters)
        float lidar_filtered_m = 0.0f;    ///< Exponentially smoothed LiDAR distance
        bool obstacle_detected = false;   ///< Obstacle condition (brake)
        bool warning_active = false;      ///< Warning condition
        float brake_distance_m = 0.0f;    ///< Dynamic brake distance threshold
        float warning_distance_m = 0.0f;  ///< Dynamic warning distance threshold
        float safety_margin_m = 0.0f;     ///< (distance - brake_distance) if obstacle/warning relevant
        float speed_approx_mps = 0.0f;    ///< Approximate forward speed (derived)
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
        // Default constructor
        WifiMonitor() : ComponentBase("WifiMonitor") {}

        // No-op setters removed (logger/modeling detached)

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
         * @brief Submit a unified telemetry frame for streaming.
         * Computes derived fields (seq, speed_approx, safety_margin) and stores last frame.
         * Thread-safe.
         * @param frame Partially filled frame (ts_us may be 0 to auto-fill)
         */
        void submitTelemetryFrame(const TelemetryFrame &frame);

        /**
         * @brief Retrieve the most recent telemetry frame.
         * @param out Destination frame
         * @return true if a frame was available
         */
        bool getLastTelemetryFrame(TelemetryFrame &out) const;

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

        // DataLogger export methods removed

        /**
         * @brief Add system log entry for console display
         * @param log_line Complete formatted log line from ESP-IDF
         */
        void addSystemLogEntry(const std::string &log_line);

        /**
         * @brief Get recent system logs for console display
         * @param max_entries Maximum number of recent entries to return
         * @return Vector of recent log lines
         */
        std::vector<std::string> getRecentSystemLogs(size_t max_entries = 50) const;

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
        void cleanupDisconnectedClients();

        // HTTP request handlers
        static esp_err_t telemetryGetHandler(httpd_req_t *req);
        static esp_err_t systemGetHandler(httpd_req_t *req);
        static esp_err_t indexGetHandler(httpd_req_t *req);
        static esp_err_t websocketHandler(httpd_req_t *req);
        static esp_err_t websocketDataHandler(httpd_req_t *req);
        static esp_err_t websocketCsvHandler(httpd_req_t *req);
        static esp_err_t loggingControlHandler(httpd_req_t *req);
        // Legacy DataLogger logging data handler removed

        // Helper methods
        static esp_err_t addCorsHeaders(httpd_req_t *req);
        static bool isWebSocketFrame(httpd_req_t *req);

    private:
        // Unified telemetry frame storage (protected by telemetry_mutex_)
        TelemetryFrame last_frame_{};   ///< Most recent telemetry frame
        TelemetryFrame prev_frame_{};   ///< Previous telemetry frame (for derivatives)
        bool last_frame_valid_ = false; ///< Indicates at least one frame submitted
        bool prev_frame_valid_ = false; ///< Indicates previous frame valid
        uint32_t frame_seq_ = 0;        ///< Sequence counter

        // Service integrations removed

        // Network interfaces
        esp_netif_t *ap_netif_ = nullptr;

        // HTTP server
        httpd_handle_t server_ = nullptr;

        // WebSocket clients management
        std::vector<int> websocket_clients_;
        std::vector<int> websocket_data_clients_; // Separate list for data streaming clients
        std::vector<int> websocket_csv_clients_;  // Dedicated CSV clients (header + rows)
        SemaphoreHandle_t ws_clients_mutex_ = nullptr;

        // Simple counters for CSV streaming status
        uint32_t csv_rows_sent_ = 0; ///< Number of CSV rows sent this session
        size_t csv_bytes_sent_ = 0;  ///< Approx bytes sent (optional UI)

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

        // CSV logging for TelemetryFrame
        // CSV file members removed; streaming only.

        // System log capture for console display
        std::vector<std::string> system_log_buffer_;
        SemaphoreHandle_t system_log_mutex_ = nullptr;
        static constexpr size_t MAX_SYSTEM_LOG_ENTRIES = 100; // Recent system logs for console

        // Static instance for HTTP handlers
        static WifiMonitor *instance_;

        // Log capture hook
        static int logHook(const char *format, va_list args);
    };

} // namespace wifi_monitor
