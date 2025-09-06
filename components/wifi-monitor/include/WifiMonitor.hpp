#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <ComponentBase.hpp>
#include <Constants.hpp>
#include <vector>

namespace wifi_monitor
{
    /**
     * @brief Telemetry data structure for real-time monitoring
     */
    struct Telemetry
    {
        bool obstacle = false;      ///< Obstacle detected flag
        bool warning = false;       ///< Warning condition flag
        float distance = 0.0f;      ///< Distance to obstacle (meters)
        float speed_est = 0.0f;     ///< Estimated speed
        uint32_t timestamp = 0;     ///< Timestamp of the data
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
    class WifiMonitor : public digitoys::core::ComponentBase
    {
    public:
        /**
         * @brief Constructor
         */
        WifiMonitor() : ComponentBase("WifiMonitor") {}
        
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
        static void webSocketTaskFunction(void* param);
        void webSocketBroadcastLoop();
        esp_err_t broadcastTelemetry();
        void cleanupDisconnectedClients();

        // HTTP request handlers
        static esp_err_t telemetryGetHandler(httpd_req_t *req);
        static esp_err_t systemGetHandler(httpd_req_t *req);
        static esp_err_t indexGetHandler(httpd_req_t *req);
        static esp_err_t websocketHandler(httpd_req_t *req);

        // Utility methods
        esp_err_t addCorsHeaders(httpd_req_t *req);
        bool isWebSocketFrame(httpd_req_t *req);

    private:
        // Network interfaces
        esp_netif_t* ap_netif_ = nullptr;
        
        // HTTP server
        httpd_handle_t server_ = nullptr;
        
        // WebSocket clients management
        std::vector<int> websocket_clients_;
        SemaphoreHandle_t ws_clients_mutex_ = nullptr;
        
        // Telemetry data protection
        Telemetry telemetry_data_{};
        SemaphoreHandle_t telemetry_mutex_ = nullptr;
        
        // WebSocket broadcast task
        TaskHandle_t ws_task_handle_ = nullptr;
        volatile bool ws_task_running_ = false;
        
        // Static instance for HTTP handlers
        static WifiMonitor* instance_;
    };

} // namespace wifi_monitor
