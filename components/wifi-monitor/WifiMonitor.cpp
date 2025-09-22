// Primary implementation file for WifiMonitor. Fixed corrupted content, added
// missing includes and proper namespace scoping.
#include "WifiMonitor.hpp"
#include "SystemMonitor.hpp"
#include "DataLoggerService.hpp" // DataLogger integration
#include "DataLogger.hpp"        // For direct DataLogger method access
#include <Logger.hpp>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cmath>
#include <math.h>

#include <esp_wifi.h>
#include <nvs_flash.h>
#include <apps/dhcpserver/dhcpserver.h> // dhcps_lease_t
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <cJSON.h>

namespace wifi_monitor
{

    // Define static instance pointer
    WifiMonitor *WifiMonitor::instance_ = nullptr;

    esp_err_t WifiMonitor::initialize()
    {
        // Register component with centralized logger (tag: WIFI)
        DIGITOYS_REGISTER_COMPONENT("WifiMonitor", "WIFI");

        // Initialize NVS (required by WiFi)
        esp_err_t nvs_ret = nvs_flash_init();
        if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            DIGITOYS_LOGW("WifiMonitor", "NVS init returned %s, erasing and retrying", esp_err_to_name(nvs_ret));
            ESP_ERROR_CHECK(nvs_flash_erase());
            nvs_ret = nvs_flash_init();
        }
        if (nvs_ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to initialize NVS: %s", esp_err_to_name(nvs_ret));
            return nvs_ret;
        }

        // Create mutexes needed across the component
        telemetry_mutex_ = xSemaphoreCreateMutex();
        if (telemetry_mutex_ == nullptr)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to create telemetry mutex");
            return ESP_ERR_NO_MEM;
        }

        ws_clients_mutex_ = xSemaphoreCreateMutex();
        if (ws_clients_mutex_ == nullptr)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to create WebSocket clients mutex");
            vSemaphoreDelete(telemetry_mutex_);
            telemetry_mutex_ = nullptr;
            return ESP_ERR_NO_MEM;
        }

        diagnostic_mutex_ = xSemaphoreCreateMutex();
        if (diagnostic_mutex_ == nullptr)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to create diagnostic logging mutex");
            vSemaphoreDelete(telemetry_mutex_);
            vSemaphoreDelete(ws_clients_mutex_);
            telemetry_mutex_ = nullptr;
            ws_clients_mutex_ = nullptr;
            return ESP_ERR_NO_MEM;
        }

        system_log_mutex_ = xSemaphoreCreateMutex();
        if (system_log_mutex_ == nullptr)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to create system log mutex");
            vSemaphoreDelete(telemetry_mutex_);
            vSemaphoreDelete(ws_clients_mutex_);
            vSemaphoreDelete(diagnostic_mutex_);
            telemetry_mutex_ = nullptr;
            ws_clients_mutex_ = nullptr;
            diagnostic_mutex_ = nullptr;
            return ESP_ERR_NO_MEM;
        }

        // Initialize telemetry timestamp
        telemetry_data_.timestamp = esp_timer_get_time() / 1000; // ms

        setState(digitoys::core::ComponentState::INITIALIZED);
        DIGITOYS_LOGI("WifiMonitor", "WiFi Monitor component initialized successfully");
        return ESP_OK;
    }

    esp_err_t WifiMonitor::start()
    {
        if (getState() != digitoys::core::ComponentState::INITIALIZED &&
            getState() != digitoys::core::ComponentState::STOPPED)
        {
            DIGITOYS_LOGW("WifiMonitor", "Component not in correct state to start");
            return ESP_ERR_INVALID_STATE;
        }

        DIGITOYS_LOGI("WifiMonitor", "Starting WiFi Monitor component");
        instance_ = this;

        // Setup WiFi Access Point
        esp_err_t ret = setupWifiAP();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to setup WiFi AP: %s", esp_err_to_name(ret));
            setState(digitoys::core::ComponentState::ERROR);
            return ret;
        }

        // Start HTTP server with WebSocket support
        ret = startHttpServer();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to start HTTP server: %s", esp_err_to_name(ret));
            teardownWifi();
            setState(digitoys::core::ComponentState::ERROR);
            return ret;
        }

        // Start WebSocket broadcast task
        ret = startWebSocketTask();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to start WebSocket task: %s", esp_err_to_name(ret));
            stopHttpServer();
            teardownWifi();
            setState(digitoys::core::ComponentState::ERROR);
            return ret;
        }

        setState(digitoys::core::ComponentState::RUNNING);

        // Set up system log capture hook
        esp_log_set_vprintf(logHook);

        DIGITOYS_LOGI("WifiMonitor", "WiFi Monitor component started successfully");
        DIGITOYS_LOGI("WifiMonitor", "Access Point: SSID='%s', IP=%s",
                      digitoys::constants::wifi_monitor::AP_SSID,
                      digitoys::constants::wifi_monitor::AP_IP);
        return ESP_OK;
    }

    esp_err_t WifiMonitor::stop()
    {
        if (getState() != digitoys::core::ComponentState::RUNNING)
        {
            DIGITOYS_LOGW("WifiMonitor", "Component not running, cannot stop");
            return ESP_ERR_INVALID_STATE;
        }

        DIGITOYS_LOGI("WifiMonitor", "Stopping WiFi Monitor component");

        // Stop WebSocket task first
        esp_err_t ret = stopWebSocketTask();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGW("WifiMonitor", "Failed to stop WebSocket task: %s", esp_err_to_name(ret));
        }

        // Stop HTTP server
        ret = stopHttpServer();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGW("WifiMonitor", "Failed to stop HTTP server: %s", esp_err_to_name(ret));
        }

        // Teardown WiFi
        ret = teardownWifi();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGW("WifiMonitor", "Failed to teardown WiFi: %s", esp_err_to_name(ret));
        }

        // Restore original vprintf
        esp_log_set_vprintf(vprintf);

        instance_ = nullptr;
        setState(digitoys::core::ComponentState::STOPPED);
        DIGITOYS_LOGI("WifiMonitor", "WiFi Monitor component stopped");
        return ESP_OK;
    }

    esp_err_t WifiMonitor::shutdown()
    {
        if (getState() == digitoys::core::ComponentState::RUNNING)
        {
            esp_err_t ret = stop();
            if (ret != ESP_OK)
            {
                DIGITOYS_LOGW("WifiMonitor", "Failed to stop during shutdown: %s", esp_err_to_name(ret));
            }
        }

        // Clean up mutexes
        if (telemetry_mutex_)
        {
            vSemaphoreDelete(telemetry_mutex_);
            telemetry_mutex_ = nullptr;
        }

        if (ws_clients_mutex_)
        {
            vSemaphoreDelete(ws_clients_mutex_);
            ws_clients_mutex_ = nullptr;
        }

        if (diagnostic_mutex_)
        {
            vSemaphoreDelete(diagnostic_mutex_);
            diagnostic_mutex_ = nullptr;
        }

        if (system_log_mutex_)
        {
            vSemaphoreDelete(system_log_mutex_);
            system_log_mutex_ = nullptr;
        }

        // Clear WebSocket clients vector
        websocket_clients_.clear();

        // Clear diagnostic log
        diagnostic_log_.clear();
        logging_active_ = false;

        // Clear system log buffer
        system_log_buffer_.clear();

        setState(digitoys::core::ComponentState::UNINITIALIZED);
        DIGITOYS_LOGI("WifiMonitor", "WiFi Monitor component shutdown complete");
        return ESP_OK;
    }

    void WifiMonitor::updateTelemetry(bool obstacle, float distance, float speed_est, bool warning)
    {
        if (telemetry_mutex_ && xSemaphoreTake(telemetry_mutex_,
                                               pdMS_TO_TICKS(digitoys::constants::wifi_monitor::HTTP_TELEMETRY_TIMEOUT_MS)))
        {
            telemetry_data_.obstacle = obstacle;
            telemetry_data_.distance = distance;
            telemetry_data_.speed_est = speed_est;
            telemetry_data_.warning = warning;
            telemetry_data_.timestamp = esp_timer_get_time() / 1000; // Convert to milliseconds

            xSemaphoreGive(telemetry_mutex_);
        }
        else
        {
            DIGITOYS_LOGW("WifiMonitor", "Failed to acquire telemetry mutex for update");
        }
    }

    void WifiMonitor::submitTelemetryFrame(const TelemetryFrame &frame)
    {
        uint64_t now_us = frame.ts_us ? frame.ts_us : esp_timer_get_time();
        if (!telemetry_mutex_)
            return;
        if (xSemaphoreTake(telemetry_mutex_, pdMS_TO_TICKS(5)))
        {
            // Move current last to prev
            if (last_frame_valid_)
            {
                prev_frame_ = last_frame_;
                prev_frame_valid_ = true;
            }

            last_frame_ = frame; // copy provided fields
            last_frame_.ts_us = now_us;
            last_frame_.seq = frame_seq_++;

            // --- Exponential smoothing for lidar_filtered_m ---
            constexpr float alpha = 0.25f; // smoothing factor (0.0 = no update, 1.0 = instant)
            if (last_frame_valid_)
            {
                last_frame_.lidar_filtered_m = alpha * last_frame_.lidar_distance_m + (1.0f - alpha) * prev_frame_.lidar_filtered_m;
            }
            else
            {
                last_frame_.lidar_filtered_m = last_frame_.lidar_distance_m;
            }

            // Derive speed if we have previous frame and time advanced (use filtered value)
            if (prev_frame_valid_)
            {
                int64_t dt_us = (int64_t)last_frame_.ts_us - (int64_t)prev_frame_.ts_us;
                float dd = prev_frame_.lidar_filtered_m - last_frame_.lidar_filtered_m; // positive if moving forward toward obstacle
                if (dt_us > 0 && dd > 0.0f)
                {
                    last_frame_.speed_approx_mps = dd / (dt_us / 1e6f);
                }
                else
                {
                    last_frame_.speed_approx_mps = 0.0f;
                }
            }
            else
            {
                last_frame_.speed_approx_mps = 0.0f;
            }

            // Safety margin (distance - brake_distance)
            last_frame_.safety_margin_m = last_frame_.lidar_distance_m - last_frame_.brake_distance_m;

            last_frame_valid_ = true;

            // Unconditionally stream CSV row to dedicated CSV clients (header already sent)
            {
                // Debug (rate-limited) to confirm path is active and client counts
                static uint32_t dbg_count = 0;
                if ((dbg_count++ % 50) == 0)
                {
                    size_t data_clients = 0;
                    size_t csv_clients = 0;
                    if (xSemaphoreTake(ws_clients_mutex_, 0) == pdTRUE)
                    {
                        data_clients = websocket_data_clients_.size();
                        csv_clients = websocket_csv_clients_.size();
                        xSemaphoreGive(ws_clients_mutex_);
                    }
                    DIGITOYS_LOGD("WifiMonitor", "CSVstream active: seq=%u, data_clients=%u, csv_clients=%u",
                                  (unsigned)last_frame_.seq, (unsigned)data_clients, (unsigned)csv_clients);
                }
                char csv_row[256];
                int len = snprintf(csv_row, sizeof(csv_row), "%u,%llu,%.6f,%d,%d,%d,%.4f,%.4f,%d,%d,%.4f,%.4f,%.4f,%.6f",
                                   (unsigned)last_frame_.seq,
                                   (unsigned long long)last_frame_.ts_us,
                                   last_frame_.rc_duty_raw,
                                   last_frame_.rc_throttle_pressed ? 1 : 0,
                                   last_frame_.rc_forward ? 1 : 0,
                                   last_frame_.rc_reverse ? 1 : 0,
                                   last_frame_.lidar_distance_m,
                                   last_frame_.lidar_filtered_m,
                                   last_frame_.obstacle_detected ? 1 : 0,
                                   last_frame_.warning_active ? 1 : 0,
                                   last_frame_.brake_distance_m,
                                   last_frame_.warning_distance_m,
                                   last_frame_.safety_margin_m,
                                   last_frame_.speed_approx_mps);
                if (len > 0 && len < (int)sizeof(csv_row))
                {
                    if (xSemaphoreTake(ws_clients_mutex_, pdMS_TO_TICKS(5)) == pdTRUE)
                    {
                        // Send to CSV clients first (unconditional)
                        for (auto it = websocket_csv_clients_.begin(); it != websocket_csv_clients_.end();)
                        {
                            httpd_ws_frame_t ws_pkt = {};
                            ws_pkt.payload = (uint8_t *)csv_row;
                            ws_pkt.len = len;
                            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
                            int sockfd = *it;
                            if (httpd_ws_send_frame_async(server_, sockfd, &ws_pkt) != ESP_OK)
                            {
                                DIGITOYS_LOGW("WifiMonitor", "CSVstream: removing disconnected CSV client %d", sockfd);
                                it = websocket_csv_clients_.erase(it);
                            }
                            else
                            {
                                ++it;
                            }
                        }
                        // Optionally also send to data clients, but only when logging is active (compat)
                        if (logging_active_)
                        {
                            for (auto it = websocket_data_clients_.begin(); it != websocket_data_clients_.end();)
                            {
                                httpd_ws_frame_t ws_pkt = {};
                                ws_pkt.payload = (uint8_t *)csv_row;
                                ws_pkt.len = len;
                                ws_pkt.type = HTTPD_WS_TYPE_TEXT;
                                int sockfd = *it;
                                if (httpd_ws_send_frame_async(server_, sockfd, &ws_pkt) != ESP_OK)
                                {
                                    DIGITOYS_LOGW("WifiMonitor", "CSVstream: removing disconnected data client %d", sockfd);
                                    it = websocket_data_clients_.erase(it);
                                }
                                else
                                {
                                    ++it;
                                }
                            }
                        }
                        xSemaphoreGive(ws_clients_mutex_);
                    }
                }
            }

            // Prepare JSON for WebSocket clients (data channel)
            if (!websocket_data_clients_.empty())
            {
                cJSON *root = cJSON_CreateObject();
                cJSON_AddNumberToObject(root, "ts_us", (double)last_frame_.ts_us);
                cJSON_AddNumberToObject(root, "seq", (double)last_frame_.seq);
                cJSON_AddNumberToObject(root, "rc_duty_raw", last_frame_.rc_duty_raw);
                cJSON_AddBoolToObject(root, "rc_throttle_pressed", last_frame_.rc_throttle_pressed);
                cJSON_AddBoolToObject(root, "rc_forward", last_frame_.rc_forward);
                cJSON_AddBoolToObject(root, "rc_reverse", last_frame_.rc_reverse);
                cJSON_AddNumberToObject(root, "lidar_distance_m", last_frame_.lidar_distance_m);
                cJSON_AddNumberToObject(root, "lidar_filtered_m", last_frame_.lidar_filtered_m);
                cJSON_AddBoolToObject(root, "obstacle_detected", last_frame_.obstacle_detected);
                cJSON_AddBoolToObject(root, "warning_active", last_frame_.warning_active);
                cJSON_AddNumberToObject(root, "brake_distance_m", last_frame_.brake_distance_m);
                cJSON_AddNumberToObject(root, "warning_distance_m", last_frame_.warning_distance_m);
                cJSON_AddNumberToObject(root, "safety_margin_m", last_frame_.safety_margin_m);
                cJSON_AddNumberToObject(root, "speed_approx_mps", last_frame_.speed_approx_mps);

                char *json_str = cJSON_PrintUnformatted(root);
                if (json_str)
                {
                    std::string payload(json_str);
                    cJSON_free(json_str);

                    if (xSemaphoreTake(ws_clients_mutex_, pdMS_TO_TICKS(5)) == pdTRUE)
                    {
                        for (auto it = websocket_data_clients_.begin(); it != websocket_data_clients_.end();)
                        {
                            httpd_ws_frame_t ws_pkt = {};
                            ws_pkt.payload = (uint8_t *)payload.c_str();
                            ws_pkt.len = payload.size();
                            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
                            int sockfd = *it;
                            if (httpd_ws_send_frame_async(server_, sockfd, &ws_pkt) != ESP_OK)
                            {
                                DIGITOYS_LOGW("WifiMonitor", "Removing disconnected data client %d", sockfd);
                                it = websocket_data_clients_.erase(it);
                            }
                            else
                            {
                                ++it;
                            }
                        }
                        xSemaphoreGive(ws_clients_mutex_);
                    }
                }
                cJSON_Delete(root);
            }

            xSemaphoreGive(telemetry_mutex_);
        }
    }

    bool WifiMonitor::getLastTelemetryFrame(TelemetryFrame &out) const
    {
        if (!telemetry_mutex_)
            return false;
        if (xSemaphoreTake(telemetry_mutex_, pdMS_TO_TICKS(5)))
        {
            if (last_frame_valid_)
            {
                out = last_frame_;
                xSemaphoreGive(telemetry_mutex_);
                return true;
            }
            xSemaphoreGive(telemetry_mutex_);
        }
        return false;
    }

    esp_err_t WifiMonitor::getTelemetry(Telemetry &data) const
    {
        if (telemetry_mutex_ && xSemaphoreTake(telemetry_mutex_,
                                               pdMS_TO_TICKS(digitoys::constants::wifi_monitor::HTTP_TELEMETRY_TIMEOUT_MS)))
        {
            data = telemetry_data_;
            xSemaphoreGive(telemetry_mutex_);
            return ESP_OK;
        }

        DIGITOYS_LOGW("WifiMonitor", "Failed to acquire telemetry mutex for read");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t WifiMonitor::setupWifiAP()
    {
        DIGITOYS_LOGI("WifiMonitor", "Setting up WiFi Access Point");

        // Initialize network interface
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        // Create AP network interface
        ap_netif_ = esp_netif_create_default_wifi_ap();
        if (ap_netif_ == nullptr)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to create AP network interface");
            return ESP_FAIL;
        }

        // Initialize WiFi
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

        // Reduce some buffer sizes to save power/memory
        cfg.static_rx_buf_num = 8;   // Reduce from default 10
        cfg.dynamic_rx_buf_num = 16; // Reduce from default 32
        cfg.dynamic_tx_buf_num = 16; // Reduce from default 32
        cfg.rx_mgmt_buf_num = 4;     // Reduce from default 5

        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        // Configure AP settings
        wifi_config_t ap_config = {};
        strcpy((char *)ap_config.ap.ssid, CONFIG_WIFI_MONITOR_AP_SSID);
        strcpy((char *)ap_config.ap.password, CONFIG_WIFI_MONITOR_AP_PASSWORD);
        ap_config.ap.ssid_len = strlen(CONFIG_WIFI_MONITOR_AP_SSID);
        ap_config.ap.max_connection = CONFIG_WIFI_MONITOR_MAX_CONNECTIONS;
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
        ap_config.ap.channel = 1;

        // If password is empty, use open authentication
        if (strlen(CONFIG_WIFI_MONITOR_AP_PASSWORD) == 0)
        {
            ap_config.ap.authmode = WIFI_AUTH_OPEN;
        }

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

        // Configure DHCP
        esp_err_t ret = configureDHCP();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to configure DHCP: %s", esp_err_to_name(ret));
            return ret;
        }

        // Start WiFi
        ESP_ERROR_CHECK(esp_wifi_start());

        // Set WiFi power using Kconfig value to prevent brownout issues
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(CONFIG_WIFI_MONITOR_TX_POWER_DBM * 4)); // Convert dBm to quarter-dBm units
        DIGITOYS_LOGI("WifiMonitor", "WiFi power set to %d dBm to prevent brownout", CONFIG_WIFI_MONITOR_TX_POWER_DBM);

        DIGITOYS_LOGI("WifiMonitor", "WiFi AP started - SSID: %s, IP: %s",
                      CONFIG_WIFI_MONITOR_AP_SSID,
                      digitoys::constants::wifi_monitor::AP_IP);

        return ESP_OK;
    }

    esp_err_t WifiMonitor::configureDHCP()
    {
        DIGITOYS_LOGI("WifiMonitor", "Configuring DHCP server");

        // Stop DHCP server first
        ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif_));

        // Configure static IP for AP
        esp_netif_ip_info_t ip_info;
        ip_info.ip.addr = esp_ip4addr_aton(digitoys::constants::wifi_monitor::AP_IP);
        ip_info.gw.addr = esp_ip4addr_aton(digitoys::constants::wifi_monitor::AP_GATEWAY);
        ip_info.netmask.addr = esp_ip4addr_aton(digitoys::constants::wifi_monitor::AP_NETMASK);
        ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif_, &ip_info));

        // Configure DHCP lease range using Kconfig values
        dhcps_lease_t dhcp_lease;
        dhcp_lease.enable = true;
        dhcp_lease.start_ip.addr = esp_ip4addr_aton(CONFIG_WIFI_MONITOR_DHCP_START_IP);
        dhcp_lease.end_ip.addr = esp_ip4addr_aton(CONFIG_WIFI_MONITOR_DHCP_END_IP);

        ESP_ERROR_CHECK(esp_netif_dhcps_option(ap_netif_, ESP_NETIF_OP_SET,
                                               ESP_NETIF_REQUESTED_IP_ADDRESS,
                                               &dhcp_lease, sizeof(dhcp_lease)));

        // Start DHCP server
        ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif_));

        DIGITOYS_LOGI("WifiMonitor", "DHCP configured - Range: %s to %s",
                      CONFIG_WIFI_MONITOR_DHCP_START_IP,
                      CONFIG_WIFI_MONITOR_DHCP_END_IP);

        return ESP_OK;
    }

    esp_err_t WifiMonitor::teardownWifi()
    {
        DIGITOYS_LOGI("WifiMonitor", "Tearing down WiFi");

        if (ap_netif_)
        {
            esp_wifi_stop();
            esp_wifi_deinit();
            esp_netif_destroy(ap_netif_);
            ap_netif_ = nullptr;
        }

        return ESP_OK;
    }

    // Placeholder implementations for remaining methods
    // These will be implemented in the next steps

    esp_err_t WifiMonitor::startHttpServer()
    {
        DIGITOYS_LOGI("WifiMonitor", "Starting HTTP server");

        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.task_priority = digitoys::constants::wifi_monitor::TASK_PRIORITY;
        config.stack_size = digitoys::constants::wifi_monitor::HTTP_SERVER_STACK_SIZE;
        config.server_port = digitoys::constants::wifi_monitor::HTTP_SERVER_PORT;
        config.max_open_sockets = 7; // Allow for multiple connections
        config.lru_purge_enable = true;

        esp_err_t ret = httpd_start(&server_, &config);
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to start HTTP server: %s", esp_err_to_name(ret));
            return ret;
        }

        // Register URI handlers
        instance_ = this; // Set static instance for handlers

        // Dashboard route (serve system.html)
        httpd_uri_t dashboard_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = indexGetHandler,
            .user_ctx = nullptr};
        httpd_register_uri_handler(server_, &dashboard_uri);

        // System data route for HTTP polling fallback
        httpd_uri_t system_uri = {
            .uri = "/system",
            .method = HTTP_GET,
            .handler = systemGetHandler,
            .user_ctx = nullptr};
        httpd_register_uri_handler(server_, &system_uri);

        // WebSocket route for real-time streaming
        httpd_uri_t ws_uri = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = websocketHandler,
            .user_ctx = nullptr,
            .is_websocket = true,
            .handle_ws_control_frames = true};

        // WebSocket route for data streaming
        httpd_uri_t ws_data_uri = {
            .uri = "/ws/data",
            .method = HTTP_GET,
            .handler = websocketDataHandler,
            .user_ctx = nullptr,
            .is_websocket = true,
            .handle_ws_control_frames = true};
        // WebSocket route for CSV streaming
        httpd_uri_t ws_csv_uri = {
            .uri = "/ws/csv",
            .method = HTTP_GET,
            .handler = websocketCsvHandler,
            .user_ctx = nullptr,
            .is_websocket = true,
            .handle_ws_control_frames = true};
        httpd_register_uri_handler(server_, &ws_uri);
        httpd_register_uri_handler(server_, &ws_data_uri);
        httpd_register_uri_handler(server_, &ws_csv_uri);

        // Logging control endpoint (start/stop/clear logging)
        httpd_uri_t logging_control_uri = {
            .uri = "/logging/control",
            .method = HTTP_POST,
            .handler = loggingControlHandler,
            .user_ctx = nullptr};
        httpd_register_uri_handler(server_, &logging_control_uri);

        // Logging control status endpoint (GET)
        httpd_uri_t logging_status_uri = {
            .uri = "/logging/control",
            .method = HTTP_GET,
            .handler = loggingControlHandler,
            .user_ctx = nullptr};
        httpd_register_uri_handler(server_, &logging_status_uri);

        // Logging data endpoint (get logged data)
        httpd_uri_t logging_data_uri = {
            .uri = "/logging/data",
            .method = HTTP_GET,
            .handler = loggingDataHandler,
            .user_ctx = nullptr};
        httpd_register_uri_handler(server_, &logging_data_uri);

        // Unified telemetry minimal dashboard
        httpd_uri_t unified_uri = {
            .uri = "/unified",
            .method = HTTP_GET,
            .handler = [](httpd_req_t *req) -> esp_err_t
            {
                const char *html =
                    "<!DOCTYPE html><html><head><meta charset='utf-8'/><title>Unified Telemetry</title>"
                    "<style>body{font-family:Arial;margin:12px;background:#111;color:#eee}table{border-collapse:collapse}td,th{border:1px solid #444;padding:4px 8px}#status{margin:8px 0} .ok{color:#4caf50}.warn{color:#ff9800}.bad{color:#f44336}</style>"
                    "</head><body><h2>Unified Telemetry Frame</h2><div id='status'>Connecting...</div>"
                    "<table id='t'><thead><tr><th>Field</th><th>Value</th></tr></thead><tbody></tbody></table>"
                    "<script>const tbody=document.querySelector('#t tbody');const status=document.getElementById('status');"
                    "const fields=['seq','ts_us','rc_duty_raw','rc_throttle_pressed','rc_forward','rc_reverse','lidar_distance_m','lidar_filtered_m','obstacle_detected','warning_active','brake_distance_m','warning_distance_m','safety_margin_m','speed_approx_mps'];"
                    "function ensureRows(){if(tbody.children.length===0){for(const f of fields){const tr=document.createElement('tr');tr.innerHTML='<td>'+f+'</td><td id=val_'+f+'></td>';tbody.appendChild(tr);}}}ensureRows();"
                    "function fmt(v){if(typeof v==='number'){if(Math.abs(v)>1000)return v.toFixed(0);if(Math.abs(v)>10)return v.toFixed(3);return v.toFixed(4);}return v;}"
                    "function update(d){for(const f of fields){const el=document.getElementById('val_'+f);if(el&&f in d)el.textContent=fmt(d[f]);}status.textContent='Live (seq '+d.seq+')';status.className='ok';}"
                    "function connect(){const proto=location.protocol==='https:'?'wss':'ws';const ws=new WebSocket(proto+'://'+location.host+'/ws/data');ws.onopen=()=>{status.textContent='Connected';status.className='ok';};ws.onmessage=e=>{try{update(JSON.parse(e.data));}catch(err){console.error(err);} };ws.onclose=()=>{status.textContent='Reconnecting...';status.className='warn';setTimeout(connect,1000);};ws.onerror=()=>ws.close();}connect();"
                    "</script></body></html>";
                httpd_resp_set_type(req, "text/html");
                httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
                httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
                return ESP_OK; },
            .user_ctx = nullptr};
        httpd_register_uri_handler(server_, &unified_uri);

        DIGITOYS_LOGI("WifiMonitor", "HTTP server started on port %d with WebSocket support", config.server_port);
        return ESP_OK;
    }

    esp_err_t WifiMonitor::stopHttpServer()
    {
        DIGITOYS_LOGI("WifiMonitor", "Stopping HTTP server");

        if (server_)
        {
            esp_err_t ret = httpd_stop(server_);
            server_ = nullptr;

            if (ret != ESP_OK)
            {
                DIGITOYS_LOGE("WifiMonitor", "Failed to stop HTTP server: %s", esp_err_to_name(ret));
                return ret;
            }
        }

        return ESP_OK;
    }

    esp_err_t WifiMonitor::startWebSocketTask()
    {
        DIGITOYS_LOGI("WifiMonitor", "WebSocket task (placeholder - skipping for now)");

        // For now, just mark as successful - we'll implement WebSocket later
        ws_task_running_ = false; // Not actually running yet

        return ESP_OK;
    }

    esp_err_t WifiMonitor::stopWebSocketTask()
    {
        DIGITOYS_LOGI("WifiMonitor", "WebSocket task stop (placeholder)");

        ws_task_running_ = false;

        return ESP_OK;
    }

    // HTTP handlers (placeholders)
    esp_err_t WifiMonitor::telemetryGetHandler(httpd_req_t *req)
    {
        // TODO: Implement telemetry endpoint
        return ESP_OK;
    }

    // Helper function to calculate real CPU load (adapted from SystemMonitor)
    static float calculate_cpu_load(uint32_t &prev_idle, uint32_t &prev_total)
    {
        const int MAX_TASKS = 20; // digitoys::constants::monitor::MAX_MONITORED_TASKS
        TaskStatus_t status[MAX_TASKS];
        uint32_t total_time = 0;
        UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, &total_time);
        uint32_t idle_time = 0;

        for (UBaseType_t i = 0; i < count; ++i)
        {
            if (strstr(status[i].pcTaskName, "IDLE") != nullptr)
            {
                idle_time += status[i].ulRunTimeCounter;
            }
        }

        float load = 0.0f;
        if (prev_total != 0 && total_time > prev_total)
        {
            uint32_t diff_total = total_time - prev_total;
            uint32_t diff_idle = idle_time - prev_idle;

            if (diff_total > 0)
            {
                load = 100.0f * (1.0f - (float)diff_idle / (float)diff_total);
            }
        }
        prev_total = total_time;
        prev_idle = idle_time;
        return load;
    }

    esp_err_t WifiMonitor::systemGetHandler(httpd_req_t *req)
    {
        if (!instance_)
        {
            httpd_resp_send_500(req);
            return ESP_ERR_INVALID_STATE;
        }

        // Set content type to JSON
        httpd_resp_set_type(req, "application/json");

        // Add CORS headers for cross-origin requests
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET");

        // Get real system information
        static uint32_t prev_idle_time = 0;
        static uint32_t prev_total_time = 0;

        // Calculate real CPU usage using FreeRTOS statistics
        float cpu_usage = calculate_cpu_load(prev_idle_time, prev_total_time);

        // Get real heap information
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

        // Get real task information
        const int MAX_TASKS = 20;
        TaskStatus_t status[MAX_TASKS];
        uint32_t total_time = 0;
        UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, &total_time);

        // Create JSON response using cJSON (more robust than snprintf for complex data)
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "cpu", cpu_usage);
        cJSON_AddNumberToObject(root, "total_heap", (double)total_heap);
        cJSON_AddNumberToObject(root, "free_heap", (double)free_heap);

        // Add update counter for debugging
        static uint32_t counter = 0;
        counter++;
        cJSON_AddNumberToObject(root, "counter", counter);

        // Add task information
        cJSON *tasks = cJSON_CreateArray();
        for (UBaseType_t i = 0; i < count && i < 10; ++i) // Limit to first 10 tasks for display
        {
            cJSON *task = cJSON_CreateObject();
            cJSON_AddStringToObject(task, "name", status[i].pcTaskName);
            cJSON_AddNumberToObject(task, "hwm", status[i].usStackHighWaterMark);
            cJSON_AddItemToArray(tasks, task);
        }
        cJSON_AddItemToObject(root, "tasks", tasks);

        // Add telemetry data if available
        Telemetry telemetry;
        if (instance_->getTelemetry(telemetry) == ESP_OK)
        {
            cJSON *telemetry_obj = cJSON_CreateObject();
            cJSON_AddBoolToObject(telemetry_obj, "obstacle", telemetry.obstacle);
            cJSON_AddBoolToObject(telemetry_obj, "warning", telemetry.warning);
            cJSON_AddNumberToObject(telemetry_obj, "distance", (double)telemetry.distance);
            cJSON_AddNumberToObject(telemetry_obj, "speed_est", (double)telemetry.speed_est);
            cJSON_AddNumberToObject(telemetry_obj, "timestamp", (double)telemetry.timestamp);
            cJSON_AddItemToObject(root, "telemetry", telemetry_obj);
        }

        // Convert to string and send response
        char *resp = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (resp)
        {
            esp_err_t ret = httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
            free(resp);
            return ret;
        }
        else
        {
            httpd_resp_send_500(req);
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t WifiMonitor::indexGetHandler(httpd_req_t *req)
    {
        if (!instance_)
        {
            httpd_resp_send_500(req);
            return ESP_ERR_INVALID_STATE;
        }

        // Set content type to HTML
        httpd_resp_set_type(req, "text/html");

        // Embedded dashboard HTML (self-contained with inline Chart.js)
        const char *dashboard_html = R"(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Digitoys System Monitor</title>
    <style>
        :root {
            --bg-color: #0d1117;
            --text-color: #c9d1d9;
            --accent-green: #2ea043;
            --accent-orange: #d29922;
            --accent-red: #f85149;
            --gauge-bg: #161b22;
        }
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 2rem 1rem;
        }
        h1 {
            font-size: 2rem;
            font-weight: 600;
            color: var(--accent-green);
            margin-bottom: 2rem;
        }
        .container {
            background-color: var(--gauge-bg);
            padding: 2rem;
            border-radius: 8px;
            margin-bottom: 1rem;
            width: 100%;
            max-width: 600px;
        }
        .cpu-status {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            margin-top: 1rem;
            font-size: 1rem;
            font-weight: 500;
        }
        .cpu-bar {
            width: 24px;
            height: 18px;
            background-color: var(--accent-green);
            border-top: 4px solid transparent;
            transition: background-color 0.3s;
        }
        .usage-list {
            list-style: none;
            margin-top: 1rem;
        }
        .usage-list li {
            padding: 0.5rem;
            margin: 0.25rem 0;
            border-left: 4px solid var(--accent-green);
            background-color: var(--gauge-bg);
        }
        #simpleChart {
            width: 100%;
            height: 200px;
            background-color: #000;
            border: 1px solid #333;
            position: relative;
            margin: 1rem 0;
        }
        .chart-line {
            stroke: var(--accent-green);
            stroke-width: 2;
            fill: none;
        }
        .status {
            display: flex;
            justify-content: space-between;
            margin: 1rem 0;
            padding: 1rem;
            background-color: var(--gauge-bg);
            border-radius: 4px;
        }
        .btn {
            background-color: var(--accent-green);
            color: white;
            border: none;
            padding: 0.75rem 1.5rem;
            margin: 0.25rem;
            border-radius: 4px;
            cursor: pointer;
            font-size: 0.9rem;
            transition: background-color 0.3s ease;
        }
        .btn:hover {
            opacity: 0.9;
        }
        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        .btn-danger {
            background-color: var(--accent-red);
        }
        .btn-warning {
            background-color: var(--accent-orange);
        }
        .btn-secondary {
            background-color: #6c757d;
        }
        .log-status {
            background-color: #1f2937;
            padding: 0.75rem;
            border-radius: 4px;
            margin-bottom: 1rem;
            font-family: 'Courier New', monospace;
            font-size: 0.9rem;
            border-left: 4px solid var(--accent-green);
        }
        .log-controls {
            margin: 1rem 0;
            text-align: center;
        }
        .log-info {
            margin-top: 1rem;
            padding: 0.5rem;
            background-color: #0f1419;
            border-radius: 4px;
            font-size: 0.85rem;
        }
        .log-info div {
            margin: 0.25rem 0;
        }
        .chart-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1rem;
            margin-bottom: 1rem;
        }
        .chart-container {
            background-color: #0f1419;
            padding: 1rem;
            border-radius: 4px;
            text-align: center;
        }
        .chart-container h3 {
            color: var(--accent-green);
            font-size: 0.9rem;
            margin-bottom: 0.5rem;
        }
        .chart-container canvas {
            width: 100%;
            height: auto;
            background-color: #000;
            border: 1px solid #333;
        }
        .chart-legend {
            display: flex;
            justify-content: center;
            margin-top: 8px;
            gap: 20px;
        }
        .legend-item {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 3px;
            font-size: 12px;
            color: #c9d1d9;
        }
        .legend-label {
            display: flex;
            align-items: center;
            gap: 6px;
        }
        .legend-axis {
            font-size: 10px;
            opacity: 0.8;
            text-align: center;
        }
        .legend-color {
            width: 16px;
            height: 2px;
            border-radius: 1px;
        }
        .physics-summary {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 0.5rem;
            margin-top: 1rem;
        }
        .physics-item {
            background-color: #0f1419;
            padding: 0.5rem;
            border-radius: 4px;
            display: flex;
            justify-content: space-between;
        }
        .physics-item .label {
            color: var(--text-color);
            opacity: 0.8;
        }
        @media (max-width: 768px) {
            .chart-grid, .physics-summary {
                grid-template-columns: 1fr;
            }
        }
        .vehicle-visual {
            text-align: center;
            margin: 1.5rem 0;
            padding: 1rem;
            background-color: #0f1419;
            border-radius: 8px;
            border: 2px solid #333;
        }
        .distance-display {
            font-size: 2rem;
            font-family: 'Courier New', monospace;
            margin-bottom: 0.5rem;
            min-height: 3rem;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .status-text {
            font-size: 1.1rem;
            font-weight: 600;
            color: var(--accent-green);
            margin-bottom: 1rem;
        }
        .status-text.warning {
            color: var(--accent-orange);
        }
        .status-text.danger {
            color: var(--accent-red);
        }
        .telemetry-details {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 1rem;
            margin-top: 1rem;
        }
        .detail-item {
            background-color: #0f1419;
            padding: 0.75rem;
            border-radius: 4px;
            text-align: center;
            border: 1px solid #333;
        }
        .detail-item .label {
            display: block;
            font-size: 0.85rem;
            color: var(--text-color);
            opacity: 0.8;
            margin-bottom: 0.25rem;
        }
        @media (max-width: 768px) {
            .chart-grid, .physics-summary, .telemetry-details {
                grid-template-columns: 1fr;
            }
            .distance-display {
                font-size: 1.5rem;
            }
        }
        .console-container {
            background-color: #0f1419;
            padding: 1rem;
            border-radius: 8px;
            margin: 1rem 0;
            border: 1px solid #333;
            font-family: 'Courier New', monospace;
            font-size: 0.8rem;
            max-height: 400px;
            overflow-y: auto;
        }
        .console-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 0.5rem;
            padding-bottom: 0.5rem;
            border-bottom: 1px solid #333;
        }
        .console-title {
            color: var(--accent-green);
            font-weight: 600;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }
        .console-controls {
            display: flex;
            gap: 0.5rem;
        }
        .console-btn {
            background-color: #333;
            color: #c9d1d9;
            border: none;
            padding: 0.25rem 0.5rem;
            border-radius: 3px;
            cursor: pointer;
            font-size: 0.7rem;
            transition: background-color 0.2s;
        }
        .console-btn:hover {
            background-color: #444;
        }
        .console-btn.active {
            background-color: var(--accent-green);
            color: white;
        }
        .console-output {
            background-color: #000;
            padding: 0.75rem;
            border-radius: 4px;
            min-height: 200px;
            max-height: 300px;
            overflow-y: auto;
            overflow-x: auto;
            border: 1px solid #222;
            white-space: nowrap;
            word-wrap: normal;
            font-family: 'Courier New', monospace;
        }
        .console-line {
            margin: 0.1rem 0;
            line-height: 1.2;
            display: flex;
            gap: 0.5rem;
            align-items: baseline;
        }
        .console-timestamp {
            color: #666;
            flex-shrink: 0;
            min-width: 80px;
        }
        .console-level-I {
            color: #2ea043;
            flex-shrink: 0;
            min-width: 15px;
        }
        .console-level-W {
            color: #d29922;
            flex-shrink: 0;
            min-width: 15px;
        }
        .console-level-E {
            color: #f85149;
            flex-shrink: 0;
            min-width: 15px;
        }
        .console-component {
            color: #58a6ff;
            font-weight: bold;
            flex-shrink: 0;
            min-width: 100px;
        }
        .console-message {
            color: #c9d1d9;
        }
    </style>
</head>
<body>
    <h1>Digitoys System Monitor</h1>
    <div class="container">
        <div class="status">
            <div>Status: <span id="status">Connecting...</span></div>
            <div>Updates: <span id="updateCount">0</span></div>
        </div>
        <svg id="simpleChart" viewBox="0 0 600 200">
            <polyline id="cpuLine" class="chart-line" points=""></polyline>
            <text x="10" y="20" fill="#888" font-size="12">CPU %</text>
            <text x="10" y="190" fill="#888" font-size="12">0</text>
            <text x="10" y="20" fill="#888" font-size="12">100</text>
        </svg>
        <div class="cpu-status">
            <div class="cpu-bar" id="cpuBar"></div>
            <div id="cpuLabel">CPU 0%</div>
        </div>
    </div>
    
    <div class="container">
        <h2 style="color: var(--accent-green); margin-bottom: 1rem;">Vehicle Telemetry</h2>
        <div class="vehicle-visual" id="vehicleVisual">
            <div class="distance-display" id="distanceDisplay">🚗</div>
            <div class="status-text" id="statusText">All Clear</div>
        </div>
        <div class="telemetry-details" id="telemetryDetails">
            <div class="detail-item">
                <span class="label">RC Input:</span>
                <span id="telemetryRcInput">--</span>
            </div>
            <div class="detail-item">
                <span class="label">Speed:</span>
                <span id="telemetrySpeed">-- km/h</span>
            </div>
            <div class="detail-item">
                <span class="label">Safety Margin:</span>
                <span id="telemetrySafety">-- m</span>
            </div>
        </div>
    </div>
    
    <div class="container">
        <div class="console-container">
            <div class="console-header">
                <div class="console-title">System Console</div>
                <div class="console-controls">
                    <button class="console-btn" id="consoleRunBtn">▶️ Start</button>
                    <button class="console-btn active" id="consolePauseBtn">⏸️ Paused</button>
                    <button class="console-btn" id="consoleClearBtn">🗑️ Clear</button>
                </div>
            </div>
            <div class="console-output" id="consoleOutput">
                <div class="console-line">
                    <span class="console-timestamp">[System Ready]</span>
                    <span class="console-message"> WiFi Monitor Console initialized - click ▶️ Start to begin logging...</span>
                </div>
            </div>
        </div>
    </div>
    
    <div class="container">
        <h2 style="color: var(--accent-green); margin-bottom: 1rem;">Data Logging</h2>
        <div class="log-status">
            Status: <span id="logStatus">Stopped</span> | 
            Entries: <span id="logEntries">0</span> | 
            Size: <span id="logSize">0 KB</span>
        </div>
        <div class="log-controls">
            <button class="btn" id="startBtn">Start Logging</button>
            <button class="btn btn-danger" id="stopBtn">Stop Logging</button>
            <button class="btn btn-warning" id="exportBtn">Export Server Data</button>
            <button class="btn btn-secondary" id="clearBtn">Clear Data</button>
        </div>
        <div class="log-info">
            <div>Last entry: <span id="lastEntry">Never</span></div>
            <div>Session: <span id="sessionTime">00:00:00</span></div>
            <div>Memory usage: <span id="memoryUsage">0%</span></div>
            <div style="margin-top: 0.5rem; font-size: 0.8rem; color: #888;">
                💡 Live streaming data auto-saves when logging stops. Export Server Data for additional diagnostics.
            </div>
        </div>
    </div>
    
    <div class="container" id="physicsCharts" style="display: none;">
        <h2 style="color: var(--accent-green); margin-bottom: 1rem;">
            Real-Time Physics Data
            <span id="chartsStatus" style="font-size: 0.7rem; color: #888; margin-left: 1rem;"></span>
        </h2>
        <div class="chart-grid">
            <div class="chart-container">
                <h3>Speed vs Distance</h3>
                <canvas id="speedChart" width="280" height="160"></canvas>
                <div class="chart-legend">
                    <div class="legend-item">
                        <div class="legend-label">
                            <div class="legend-color" style="background-color: #2ea043;"></div>
                            <span>Speed</span>
                        </div>
                        <div class="legend-axis">(left axis, km/h)</div>
                    </div>
                    <div class="legend-item">
                        <div class="legend-label">
                            <div class="legend-color" style="background-color: #d29922;"></div>
                            <span>Distance</span>
                        </div>
                        <div class="legend-axis">(right axis, cm)</div>
                    </div>
                </div>
            </div>
            <div class="chart-container">
                <h3>RC Input vs Safety Margin</h3>
                <canvas id="safetyChart" width="280" height="160"></canvas>
                <div class="chart-legend">
                    <div class="legend-item">
                        <div class="legend-label">
                            <div class="legend-color" style="background-color: #2ea043;"></div>
                            <span>RC Input</span>
                        </div>
                        <div class="legend-axis">(left axis, %)</div>
                    </div>
                    <div class="legend-item">
                        <div class="legend-label">
                            <div class="legend-color" style="background-color: #f85149;"></div>
                            <span>Safety Margin</span>
                        </div>
                        <div class="legend-axis">(right axis, cm)</div>
                    </div>
                </div>
            </div>
        </div>
        <div class="physics-summary">
            <div class="physics-item">
                <span class="label">RC Input:</span>
                <span id="currentRcInput">0</span>
            </div>
            <div class="physics-item">
                <span class="label">Distance:</span>
                <span id="currentDistance">0 cm</span>
            </div>
            <div class="physics-item">
                <span class="label">Safety Margin:</span>
                <span id="currentSafety">0 cm</span>
            </div>
            <div class="physics-item">
                <span class="label">Brake Events:</span>
                <span id="brakeEvents">0</span>
            </div>
        </div>
    </div>
    
    <ul class="usage-list" id="metrics"></ul>

    <script>
        let updateCounter = 0;
        let cpuHistory = [];
        const MAX_POINTS = 50;

        function updateSimpleChart(cpuValue) {
            cpuHistory.push(cpuValue);
            if (cpuHistory.length > MAX_POINTS) {
                cpuHistory.shift();
            }

            const svg = document.getElementById('simpleChart');
            const line = document.getElementById('cpuLine');
            
            let points = '';
            cpuHistory.forEach((value, index) => {
                const x = (index / (MAX_POINTS - 1)) * 580 + 10; // 10px margin
                const y = 180 - (value / 100) * 160; // Invert Y, 20px margins
                points += `${x},${y} `;
            });
            
            line.setAttribute('points', points.trim());
        }

        function updateTelemetryDisplay(telemetryData) {
            const distanceDisplay = document.getElementById('distanceDisplay');
            const statusText = document.getElementById('statusText');
            const rcInput = document.getElementById('telemetryRcInput');
            const speed = document.getElementById('telemetrySpeed');
            const safety = document.getElementById('telemetrySafety');
            
            if (!telemetryData) {
                distanceDisplay.textContent = '🚗';
                statusText.textContent = 'No Data';
                statusText.className = 'status-text';
                rcInput.textContent = '--';
                speed.textContent = '-- km/h';
                safety.textContent = '-- m';
                return;
            }
            
            // Update detail values
            rcInput.textContent = telemetryData.rc_input ? `${telemetryData.rc_input.toFixed(1)}%` : '--';
            speed.textContent = `${telemetryData.speed_est.toFixed(1)} km/h`;
            safety.textContent = `${telemetryData.distance.toFixed(2)} m`;
            
            // Determine visual state based on obstacle and warning
            if (telemetryData.obstacle) {
                // CRITICAL - Car with braking
                distanceDisplay.textContent = `🚧 ←--${telemetryData.distance.toFixed(1)}m--> 🛑🚗`;
                statusText.textContent = 'EMERGENCY BRAKE';
                statusText.className = 'status-text danger';
            } else if (telemetryData.warning) {
                // WARNING - Car with distance
                distanceDisplay.textContent = `🚧 ←--${telemetryData.distance.toFixed(1)}m--> 🚗`;
                statusText.textContent = 'WARNING';
                statusText.className = 'status-text warning';
            } else {
                // SAFE - Just the car
                distanceDisplay.textContent = '🚗';
                statusText.textContent = 'All Clear';
                statusText.className = 'status-text';
            }
        }

        // WebSocket-first real-time monitoring with HTTP fallback
        // updateCounter already declared above
        let wsConnected = false;
        let ws = null;
        let reconnectInterval = null;
        let httpFallbackInterval = null;

        // Console functionality
        let consoleRunning = false; // Start paused by default
        let consoleLines = [];
        const MAX_CONSOLE_LINES = 100;

    // File streaming variables
        let dataStreamWs = null;
        let currentLogFile = null;
        let logFileWriter = null;
        let logFileName = '';
        let streamingActive = false;
        
        // Wide CSV (5 Hz) streaming aggregator (client-side, no device storage)
        const WIDE_RATE_HZ = 5;                 // default target
        const WIDE_PERIOD_MS = 1000 / WIDE_RATE_HZ; // 200 ms
        let wideState = null;      // populated from incoming entries
        let wideTimerId = null;    // setInterval id
        let wallStartMs = null;    // fallback base time when ESP timestamps absent
        let lastEventTsUs = null;  // last ESP timestamp observed
        
        // Streaming statistics tracking
        let streamingStartTime = null;
        let streamingEntryCount = 0;
        let streamingRowCount = 0; // Track completed CSV rows (complete datasets)
        let lastStreamingUpdate = null;

        function connectWebSocket() {
            const wsUrl = `ws://${window.location.host}/ws`;
            console.log('Connecting to WebSocket:', wsUrl);
            
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function(event) {
                console.log('WebSocket connected');
                wsConnected = true;
                document.getElementById('status').textContent = 'Connected (WebSocket)';
                document.getElementById('status').style.color = 'var(--accent-green)';
                
                // Clear reconnect timer
                if (reconnectInterval) {
                    clearInterval(reconnectInterval);
                    reconnectInterval = null;
                }
                
                // Stop HTTP fallback
                if (httpFallbackInterval) {
                    clearInterval(httpFallbackInterval);
                    httpFallbackInterval = null;
                }
                
                // Request initial data
                ws.send('get_data');
            };
            
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    console.log('WebSocket data:', data);
                    
                    updateCounter++;
                    document.getElementById('updateCount').textContent = updateCounter;
                    
                    // Update chart
                    updateSimpleChart(data.cpu);
                    
                    // Update CPU display
                    const cpuLabel = document.getElementById('cpuLabel');
                    const cpuBar = document.getElementById('cpuBar');
                    cpuLabel.textContent = `CPU ${data.cpu.toFixed(1)}%`;

                    if (data.cpu >= 70) {
                        cpuBar.style.backgroundColor = 'var(--accent-red)';
                    } else if (data.cpu >= 50) {
                        cpuBar.style.backgroundColor = 'var(--accent-orange)';
                    } else {
                        cpuBar.style.backgroundColor = 'var(--accent-green)';
                    }

                    // Update metrics
                    const list = document.getElementById('metrics');
                    list.innerHTML = '';
                    
                    const counterLi = document.createElement('li');
                    counterLi.textContent = `WS Counter: ${data.counter || 'N/A'}`;
                    list.appendChild(counterLi);
                    
                    const heapLi = document.createElement('li');
                    heapLi.textContent = `Free Heap: ${data.free_heap} bytes`;
                    list.appendChild(heapLi);
                    
                    // Update telemetry display
                    updateTelemetryDisplay(data.telemetry);
                    
                    // Update physics charts with real telemetry data
                    if (document.getElementById('physicsCharts').style.display === 'block') {
                        updatePhysicsDisplay(data.telemetry);
                    }
                    
                    // Update console with system logs
                    if (data.logs && Array.isArray(data.logs)) {
                        data.logs.forEach(logLine => addConsoleLog(logLine));
                    }
                    
                    const typeLi = document.createElement('li');
                    typeLi.textContent = `Transport: ${data.type || 'WebSocket'}`;
                    list.appendChild(typeLi);
                    
                    if (data.tasks) {
                        data.tasks.forEach(task => {
                            const li = document.createElement('li');
                            li.textContent = `${task.name}: Stack HWM ${task.hwm}`;
                            list.appendChild(li);
                        });
                    }
                    
                    // Request next data after 500ms for faster updates
                    setTimeout(() => {
                        if (ws && ws.readyState === WebSocket.OPEN) {
                            ws.send('get_data');
                        }
                    }, 500);
                    
                } catch (e) {
                    console.error('Error parsing WebSocket data:', e);
                }
            };
            
            ws.onclose = function(event) {
                console.log('WebSocket closed:', event.code, event.reason);
                wsConnected = false;
                document.getElementById('status').textContent = 'Disconnected - Reconnecting...';
                document.getElementById('status').style.color = 'var(--accent-orange)';
                
                // Start reconnection timer (faster reconnection)
                if (!reconnectInterval) {
                    reconnectInterval = setInterval(() => {
                        console.log('Attempting WebSocket reconnection...');
                        connectWebSocket();
                    }, 2000); // Faster reconnection (was 3000ms)
                }
                
                // Start HTTP fallback after 3 seconds (faster fallback)
                setTimeout(() => {
                    if (!wsConnected && !httpFallbackInterval) {
                        console.log('Starting HTTP fallback...');
                        startHttpFallback();
                    }
                }, 3000); // Faster fallback (was 5000ms)
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
                wsConnected = false;
                document.getElementById('status').textContent = 'WebSocket Error';
                document.getElementById('status').style.color = 'var(--accent-red)';
            };
        }

        // HTTP fallback function
        async function fetchStatsHTTP() {
            try {
                const resp = await fetch('/system', { cache: 'no-cache' });
                if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
                
                const data = await resp.json();
                
                updateCounter++;
                document.getElementById('updateCount').textContent = updateCounter;
                document.getElementById('status').textContent = 'Connected (HTTP fallback)';
                document.getElementById('status').style.color = 'var(--accent-orange)';
                
                // Update display (same logic as WebSocket)
                updateSimpleChart(data.cpu);
                
                const cpuLabel = document.getElementById('cpuLabel');
                const cpuBar = document.getElementById('cpuBar');
                cpuLabel.textContent = `CPU ${data.cpu.toFixed(1)}%`;

                if (data.cpu >= 70) {
                    cpuBar.style.backgroundColor = 'var(--accent-red)';
                } else if (data.cpu >= 50) {
                    cpuBar.style.backgroundColor = 'var(--accent-orange)';
                } else {
                    cpuBar.style.backgroundColor = 'var(--accent-green)';
                }

                const list = document.getElementById('metrics');
                list.innerHTML = '';
                
                const counterLi = document.createElement('li');
                counterLi.textContent = `HTTP Counter: ${data.counter || 'N/A'}`;
                list.appendChild(counterLi);
                
                const heapLi = document.createElement('li');
                heapLi.textContent = `Free Heap: ${data.free_heap} bytes`;
                list.appendChild(heapLi);
                
                // Update telemetry display
                updateTelemetryDisplay(data.telemetry);
                
                // Update physics charts with real telemetry data
                if (document.getElementById('physicsCharts').style.display === 'block') {
                    updatePhysicsDisplay(data.telemetry);
                }
                
                // Update console with system logs
                if (data.logs && Array.isArray(data.logs)) {
                    data.logs.forEach(logLine => addConsoleLog(logLine));
                }
                
                const typeLi = document.createElement('li');
                typeLi.textContent = `Transport: HTTP fallback`;
                list.appendChild(typeLi);
                
                if (data.tasks) {
                    data.tasks.forEach(task => {
                        const li = document.createElement('li');
                        li.textContent = `${task.name}: Stack HWM ${task.hwm}`;
                        list.appendChild(li);
                    });
                }
                
            } catch (e) {
                console.error('HTTP fallback error:', e);
                document.getElementById('status').textContent = 'All connections failed';
                document.getElementById('status').style.color = 'var(--accent-red)';
            }
        }

        function startHttpFallback() {
            httpFallbackInterval = setInterval(fetchStatsHTTP, 1000); // Faster HTTP polling (was 2000ms)
            fetchStatsHTTP(); // Initial call
        }

        // Start with WebSocket
        connectWebSocket();
        
        // Console functions
        function addConsoleLog(logLine) {
            if (!consoleRunning) return;
            
            const now = new Date();
            const timestamp = now.toTimeString().split(' ')[0];
            
            // Parse ESP-IDF log format: I (timestamp) COMPONENT: message
            let parsedLog = parseESPLog(logLine);
            if (!parsedLog) {
                // Fallback for non-standard format
                parsedLog = {
                    level: 'I',
                    timestamp: timestamp,
                    component: 'SYSTEM',
                    message: logLine
                };
            }
            
            consoleLines.push(parsedLog);
            if (consoleLines.length > MAX_CONSOLE_LINES) {
                consoleLines.shift();
            }
            
            updateConsoleDisplay();
        }
        
        function parseESPLog(logLine) {
            // Parse ESP-IDF format: I (281223) CONTROL: [logDiagnostics] DUTY_TEST: ...
            const match = logLine.match(/^([IWED])\s*\((\d+)\)\s*([^:]+):\s*(.+)$/);
            if (match) {
                return {
                    level: match[1],
                    timestamp: `(${match[2]})`,
                    component: match[3].trim(),
                    message: match[4]
                };
            }
            return null;
        }
        
        function updateConsoleDisplay() {
            const output = document.getElementById('consoleOutput');
            if (!output) return;
            
            // Keep scroll position if user hasn't scrolled up
            const shouldAutoScroll = output.scrollTop >= output.scrollHeight - output.clientHeight - 20;
            
            output.innerHTML = '';
            
            consoleLines.forEach(log => {
                const lineDiv = document.createElement('div');
                lineDiv.className = 'console-line';
                
                const timestampSpan = document.createElement('span');
                timestampSpan.className = 'console-timestamp';
                timestampSpan.textContent = log.timestamp;
                
                const levelSpan = document.createElement('span');
                levelSpan.className = `console-level-${log.level}`;
                levelSpan.textContent = log.level;
                
                const componentSpan = document.createElement('span');
                componentSpan.className = 'console-component';
                componentSpan.textContent = log.component + ':';
                
                const messageSpan = document.createElement('span');
                messageSpan.className = 'console-message';
                messageSpan.textContent = log.message;
                
                lineDiv.appendChild(timestampSpan);
                lineDiv.appendChild(levelSpan);
                lineDiv.appendChild(componentSpan);
                lineDiv.appendChild(messageSpan);
                
                output.appendChild(lineDiv);
            });
            
            // Auto-scroll to bottom if user was at bottom
            if (shouldAutoScroll) {
                output.scrollTop = output.scrollHeight;
            }
        }
        
        function toggleConsole() {
            consoleRunning = !consoleRunning;
            const runBtn = document.getElementById('consoleRunBtn');
            const pauseBtn = document.getElementById('consolePauseBtn');
            
            if (consoleRunning) {
                runBtn.classList.add('active');
                runBtn.textContent = '⏸️ Running';
                pauseBtn.classList.remove('active');
                pauseBtn.textContent = '⏸️ Pause';
            } else {
                runBtn.classList.remove('active');
                runBtn.textContent = '▶️ Start';
                pauseBtn.classList.add('active');
                pauseBtn.textContent = '⏸️ Paused';
            }
        }
        
        function clearConsole() {
            consoleLines = [];
            updateConsoleDisplay();
            // Add system message
            addConsoleLog('I (0) SYSTEM: Console cleared by user');
        }
        
        // File streaming functions for data logging
        function connectDataStream() {
            if (dataStreamWs && dataStreamWs.readyState === WebSocket.OPEN) {
                return; // Already connected
            }
            
            const wsUrl = `ws://${window.location.host}/ws/data`;
            console.log('Connecting to data stream WebSocket:', wsUrl);
            
            dataStreamWs = new WebSocket(wsUrl);
            
            dataStreamWs.onopen = function(event) {
                console.log('Data stream WebSocket connected');
            };
            
            dataStreamWs.onmessage = function(event) {
                if (!(streamingActive && logFileWriter)) return;
                try {
                    const e = JSON.parse(event.data);
                    streamingEntryCount++;
                    if (!streamingStartTime) streamingStartTime = Date.now();

                    if (!wideState) {
                        wideState = {
                            base_ts_us: e.timestamp_us || 0,
                            rc_input_pct: null,
                            dist_m: null,
                            speed_kmh: null,
                            brake_m: null,
                            safety_m: null,
                            obstacle: 0,
                            warning: 0,
                            driving_fwd: 0,
                            wants_rev: 0,
                            throttle: 0,
                            tti_s: null,
                            decel_mps2: null,
                            brake_events: 0,
                            warning_events: 0,
                            source: ''
                        };
                        wallStartMs = Date.now();
                        if (!wideTimerId) wideTimerId = setInterval(writeWideRow, WIDE_PERIOD_MS);
                    }

                    lastEventTsUs = e.timestamp_us || lastEventTsUs;
                    if (e.source) wideState.source = e.source;

                    const key = e.key;
                    const valStr = (e.value ?? '').toString();
                    const toF = (s) => { const v = parseFloat(s); return Number.isFinite(v) ? v : null; };
                    const isTrue = (s) => s === '1' || s === 'true' || s === 1 || s === true;

                    if (key === 'rc_input') {
                        const v = toF(valStr); if (v !== null) wideState.rc_input_pct = v * 100.0;
                    } else if (key === 'obstacle_distance') {
                        const v = toF(valStr); if (v !== null) wideState.dist_m = (v > 10.0 ? v/100.0 : v);
                    } else if (key === 'calculated_speed' || key === 'speed_estimate') {
                        const v = toF(valStr); if (v !== null) wideState.speed_kmh = v * 3.6;
                    } else if (key === 'brake_distance') {
                        const v = toF(valStr); if (v !== null) wideState.brake_m = (v > 10.0 ? v/100.0 : v);
                    } else if (key === 'safety_margin') {
                        const v = toF(valStr); if (v !== null) wideState.safety_m = (v > 10.0 ? v/100.0 : v);
                    } else if (key === 'is_obstacle_state' || key === 'obstacle_detected') {
                        wideState.obstacle = isTrue(valStr) ? 1 : 0;
                    } else if (key === 'is_warning_state' || key === 'warning_active') {
                        wideState.warning = isTrue(valStr) ? 1 : 0;
                    } else if (key === 'driving_forward') {
                        wideState.driving_fwd = isTrue(valStr) ? 1 : 0;
                    } else if (key === 'wants_reverse') {
                        wideState.wants_rev = isTrue(valStr) ? 1 : 0;
                    } else if (key === 'throttle_pressed') {
                        wideState.throttle = isTrue(valStr) ? 1 : 0;
                    } else if (key === 'time_to_impact') {
                        const v = toF(valStr); if (v !== null) wideState.tti_s = v;
                    } else if (key === 'deceleration') {
                        const v = toF(valStr); if (v !== null) wideState.decel_mps2 = v;
                    } else if (key === 'brake_events') {
                        const v = parseInt(valStr, 10); if (!Number.isNaN(v)) wideState.brake_events = v;
                    } else if (key === 'warning_events') {
                        const v = parseInt(valStr, 10); if (!Number.isNaN(v)) wideState.warning_events = v;
                    }
                } catch (error) {
                    console.error('Error processing data:', error);
                }
            };
            
            dataStreamWs.onclose = function(event) {
                console.log('Data stream WebSocket disconnected');
                dataStreamWs = null;
            };
            
            dataStreamWs.onerror = function(error) {
                console.error('Data stream WebSocket error:', error);
            };
        }
        
        function startFileStreaming() {
            if (streamingActive) return;
            
            // Generate filename with timestamp
            const now = new Date();
            const timestamp = now.toISOString().replace(/[:.]/g, '-').slice(0, -5);
            logFileName = `digitoys-log-${timestamp}.csv`;
            
            // Create file stream using File System Access API (if available)
            if ('showSaveFilePicker' in window) {
                // Modern browsers with File System Access API
                createFileWithPicker();
            } else {
                // Fallback: accumulate data in memory and download at the end
                createFileWithFallback();
            }
        }
        
    async function createFileWithPicker() {
            try {
                const fileHandle = await window.showSaveFilePicker({
                    suggestedName: logFileName,
                    types: [{
                        description: 'CSV files',
                        accept: { 'text/csv': ['.csv'] }
                    }]
                });
                
                const writable = await fileHandle.createWritable();
                logFileWriter = writable;
                
                // Write wide CSV header (client-side streaming)
                const header = 'timestamp_us,rc_input_pct,obstacle_distance_m,speed_est_kmh,brake_distance_m,safety_margin_m,is_obstacle,is_warning,driving_forward,driving_backward,wants_reverse,throttle_pressed,time_to_impact_s,deceleration_mps2,brake_events,warning_events,source\n';
                await logFileWriter.write(header);
                
                // Reset streaming statistics
                streamingStartTime = null;
                streamingEntryCount = 0;
                streamingRowCount = 0;
                // no pending row buffer in wide mode
                
                streamingActive = true;
                connectDataStream();
                console.log('File streaming started with picker:', logFileName);
                
            } catch (error) {
                console.error('Error creating file with picker:', error);
                createFileWithFallback(); // Fallback to memory accumulation
            }
        }
        
    function createFileWithFallback() {
            // Fallback: accumulate data in memory
            currentLogFile = [];
            
            // Reset streaming statistics
            streamingStartTime = null;
            streamingEntryCount = 0;
            streamingRowCount = 0;
            // no pending row buffer in wide mode
            
            streamingActive = true;
            connectDataStream();
            console.log('File streaming started with fallback (memory accumulation)');
            
            // Create a simple file writer that accumulates in memory
            logFileWriter = {
                write: function(data) {
                    currentLogFile.push(data);
                },
                close: function() {
                    return Promise.resolve();
                }
            };
            
            // Write wide CSV header (client-side streaming)
            const header = 'timestamp_us,rc_input_pct,obstacle_distance_m,speed_est_kmh,brake_distance_m,safety_margin_m,is_obstacle,is_warning,driving_forward,driving_backward,wants_reverse,throttle_pressed,time_to_impact_s,deceleration_mps2,brake_events,warning_events,source\n';
            logFileWriter.write(header);
        }
        
        async function stopFileStreaming() {
            if (!streamingActive) return;
            
            streamingActive = false;
            
            // No aggregation used; nothing to flush
            if (wideTimerId) {
                clearInterval(wideTimerId);
                wideTimerId = null;
            }
            wideState = null;
            wallStartMs = null;
            lastEventTsUs = null;
            
            // Close WebSocket connection
            if (dataStreamWs) {
                dataStreamWs.close();
                dataStreamWs = null;
            }
            
            // Close file writer
            if (logFileWriter && logFileWriter.close) {
                await logFileWriter.close();
            }
            
            // If using fallback (memory accumulation), trigger download
            if (currentLogFile && currentLogFile.length > 0) {
                downloadLogFile();
            }
            
            console.log('File streaming stopped');
        }
        
        function downloadLogFile() {
            if (!currentLogFile || currentLogFile.length === 0) return;
            
            const csvContent = currentLogFile.join('');
            const blob = new Blob([csvContent], { type: 'text/csv' });
            const url = URL.createObjectURL(blob);
            
            const a = document.createElement('a');
            a.href = url;
            a.download = logFileName;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            URL.revokeObjectURL(url);
            
            currentLogFile = null;
            console.log('Log file downloaded:', logFileName);
        }
        
        function convertToCSV(data) {
            // Convert JSON data to CSV format for generic DataEntry
            const source = (data.source || '').toString();
            const key = (data.key || '').toString();
            const value = (data.value !== undefined && data.value !== null) ? data.value.toString() : '';
            const timestamp_us = data.timestamp_us || 0;
            const type = data.type || 0;
            
            // Escape commas and quotes in values and wrap fields that may contain separators
            const esc = (s) => s.replace(/"/g, '""');
            const wrap = (s) => (s.includes(',') || s.includes('"') || s.includes('\n')) ? `"${esc(s)}"` : s;
            
            return `${timestamp_us},${wrap(source)},${wrap(key)},${wrap(value)},${type}`;
        }
        
        // Aggregation no longer used
        function flushPendingRows() { /* no-op */ }
        
        function convertAggregatedToCSV(rowData) {
            // Convert aggregated data to CSV format matching new header structure
            const timestamp = rowData.timestamp_us || 0;
            const rcInput = rowData.entries.rc_input || rowData.entries['rc_input_duty_cycle'] || 0;
            const distance = rowData.entries.distance || rowData.entries['obstacle_distance'] || 0;
            const speed = rowData.entries.speed || rowData.entries['speed_estimate'] || 0;
            const safetyMargin = rowData.entries.safety_margin || 0;
            const obstacleDetected = (rowData.entries.obstacle_detected === 'true' || 
                                     rowData.entries.is_obstacle_state === 'true' ||
                                     rowData.entries.obstacle_detected === '1') ? 1 : 0;
            const warningActive = (rowData.entries.warning_active === 'true' || 
                                  rowData.entries.warning_active === '1') ? 1 : 0;
            
            return `${timestamp},${rcInput},${distance},${speed},${safetyMargin},${obstacleDetected},${warningActive}`;
        }

        // Periodic writer for wide CSV rows at fixed rate (client-side streaming)
        function writeWideRow() {
            if (!(streamingActive && logFileWriter && wideState)) return;

            const nowMs = Date.now();
            const relUs = wallStartMs ? Math.max(0, Math.floor((nowMs - wallStartMs) * 1000))
                                      : (lastEventTsUs && wideState.base_ts_us ? Math.max(0, lastEventTsUs - wideState.base_ts_us) : 0);

            const driving_bwd = (wideState.wants_rev && !wideState.driving_fwd) ? 1 : 0;

            const num = (v) => (v === null || v === undefined || Number.isNaN(v)) ? '' : String(v);
            const intn = (v) => (v === null || v === undefined || Number.isNaN(v)) ? '0' : String(v|0);
            const esc = (s) => String(s).replace(/"/g, '""');
            const wrap = (s) => {
                const str = (s === null || s === undefined) ? '' : String(s);
                return (str.includes(',') || str.includes('"') || str.includes('\n')) ? `"${esc(str)}"` : str;
            };

            const line = `${relUs},${num(wideState.rc_input_pct)},${num(wideState.dist_m)},${num(wideState.speed_kmh)},`+
                         `${num(wideState.brake_m)},${num(wideState.safety_m)},${intn(wideState.obstacle)},${intn(wideState.warning)},`+
                         `${intn(wideState.driving_fwd)},${intn(driving_bwd)},${intn(wideState.wants_rev)},${intn(wideState.throttle)},`+
                         `${num(wideState.tti_s)},${num(wideState.decel_mps2)},${intn(wideState.brake_events)},${intn(wideState.warning_events)},`+
                         `${wrap(wideState.source)}\n`;

            try {
                logFileWriter.write(line);
                streamingRowCount++;
            } catch (err) {
                console.error('Failed writing wide CSV row:', err);
            }
        }

        function parseCsvRow(line) {
            const parts = line.split(',');
            if (parts.length !== 14) return null;
            const [seq, ts_us, rc_duty_raw, rc_throttle_pressed, rc_forward, rc_reverse,
                   lidar_distance_m, lidar_filtered_m, obstacle_detected, warning_active,
                   brake_distance_m, warning_distance_m, safety_margin_m, speed_approx_mps] = parts;
            const toNum = (v) => { const n = parseFloat(v); return Number.isFinite(n) ? n : 0; };
            return {
                seq: parseInt(seq, 10) || 0,
                ts_us: parseInt(ts_us, 10) || 0,
                rc_input: toNum(rc_duty_raw),
                throttle: rc_throttle_pressed === '1',
                fwd: rc_forward === '1',
                rev: rc_reverse === '1',
                distance: toNum(lidar_distance_m),
                distance_f: toNum(lidar_filtered_m),
                obstacle: obstacle_detected === '1',
                warning: warning_active === '1',
                brake_m: toNum(brake_distance_m),
                warn_m: toNum(warning_distance_m),
                safety_m: toNum(safety_margin_m),
                speed_mps: toNum(speed_approx_mps)
            };
        }

        function onCsvData(f) {
            // Update physics display from CSV-derived values
            const tele = {
                rc_input: f.rc_input, // % duty
                distance: f.distance, // m
                speed_est: f.speed_mps * 3.6, // km/h
                obstacle: f.obstacle,
                warning: f.warning
            };
            updateTelemetryDisplay(tele);
            if (document.getElementById('physicsCharts').style.display === 'block') {
                updatePhysicsDisplay(tele);
            }
        }

    // --- CSV live logger via dedicated WS (/ws/csv) ---
    let csvWS = null;
    let csvRows = [];
    let csvGotHeader = false;
    let csvBytes = 0; // running size estimate for UI
    const CSV_HEADER_STR = 'seq,ts_us,rc_duty_raw,rc_throttle_pressed,rc_forward,rc_reverse,lidar_distance_m,lidar_filtered_m,obstacle_detected,warning_active,brake_distance_m,warning_distance_m,safety_margin_m,speed_approx_mps';

        function startCsvStream() {
            if (csvWS) { try { csvWS.close(); } catch(e){} csvWS = null; }
            csvRows = [];
            csvGotHeader = false;
            csvBytes = 0;
            streamingActive = true;
            streamingStartTime = Date.now();
            streamingRowCount = 0;
            const wsUrl = `ws://${window.location.host}/ws/csv`;
            console.log('CSV logger connecting:', wsUrl);
            csvWS = new WebSocket(wsUrl);
            csvWS.onopen = () => console.log('CSV logger connected');
            csvWS.onmessage = (e) => {
                const text = typeof e.data === 'string' ? e.data : '';
                // Split in case multiple lines arrive in a single frame
                text.split('\n').forEach(line => {
                    const s = line.trim();
                    if (!s) return;
                    if (!csvGotHeader && s.startsWith('seq,')) { csvRows.push(s); csvGotHeader = true; csvBytes += s.length + 1; return; }
                    if (s.split(',').length === 14) {
                        csvRows.push(s);
                        csvBytes += s.length + 1; // + newline
                        streamingRowCount++;
                        const f = parseCsvRow(s);
                        if (f) onCsvData(f);
                    }
                });
            };
            csvWS.onerror = (e) => console.warn('CSV logger error', e);
            csvWS.onclose = () => console.log('CSV logger closed');
        }

        function stopCsvStreamAndDownload() {
            if (csvWS) { try { csvWS.close(); } catch(e){} csvWS = null; }
            streamingActive = false;
            // Ensure header exists for download
            const hasHeader = csvRows.length > 0 && csvRows[0].startsWith('seq,');
            const rows = hasHeader ? csvRows : [CSV_HEADER_STR, ...csvRows];
            const csvContent = rows.join('\n');
            const blob = new Blob([csvContent], { type: 'text/csv' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'digitoys-telemetry-' + new Date().toISOString().replace(/[:.]/g, '-') + '.csv';
            document.body.appendChild(a);
            a.click();
            setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 100);
            console.log('CSV file downloaded, rows:', rows.length);
        }

        function downloadLastCsv() {
            if (!csvRows || csvRows.length === 0) {
                alert('No CSV data available yet. Start logging first.');
                return;
            }
            // Download without changing connection state
            const hasHeader = csvRows[0].startsWith('seq,');
            const rows = hasHeader ? csvRows : [CSV_HEADER_STR, ...csvRows];
            const csvContent = rows.join('\n');
            const blob = new Blob([csvContent], { type: 'text/csv' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'digitoys-telemetry-' + new Date().toISOString().replace(/[:.]/g, '-') + '.csv';
            document.body.appendChild(a);
            a.click();
            setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 100);
        }
        
        // Logging functionality - Phase 2
        var loggingActive = false;
        var logEntries = 0;
        var sessionStartTime = null;
        var sessionTimer = null;
        var dataRefreshTimer = null;
        
        function updateLogStatus(status) {
            document.getElementById('logStatus').textContent = status;
        }
        
        function updateLogEntries(count) {
            logEntries = count;
            document.getElementById('logEntries').textContent = count;
        }
        
        function updateLogSize(sizeKB) {
            document.getElementById('logSize').textContent = sizeKB + ' KB';
        }
        
        function updateLastEntry() {
            var now = new Date();
            var timeStr = now.getHours() + ':' + 
                         (now.getMinutes() < 10 ? '0' : '') + now.getMinutes() + ':' +
                         (now.getSeconds() < 10 ? '0' : '') + now.getSeconds();
            document.getElementById('lastEntry').textContent = timeStr;
        }
        
        function updateSessionTime() {
            if (!sessionStartTime) return;
            
            var now = new Date();
            var diff = Math.floor((now - sessionStartTime) / 1000);
            var hours = Math.floor(diff / 3600);
            var minutes = Math.floor((diff % 3600) / 60);
            var seconds = diff % 60;
            
            var timeStr = (hours < 10 ? '0' : '') + hours + ':' +
                         (minutes < 10 ? '0' : '') + minutes + ':' +
                         (seconds < 10 ? '0' : '') + seconds;
            document.getElementById('sessionTime').textContent = timeStr;
        }
        
        function startLogging() {
            if (loggingActive) return;
            
            console.log('Starting data logging...');
            
            // Call backend to start logging
            fetch('/logging/control', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({action: 'start'})
            })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success') {
                    loggingActive = true;
                    sessionStartTime = new Date();
                    
                    updateLogStatus('Active');
                    updateLogEntries(0);
                    updateLogSize(0);
                    
                    // Start session timer
                    sessionTimer = setInterval(updateSessionTime, 1000);
                    
                    // Start periodic data refresh (faster updates during logging)
                    dataRefreshTimer = setInterval(refreshLogData, 1000);
                    
                    // Update button states
                    document.getElementById('startBtn').disabled = true;
                    document.getElementById('stopBtn').disabled = false;
                    
                    // Show physics charts
                    document.getElementById('physicsCharts').style.display = 'block';
                    initPhysicsCharts();
                    
                    // Unlock charts for real-time updates
                    chartsLocked = false;
                    document.getElementById('chartsStatus').textContent = '(Live Updates)';
                    document.getElementById('chartsStatus').style.color = '#2ea043';
                    
                    // Start dedicated CSV streaming for unified TelemetryFrame
                    startCsvStream();
                    
                    console.log('Logging started successfully');
                }
            })
            .catch(error => {
                console.error('Failed to start logging:', error);
                alert('Failed to start logging: ' + error.message);
            });
        }
        
        function stopLogging() {
            if (!loggingActive) return;
            
            console.log('Stopping data logging...');
            
            // Call backend to stop logging
            fetch('/logging/control', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({action: 'stop'})
            })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success') {
                    loggingActive = false;
                    sessionStartTime = null;
                    
                    updateLogStatus('Stopped');
                    
                    // Stop timers
                    if (sessionTimer) {
                        clearInterval(sessionTimer);
                        sessionTimer = null;
                    }
                    if (dataRefreshTimer) {
                        clearInterval(dataRefreshTimer);
                        dataRefreshTimer = null;
                    }
                    
                    // Update button states
                    document.getElementById('startBtn').disabled = false;
                    document.getElementById('stopBtn').disabled = true;
                    
                    document.getElementById('sessionTime').textContent = '00:00:00';
                    
                    // Keep charts visible after stopping - they should persist until user clears data
                    // Charts will only be hidden when user explicitly clicks "Clear Data"
                    
                    // Lock charts to show final logged data - stop real-time updates
                    chartsLocked = true;
                    document.getElementById('chartsStatus').textContent = '(Showing Last Logged Data)';
                    document.getElementById('chartsStatus').style.color = '#d29922';
                    
                    // Stop CSV stream and download collected data
                    stopCsvStreamAndDownload();
                    
                    console.log('Logging stopped successfully');
                }
            })
            .catch(error => {
                console.error('Failed to stop logging:', error);
                alert('Failed to stop logging: ' + error.message);
            });
        }

        function updatePhysicsChartsVisibility() {
            // Check current status to determine if we have logged data
            fetch('/logging/control')
            .then(response => response.json())
            .then(data => {
                const physicsCharts = document.getElementById('physicsCharts');
                if (data.entry_count > 0) {
                    // Show charts if we have logged data
                    physicsCharts.style.display = 'block';
                } else {
                    // Hide charts if no logged data
                    physicsCharts.style.display = 'none';
                }
            })
            .catch(error => {
                console.error('Failed to check data status for chart visibility:', error);
            });
        }

        function clearData() {
            if (loggingActive) {
                alert('Please stop logging before clearing data.');
                return;
            }
            
            if (!confirm('Are you sure you want to clear all logged data? This cannot be undone.')) {
                return;
            }
            
            console.log('Clearing log data...');
            
            fetch('/logging/control', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({action: 'clear'})
            })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'success') {
                    updateLogEntries(0);
                    updateLogSize(0);
                    document.getElementById('lastEntry').textContent = 'Never';
                    document.getElementById('memoryUsage').textContent = '0%';
                    
                    // Hide physics charts when data is cleared
                    document.getElementById('physicsCharts').style.display = 'none';
                    
                    // Unlock charts for next logging session
                    chartsLocked = false;
                    document.getElementById('chartsStatus').textContent = '';
                    
                    // Reset brake events counter
                    document.getElementById('brakeEvents').textContent = '0';
                    // Reset client-side CSV buffers/stats
                    csvRows = [];
                    csvBytes = 0;
                    streamingRowCount = 0;
                    
                    console.log('Data cleared successfully');
                    alert('All logged data has been cleared successfully.');
                } else {
                    alert('Failed to clear data: ' + (data.error || 'Unknown error'));
                }
            })
            .catch(error => {
                console.error('Failed to clear data:', error);
                alert('Failed to clear data: ' + error.message);
            });
        }

        var lastEntryCount = 0;
        var lastUpdateTime = Date.now();
        
        function refreshLogData() {
            if (!loggingActive) return;

            // Preferred: use CSV stream stats
            if (csvWS && csvWS.readyState === WebSocket.OPEN) {
                const totalRows = streamingRowCount; // counted from CSV rows
                updateLogEntries(totalRows);
                updateLogSize(Math.max(1, Math.floor(csvBytes / 1024))); // actual bytes seen
                updateLastEntry();
                document.getElementById('memoryUsage').textContent = Math.min((totalRows / 2000) * 100, 100).toFixed(1) + '%';
                lastUpdateTime = Date.now();
                return;
            }

            // If CSV WS closed but we still have rows buffered, reflect them
            if (csvRows && csvRows.length > 1) {
                const totalRows = streamingRowCount;
                updateLogEntries(totalRows);
                updateLogSize(Math.max(1, Math.floor(csvBytes / 1024)));
                document.getElementById('memoryUsage').textContent = Math.min((totalRows / 2000) * 100, 100).toFixed(1) + '%';
                return;
            }

            // Fallback: server-side status (still used for buttons/diagnostics but not for CSV counts)
            fetch('/logging/control')
            .then(response => response.json())
            .then(() => {
                // No-op; UI already driven by CSV
            })
            .catch(error => console.error('Failed to refresh log data:', error));
        }
        
        // Physics data visualization
        var speedChart, safetyChart;
        var speedData = [];
        var distanceData = [];
        var rcInputData = [];
        var safetyData = [];
        var maxDataPoints = 50;
        var chartsLocked = false; // When true, charts freeze and don't update
        
        function initPhysicsCharts() {
            // Initialize speed chart
            const speedCtx = document.getElementById('speedChart').getContext('2d');
            speedChart = {
                ctx: speedCtx,
                data: { speed: [], distance: [] }
            };
            
            // Initialize safety chart  
            const safetyCtx = document.getElementById('safetyChart').getContext('2d');
            safetyChart = {
                ctx: safetyCtx,
                data: { rcInput: [], safety: [] }
            };
            
            // Clear existing data arrays
            speedData = [];
            distanceData = [];
            rcInputData = [];
            safetyData = [];
        }
        
        function updatePhysicsDisplay(telemetryData) {
            // Skip updates if charts are locked (after logging stops)
            if (chartsLocked) {
                return; // Charts frozen on last logged data
            }
            
            // Use real telemetry data for physics charts
            if (!telemetryData) {
                return; // No data available, skip update
            }
            
            // Extract real vehicle physics data from telemetry
            var rcInput = telemetryData.rc_input || 0; // RC input percentage
            var distance = (telemetryData.distance || 0) * 100; // Convert m to cm for chart
            var speed = telemetryData.speed_est || 0; // Speed in km/h
            var safetyMargin = Math.max(0, distance - 100); // Safety calculation in cm
            
            // Update data arrays
            speedData.push(speed);
            distanceData.push(distance);
            rcInputData.push(rcInput);
            safetyData.push(safetyMargin);
            
            // Limit array sizes
            if (speedData.length > maxDataPoints) speedData.shift();
            if (distanceData.length > maxDataPoints) distanceData.shift();
            if (rcInputData.length > maxDataPoints) rcInputData.shift();
            if (safetyData.length > maxDataPoints) safetyData.shift();
            
            // Update charts with dual-axis system
            drawDualAxisChart(speedChart.ctx, 
                { data: speedData, color: '#2ea043', label: 'Speed' },
                { data: distanceData, color: '#d29922', label: 'Distance' },
                { min: 0, max: 50, unit: 'km/h', decimals: 0 },
                { min: 0, max: 300, unit: 'cm', decimals: 0 }
            );
            
            drawDualAxisChart(safetyChart.ctx, 
                { data: rcInputData, color: '#2ea043', label: 'RC Input' },
                { data: safetyData, color: '#f85149', label: 'Safety Margin' },
                { min: 0, max: 15, unit: '%', decimals: 1 },
                { min: 0, max: 300, unit: 'cm', decimals: 0 }
            );
            
            // Update summary values
            document.getElementById('currentRcInput').textContent = rcInput.toFixed(1);
            document.getElementById('currentDistance').textContent = distance.toFixed(1) + ' cm';
            document.getElementById('currentSafety').textContent = safetyMargin.toFixed(1) + ' cm';
            
            // Simulate brake events
            if (safetyMargin < 20) {
                var currentBrakes = parseInt(document.getElementById('brakeEvents').textContent) || 0;
                document.getElementById('brakeEvents').textContent = currentBrakes + 1;
            }
        }
        
        function drawDualAxisChart(ctx, leftDataset, rightDataset, leftRange, rightRange) {
            const canvas = ctx.canvas;
            const width = canvas.width;
            const height = canvas.height;
            const leftPadding = 35; // Reduced padding for wider charts
            const rightPadding = 35; // Reduced padding for wider charts
            const chartWidth = width - leftPadding - rightPadding;
            const chartHeight = height - 20; // Reduced bottom padding since no axis labels
            
            // Clear canvas
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, width, height);
            
            // Draw left Y-axis scale and labels
            ctx.fillStyle = '#c9d1d9';
            ctx.font = '10px Arial';
            ctx.textAlign = 'right';
            ctx.textBaseline = 'middle';
            
            const steps = 5;
            
            // Left axis
            for (let i = 0; i <= steps; i++) {
                const value = leftRange.min + (leftRange.max - leftRange.min) * (i / steps);
                const y = chartHeight - (i / steps) * chartHeight + 10;
                
                // Draw left tick mark
                ctx.strokeStyle = leftDataset.color;
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(leftPadding - 5, y);
                ctx.lineTo(leftPadding, y);
                ctx.stroke();
                
                // Draw left label with color coding
                ctx.fillStyle = leftDataset.color;
                ctx.fillText(value.toFixed(leftRange.decimals || 0), leftPadding - 8, y);
                
                // Draw grid line (subtle)
                if (i > 0 && i < steps) {
                    ctx.strokeStyle = '#222';
                    ctx.lineWidth = 1;
                    ctx.beginPath();
                    ctx.moveTo(leftPadding, y);
                    ctx.lineTo(leftPadding + chartWidth, y);
                    ctx.stroke();
                }
            }
            
            // Right axis
            ctx.textAlign = 'left';
            for (let i = 0; i <= steps; i++) {
                const value = rightRange.min + (rightRange.max - rightRange.min) * (i / steps);
                const y = chartHeight - (i / steps) * chartHeight + 10;
                
                // Draw right tick mark
                ctx.strokeStyle = rightDataset.color;
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(leftPadding + chartWidth, y);
                ctx.lineTo(leftPadding + chartWidth + 5, y);
                ctx.stroke();
                
                // Draw right label with color coding
                ctx.fillStyle = rightDataset.color;
                ctx.fillText(value.toFixed(rightRange.decimals || 0), leftPadding + chartWidth + 8, y);
            }
            
            // Draw vertical grid lines
            ctx.strokeStyle = '#222';
            ctx.lineWidth = 1;
            for (let i = 1; i < 10; i++) {
                const x = leftPadding + (i / 10) * chartWidth;
                ctx.beginPath();
                ctx.moveTo(x, 10);
                ctx.lineTo(x, chartHeight + 10);
                ctx.stroke();
            }
            
            // Draw left dataset
            if (leftDataset.data.length >= 2) {
                ctx.strokeStyle = leftDataset.color;
                ctx.lineWidth = 2;
                ctx.beginPath();
                
                leftDataset.data.forEach((value, index) => {
                    const x = leftPadding + (index / (maxDataPoints - 1)) * chartWidth;
                    const y = (chartHeight + 10) - ((value - leftRange.min) / (leftRange.max - leftRange.min)) * chartHeight;
                    
                    if (index === 0) {
                        ctx.moveTo(x, y);
                    } else {
                        ctx.lineTo(x, y);
                    }
                });
                
                ctx.stroke();
            }
            
            // Draw right dataset
            if (rightDataset.data.length >= 2) {
                ctx.strokeStyle = rightDataset.color;
                ctx.lineWidth = 2;
                ctx.beginPath();
                
                rightDataset.data.forEach((value, index) => {
                    const x = leftPadding + (index / (maxDataPoints - 1)) * chartWidth;
                    const y = (chartHeight + 10) - ((value - rightRange.min) / (rightRange.max - rightRange.min)) * chartHeight;
                    
                    if (index === 0) {
                        ctx.moveTo(x, y);
                    } else {
                        ctx.lineTo(x, y);
                    }
                });
                
                ctx.stroke();
            }
            
            // Draw chart border
            ctx.strokeStyle = '#555';
            ctx.lineWidth = 1;
            ctx.strokeRect(leftPadding, 10, chartWidth, chartHeight);
            
            // No axis labels needed - all info is in the legend
        }
        
        function exportData() {
            console.log('Exporting log data...');
            
            // Get current entry count first
            fetch('/logging/control')
            .then(response => response.json())
            .then(statusData => {
                if (statusData.entry_count === 0) {
                    alert('No data to export. Start logging first.');
                    return;
                }
                
                // Fetch the actual log data as CSV
                return fetch('/logging/data');
            })
            .then(response => {
                if (!response.ok) {
                    throw new Error('Failed to fetch data: ' + response.statusText);
                }
                return response.text(); // Get as text since it's CSV now
            })
            .then(csvData => {
                if (!csvData || csvData.length === 0) {
                    alert('No data available to export');
                    return;
                }
                
                // Check if the response is an error message
                if (csvData.startsWith('error')) {
                    alert('Export failed: ' + csvData.substring(6)); // Remove 'error\n' prefix
                    return;
                }
                
                // Create and download CSV file
                const blob = new Blob([csvData], { type: 'text/csv' });
                const url = window.URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = 'digitoys_physics_data_' + new Date().toISOString().replace(/[:.]/g, '-') + '.csv';
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                window.URL.revokeObjectURL(url);
                
                console.log('CSV file downloaded successfully');
            })
            .catch(error => {
                console.error('Failed to export data:', error);
                alert('Failed to export data: ' + error.message);
            });
        }
        
        // Initialize logging UI
        function initLogging() {
            var startBtn = document.getElementById('startBtn');
            var stopBtn = document.getElementById('stopBtn');
            var exportBtn = document.getElementById('exportBtn');
            var clearBtn = document.getElementById('clearBtn');
            
            if (startBtn) startBtn.addEventListener('click', startLogging);
            if (stopBtn) stopBtn.addEventListener('click', stopLogging);
            if (exportBtn) exportBtn.addEventListener('click', exportData);
            if (clearBtn) clearBtn.addEventListener('click', clearData);
            
            // Initial button states
            if (stopBtn) stopBtn.disabled = true;
            
            // Restore dashboard state from server
            restoreDashboardState();
            
            console.log('Logging UI initialized');
        }

        // Restore dashboard state on page load
        function restoreDashboardState() {
            console.log('Restoring dashboard state...');
            
            fetch('/logging/control')
            .then(response => response.json())
            .then(data => {
                console.log('Dashboard state received:', data);
                
                // Update logging status
                loggingActive = data.logging_active || false;
                document.getElementById('logStatus').textContent = loggingActive ? 'Running' : 'Stopped';
                
                // Update charts lock status based on logging state
                if (loggingActive) {
                    chartsLocked = false;
                    document.getElementById('chartsStatus').textContent = '(Live Updates)';
                    document.getElementById('chartsStatus').style.color = '#2ea043';
                } else if (data.entry_count > 0) {
                    chartsLocked = true;
                    document.getElementById('chartsStatus').textContent = '(Showing Last Logged Data)';
                    document.getElementById('chartsStatus').style.color = '#d29922';
                } else {
                    chartsLocked = false;
                    document.getElementById('chartsStatus').textContent = '';
                }
                
                // Update button states
                var startBtn = document.getElementById('startBtn');
                var stopBtn = document.getElementById('stopBtn');
                if (startBtn) startBtn.disabled = loggingActive;
                if (stopBtn) stopBtn.disabled = !loggingActive;
                
                // Update entry count
                if (data.entry_count !== undefined) {
                    updateLogEntries(data.entry_count);
                }
                
                // Update data size
                if (data.data_size_kb !== undefined) {
                    updateLogSize(data.data_size_kb);
                }
                
                // Update memory usage
                if (data.memory_usage_percent !== undefined) {
                    document.getElementById('memoryUsage').textContent = 
                        Math.round(data.memory_usage_percent * 10) / 10 + '%';
                }
                
                // Update last entry (if we have data)
                if (data.entry_count > 0) {
                    document.getElementById('lastEntry').textContent = 'Recently';
                    // Show physics charts if we have logged data
                    document.getElementById('physicsCharts').style.display = 'block';
                } else {
                    document.getElementById('lastEntry').textContent = 'Never';
                    // Hide physics charts if no logged data
                    document.getElementById('physicsCharts').style.display = 'none';
                }
                
                console.log('Dashboard state restored successfully');
            })
            .catch(error => {
                console.error('Failed to restore dashboard state:', error);
                // Continue with default state if restoration fails
            });
        }
        
        // Initialize everything
        initLogging();
        
        // Initialize console controls
        document.getElementById('consoleRunBtn').addEventListener('click', () => {
            if (!consoleRunning) toggleConsole();
        });
        document.getElementById('consolePauseBtn').addEventListener('click', () => {
            if (consoleRunning) toggleConsole();
        });
        document.getElementById('consoleClearBtn').addEventListener('click', clearConsole);
        
        console.log('Dashboard initialized with WebSocket + HTTP fallback');
    </script>
</body>
</html>)";

        return httpd_resp_send(req, dashboard_html, HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t WifiMonitor::websocketHandler(httpd_req_t *req)
    {
        if (!instance_)
        {
            DIGITOYS_LOGE("WifiMonitor", "WebSocket handler called but no instance available");
            return ESP_ERR_INVALID_STATE;
        }

        if (req->method == HTTP_GET)
        {
            DIGITOYS_LOGI("WifiMonitor", "WebSocket handshake from client");
            // Track generic WS clients as best-effort (system dashboard channel)
            if (xSemaphoreTake(instance_->ws_clients_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                int sockfd = httpd_req_to_sockfd(req);
                instance_->websocket_clients_.push_back(sockfd);
                DIGITOYS_LOGD("WifiMonitor", "Added WS client: %d (total=%u)", sockfd, (unsigned)instance_->websocket_clients_.size());
                xSemaphoreGive(instance_->ws_clients_mutex_);
            }
            return ESP_OK;
        }

        // Handle WebSocket frame
        httpd_ws_frame_t ws_pkt;
        uint8_t *buf = nullptr;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

        // First call to get frame length
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "httpd_ws_recv_frame failed to get frame len with %d", ret);
            return ret;
        }

        if (ws_pkt.len)
        {
            // Allocate buffer for the payload
            buf = (uint8_t *)calloc(1, ws_pkt.len + 1);
            if (buf == nullptr)
            {
                DIGITOYS_LOGE("WifiMonitor", "Failed to calloc memory for buf");
                return ESP_ERR_NO_MEM;
            }
            ws_pkt.payload = buf;

            // Second call to get the actual frame payload
            ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
            if (ret != ESP_OK)
            {
                DIGITOYS_LOGE("WifiMonitor", "httpd_ws_recv_frame failed with %d", ret);
                free(buf);
                return ret;
            }
        }

        // Handle different frame types
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT)
        {
            // DIGITOYS_LOGI("WifiMonitor", "Received WebSocket text: %.*s", ws_pkt.len, (char *)ws_pkt.payload);

            // Send system data as JSON response
            cJSON *root = cJSON_CreateObject();

            // Get real system data (reuse logic from systemGetHandler)
            static uint32_t prev_idle_time = 0;
            static uint32_t prev_total_time = 0;
            static uint32_t ws_counter = 0;
            ws_counter++;

            float cpu_usage = calculate_cpu_load(prev_idle_time, prev_total_time);
            size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
            size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

            cJSON_AddNumberToObject(root, "cpu", cpu_usage);
            cJSON_AddNumberToObject(root, "total_heap", (double)total_heap);
            cJSON_AddNumberToObject(root, "free_heap", (double)free_heap);
            cJSON_AddNumberToObject(root, "counter", ws_counter);
            cJSON_AddStringToObject(root, "type", "websocket");

            // Add telemetry data for Vehicle Telemetry dashboard section
            if (instance_->telemetry_mutex_ && xSemaphoreTake(instance_->telemetry_mutex_, pdMS_TO_TICKS(50)))
            {
                cJSON *telemetry = cJSON_CreateObject();
                cJSON_AddBoolToObject(telemetry, "obstacle", instance_->telemetry_data_.obstacle);
                cJSON_AddNumberToObject(telemetry, "distance", instance_->telemetry_data_.distance);
                cJSON_AddNumberToObject(telemetry, "speed_est", instance_->telemetry_data_.speed_est);
                cJSON_AddBoolToObject(telemetry, "warning", instance_->telemetry_data_.warning);
                cJSON_AddNumberToObject(telemetry, "timestamp", instance_->telemetry_data_.timestamp);

                // Add RC input for dashboard display (duty cycle as percentage)
                cJSON_AddNumberToObject(telemetry, "rc_input", instance_->telemetry_data_.speed_est * 100.0f);

                cJSON_AddItemToObject(root, "telemetry", telemetry);
                xSemaphoreGive(instance_->telemetry_mutex_);
            }
            else
            {
                // Fallback: add empty telemetry object
                cJSON *telemetry = cJSON_CreateObject();
                cJSON_AddBoolToObject(telemetry, "obstacle", false);
                cJSON_AddNumberToObject(telemetry, "distance", 0.0);
                cJSON_AddNumberToObject(telemetry, "speed_est", 0.0);
                cJSON_AddBoolToObject(telemetry, "warning", false);
                cJSON_AddNumberToObject(telemetry, "timestamp", 0);
                cJSON_AddNumberToObject(telemetry, "rc_input", 0.0);
                cJSON_AddItemToObject(root, "telemetry", telemetry);
            }

            // Add some task info
            const int MAX_TASKS = 5; // Limit for WebSocket to reduce payload
            TaskStatus_t status[MAX_TASKS];
            uint32_t total_time = 0;
            UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, &total_time);

            cJSON *tasks = cJSON_CreateArray();
            for (UBaseType_t i = 0; i < count && i < MAX_TASKS; ++i)
            {
                cJSON *task = cJSON_CreateObject();
                cJSON_AddStringToObject(task, "name", status[i].pcTaskName);
                cJSON_AddNumberToObject(task, "hwm", status[i].usStackHighWaterMark);
                cJSON_AddItemToArray(tasks, task);
            }
            cJSON_AddItemToObject(root, "tasks", tasks);

            // Add recent system logs for console display
            cJSON *logs = cJSON_CreateArray();

            // Get recent system logs from the captured log buffer
            if (instance_)
            {
                std::vector<std::string> recent_logs = instance_->getRecentSystemLogs(20);
                for (const auto &log_line : recent_logs)
                {
                    cJSON_AddItemToArray(logs, cJSON_CreateString(log_line.c_str()));
                }
            }

            // Add dynamic logs based on telemetry state
            if (instance_->telemetry_mutex_ && xSemaphoreTake(instance_->telemetry_mutex_, pdMS_TO_TICKS(10)))
            {
                char log_buffer[256];
                if (instance_->telemetry_data_.warning)
                {
                    snprintf(log_buffer, sizeof(log_buffer),
                             "W (%u) SPEED_CONTROLLER: Dynamic warning! Speed=%.4f, ActualDist=%.2fm",
                             (unsigned int)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                             instance_->telemetry_data_.speed_est,
                             instance_->telemetry_data_.distance);
                    cJSON_AddItemToArray(logs, cJSON_CreateString(log_buffer));
                }
                if (instance_->telemetry_data_.obstacle)
                {
                    snprintf(log_buffer, sizeof(log_buffer),
                             "E (%u) SPEED_CONTROLLER: EMERGENCY BRAKE! Distance=%.2fm",
                             (unsigned int)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                             instance_->telemetry_data_.distance);
                    cJSON_AddItemToArray(logs, cJSON_CreateString(log_buffer));
                }
                xSemaphoreGive(instance_->telemetry_mutex_);
            }

            cJSON_AddItemToObject(root, "logs", logs);

            char *json_string = cJSON_PrintUnformatted(root);
            cJSON_Delete(root);

            if (json_string)
            {
                httpd_ws_frame_t response = {
                    .final = true,
                    .fragmented = false,
                    .type = HTTPD_WS_TYPE_TEXT,
                    .payload = (uint8_t *)json_string,
                    .len = strlen(json_string)};

                ret = httpd_ws_send_frame(req, &response);
                free(json_string);

                if (ret != ESP_OK)
                {
                    DIGITOYS_LOGE("WifiMonitor", "httpd_ws_send_frame failed with %d", ret);
                }
            }
        }
        else if (ws_pkt.type == HTTPD_WS_TYPE_PING)
        {
            // Respond to ping with pong
            httpd_ws_frame_t pong = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_PONG,
                .payload = nullptr,
                .len = 0};
            ret = httpd_ws_send_frame(req, &pong);
        }
        else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
        {
            DIGITOYS_LOGI("WifiMonitor", "WebSocket connection closed by client");
            // Remove from generic WS clients list
            if (xSemaphoreTake(instance_->ws_clients_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                int sockfd = httpd_req_to_sockfd(req);
                auto it = std::find(instance_->websocket_clients_.begin(), instance_->websocket_clients_.end(), sockfd);
                if (it != instance_->websocket_clients_.end())
                {
                    instance_->websocket_clients_.erase(it);
                    DIGITOYS_LOGD("WifiMonitor", "Removed WS client: %d (total=%u)", sockfd, (unsigned)instance_->websocket_clients_.size());
                }
                xSemaphoreGive(instance_->ws_clients_mutex_);
            }
        }

        if (buf)
        {
            free(buf);
        }
        return ret;
    }

    esp_err_t WifiMonitor::websocketDataHandler(httpd_req_t *req)
    {
        if (!instance_)
        {
            DIGITOYS_LOGE("WifiMonitor", "WebSocket data handler called but no instance available");
            return ESP_ERR_INVALID_STATE;
        }

        if (req->method == HTTP_GET)
        {
            DIGITOYS_LOGI("WifiMonitor", "WebSocket data streaming handshake from client");

            // Add client to data streaming list
            if (xSemaphoreTake(instance_->ws_clients_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                int sockfd = httpd_req_to_sockfd(req);
                instance_->websocket_data_clients_.push_back(sockfd);
                DIGITOYS_LOGI("WifiMonitor", "Added data streaming client: %d (total=%u)", sockfd, (unsigned)instance_->websocket_data_clients_.size());
                xSemaphoreGive(instance_->ws_clients_mutex_);
            }

            return ESP_OK;
        }

        // Handle client disconnection or data frames
        httpd_ws_frame_t ws_pkt;
        uint8_t *buf = nullptr;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "httpd_ws_recv_frame failed with %d", ret);
            return ret;
        }

        if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
        {
            // Remove client from data streaming list
            if (xSemaphoreTake(instance_->ws_clients_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                int sockfd = httpd_req_to_sockfd(req);
                auto it = std::find(instance_->websocket_data_clients_.begin(),
                                    instance_->websocket_data_clients_.end(), sockfd);
                if (it != instance_->websocket_data_clients_.end())
                {
                    instance_->websocket_data_clients_.erase(it);
                    DIGITOYS_LOGI("WifiMonitor", "Removed data streaming client: %d (total=%u)", sockfd, (unsigned)instance_->websocket_data_clients_.size());
                }
                xSemaphoreGive(instance_->ws_clients_mutex_);
            }
        }

        if (buf)
        {
            free(buf);
        }
        return ESP_OK;
    }

    esp_err_t WifiMonitor::websocketCsvHandler(httpd_req_t *req)
    {
        if (!instance_)
        {
            DIGITOYS_LOGE("WifiMonitor", "WebSocket CSV handler called but no instance available");
            return ESP_ERR_INVALID_STATE;
        }

        if (req->method == HTTP_GET)
        {
            // Distinguish plain HTTP GET vs WebSocket upgrade
            if (!isWebSocketFrame(req))
            {
                const char *msg = "<!DOCTYPE html><html><body><h3>/ws/csv</h3><p>This is a WebSocket endpoint for live CSV streaming.</p><p>Please use ws://&lt;host&gt;/ws/csv from a WebSocket client.</p></body></html>";
                httpd_resp_set_type(req, "text/html");
                httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }

            DIGITOYS_LOGI("WifiMonitor", "WebSocket CSV handshake from client");
            // Add client to CSV clients list and send header
            int sockfd = httpd_req_to_sockfd(req);
            if (xSemaphoreTake(instance_->ws_clients_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                instance_->websocket_csv_clients_.push_back(sockfd);
                DIGITOYS_LOGI("WifiMonitor", "Added CSV client: %d (total=%u)", sockfd, (unsigned)instance_->websocket_csv_clients_.size());
                xSemaphoreGive(instance_->ws_clients_mutex_);
            }

            // Send CSV header immediately
            const char *header = "seq,ts_us,rc_duty_raw,rc_throttle_pressed,rc_forward,rc_reverse,lidar_distance_m,lidar_filtered_m,obstacle_detected,warning_active,brake_distance_m,warning_distance_m,safety_margin_m,speed_approx_mps\n";
            httpd_ws_frame_t ws_pkt = {};
            ws_pkt.payload = (uint8_t *)header;
            ws_pkt.len = strlen(header);
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
            (void)httpd_ws_send_frame_async(instance_->server_, sockfd, &ws_pkt);
            return ESP_OK;
        }

        // Control frames: detect close to remove client
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "websocketCsvHandler recv len failed: %d", ret);
            return ret;
        }
        if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE)
        {
            if (xSemaphoreTake(instance_->ws_clients_mutex_, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                int sockfd = httpd_req_to_sockfd(req);
                auto it = std::find(instance_->websocket_csv_clients_.begin(), instance_->websocket_csv_clients_.end(), sockfd);
                if (it != instance_->websocket_csv_clients_.end())
                {
                    instance_->websocket_csv_clients_.erase(it);
                    DIGITOYS_LOGI("WifiMonitor", "Removed CSV client: %d (total=%u)", sockfd, (unsigned)instance_->websocket_csv_clients_.size());
                }
                xSemaphoreGive(instance_->ws_clients_mutex_);
            }
        }
        return ESP_OK;
    }

    esp_err_t WifiMonitor::broadcastDataEntry(const std::string &data_json)
    {
        if (xSemaphoreTake(ws_clients_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            return ESP_ERR_TIMEOUT;
        }

        // Create WebSocket frame
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t *)data_json.c_str();
        ws_pkt.len = data_json.length();
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;

        // Send to all data streaming clients
        auto it = websocket_data_clients_.begin();
        while (it != websocket_data_clients_.end())
        {
            int sockfd = *it;

            // Try to send data
            if (httpd_ws_send_frame_async(server_, sockfd, &ws_pkt) != ESP_OK)
            {
                // Client disconnected, remove from list
                DIGITOYS_LOGW("WifiMonitor", "Failed to send data to client %d, removing", sockfd);
                it = websocket_data_clients_.erase(it);
            }
            else
            {
                ++it;
            }
        }

        xSemaphoreGive(ws_clients_mutex_);
        return ESP_OK;
    }

    void WifiMonitor::setupDataLoggerStreaming()
    {
        if (!data_logger_service_)
        {
            DIGITOYS_LOGW("WifiMonitor", "No DataLogger service available for streaming setup");
            return;
        }

        auto *data_logger = data_logger_service_->getDataLogger();
        if (!data_logger)
        {
            DIGITOYS_LOGW("WifiMonitor", "No DataLogger instance available for streaming setup");
            return;
        }

        // Set up streaming callback
        auto streaming_callback = [this](const digitoys::datalogger::DataEntry &entry, const std::string &source_name)
        {
            // Convert DataEntry to JSON and broadcast
            cJSON *json = cJSON_CreateObject();
            cJSON_AddStringToObject(json, "source", source_name.c_str());
            cJSON_AddStringToObject(json, "key", entry.key.c_str());
            cJSON_AddStringToObject(json, "value", entry.value.c_str());
            cJSON_AddNumberToObject(json, "timestamp_us", entry.timestamp_us);
            cJSON_AddNumberToObject(json, "type", static_cast<int>(entry.type));

            char *json_string = cJSON_Print(json);
            if (json_string)
            {
                std::string data_str(json_string);
                broadcastDataEntry(data_str);
                free(json_string);
            }
            cJSON_Delete(json);
        };

        data_logger->setStreamingCallback(streaming_callback);
        DIGITOYS_LOGI("WifiMonitor", "DataLogger streaming callback configured");
    }

    // WebSocket task function (placeholder for future implementation)
    void WifiMonitor::webSocketTaskFunction(void *param)
    {
        // TODO: Implement periodic WebSocket broadcast to all connected clients
    }

    // Diagnostic logging implementation
    void WifiMonitor::addDiagnosticEntry(const DiagnosticEntry &entry)
    {
        if (!logging_active_)
            return;

        if (xSemaphoreTake(diagnostic_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Add timestamp if not set
            DiagnosticEntry timestamped_entry = entry;
            if (timestamped_entry.timestamp == 0)
            {
                timestamped_entry.timestamp = esp_timer_get_time() / 1000; // Convert to milliseconds
            }

            diagnostic_log_.push_back(timestamped_entry);

            // Limit log size to prevent memory overflow
            if (diagnostic_log_.size() > MAX_LOG_ENTRIES)
            {
                diagnostic_log_.erase(diagnostic_log_.begin());
            }

            xSemaphoreGive(diagnostic_mutex_);
        }
    }

    void WifiMonitor::logControlDiagnostics(float cached_duty, float direct_duty, float current_input,
                                            float distance, float brake_distance, float warning_distance,
                                            bool cached_throttle, bool throttle_pressed, bool driving_forward,
                                            bool wants_reverse, bool obstacle_detected, bool warning_active)
    {
        DiagnosticEntry entry;
        entry.cached_duty = cached_duty;
        entry.direct_duty = direct_duty;
        entry.current_input = current_input;
        entry.distance = distance;
        entry.brake_distance = brake_distance;
        entry.warning_distance = warning_distance;
        entry.cached_throttle = cached_throttle;
        entry.throttle_pressed = throttle_pressed;
        entry.driving_forward = driving_forward;
        entry.wants_reverse = wants_reverse;
        entry.obstacle_detected = obstacle_detected;
        entry.warning_active = warning_active;

        addDiagnosticEntry(entry);
    }

    esp_err_t WifiMonitor::startLogging()
    {
        logging_active_ = true;
        DIGITOYS_LOGI("WifiMonitor", "Started TelemetryFrame CSV streaming (WebSocket only)");
        // Snapshot client counts for diagnostics
        size_t ws_cnt = 0, data_cnt = 0;
        if (ws_clients_mutex_ && xSemaphoreTake(ws_clients_mutex_, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            ws_cnt = websocket_clients_.size();
            data_cnt = websocket_data_clients_.size();
            xSemaphoreGive(ws_clients_mutex_);
        }
        DIGITOYS_LOGD("WifiMonitor", "Logging start: ws_clients=%u, data_clients=%u", (unsigned)ws_cnt, (unsigned)data_cnt);
        return ESP_OK;
    }

    esp_err_t WifiMonitor::stopLogging()
    {
        logging_active_ = false;
        DIGITOYS_LOGI("WifiMonitor", "Stopped TelemetryFrame CSV streaming");
        return ESP_OK;
    }

    bool WifiMonitor::isLoggingActive() const
    {
        bool active = logging_active_;
        if (data_logger_service_)
        {
            auto *data_logger = data_logger_service_->getDataLogger();
            if (data_logger)
            {
                active = active || (data_logger->isRunning() && !data_logger->isMonitoringMode());
            }
        }
        return active;
    }

    void WifiMonitor::clearDiagnosticData()
    {
        if (xSemaphoreTake(diagnostic_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            diagnostic_log_.clear();
            xSemaphoreGive(diagnostic_mutex_);
            DIGITOYS_LOGI("WifiMonitor", "Diagnostic log cleared");
        }
    }

    std::string WifiMonitor::getDiagnosticDataJSON() const
    {
        DIGITOYS_LOGI("WifiMonitor", "Starting getDiagnosticDataJSON");

        std::string json = "{\"entries\":[";
        size_t entry_count = 0;

        if (xSemaphoreTake(diagnostic_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            entry_count = diagnostic_log_.size();
            DIGITOYS_LOGI("WifiMonitor", "Processing %d diagnostic entries", entry_count);

            // Limit the number of entries to prevent memory issues
            size_t max_entries = 1000; // Limit to 1000 entries
            size_t actual_entries = std::min(entry_count, max_entries);

            for (size_t i = 0; i < actual_entries; ++i)
            {
                const auto &entry = diagnostic_log_[i];
                if (i > 0)
                    json += ",";

                json += "{";
                json += "\"timestamp\":" + std::to_string(entry.timestamp) + ",";
                json += "\"cached_duty\":" + std::to_string(entry.cached_duty) + ",";
                json += "\"direct_duty\":" + std::to_string(entry.direct_duty) + ",";
                json += "\"current_input\":" + std::to_string(entry.current_input) + ",";
                json += "\"distance\":" + std::to_string(entry.distance) + ",";
                json += "\"brake_distance\":" + std::to_string(entry.brake_distance) + ",";
                json += "\"warning_distance\":" + std::to_string(entry.warning_distance) + ",";
                json += "\"cached_throttle\":" + std::string(entry.cached_throttle ? "true" : "false") + ",";
                json += "\"throttle_pressed\":" + std::string(entry.throttle_pressed ? "true" : "false") + ",";
                json += "\"driving_forward\":" + std::string(entry.driving_forward ? "true" : "false") + ",";
                json += "\"wants_reverse\":" + std::string(entry.wants_reverse ? "true" : "false") + ",";
                json += "\"obstacle_detected\":" + std::string(entry.obstacle_detected ? "true" : "false") + ",";
                json += "\"warning_active\":" + std::string(entry.warning_active ? "true" : "false");
                json += "}";

                // Yield periodically to prevent watchdog timeouts
                if (i % 100 == 0)
                {
                    vTaskDelay(1); // Brief yield every 100 entries
                }
            }
            xSemaphoreGive(diagnostic_mutex_);

            DIGITOYS_LOGI("WifiMonitor", "Processed %d entries, JSON size: %d bytes", actual_entries, json.length());
        }
        else
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to acquire mutex for diagnostic data");
        }

        json += "],\"count\":" + std::to_string(entry_count) + "}";

        DIGITOYS_LOGI("WifiMonitor", "getDiagnosticDataJSON completed, final size: %d bytes", json.length());
        return json;
    }

    std::string WifiMonitor::getDiagnosticDataCSV() const
    {
        DIGITOYS_LOGI("WifiMonitor", "Starting getDiagnosticDataCSV");

        // CSV header
        std::string csv = "timestamp,cached_duty,direct_duty,current_input,distance,brake_distance,warning_distance,cached_throttle,throttle_pressed,driving_forward,wants_reverse,obstacle_detected,warning_active\n";

        size_t entry_count = 0;

        if (xSemaphoreTake(diagnostic_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            entry_count = diagnostic_log_.size();
            DIGITOYS_LOGI("WifiMonitor", "Processing %d diagnostic entries for CSV", entry_count);

            for (size_t i = 0; i < entry_count; ++i)
            {
                const auto &entry = diagnostic_log_[i];

                csv += std::to_string(entry.timestamp) + ",";
                csv += std::to_string(entry.cached_duty) + ",";
                csv += std::to_string(entry.direct_duty) + ",";
                csv += std::to_string(entry.current_input) + ",";
                csv += std::to_string(entry.distance) + ",";
                csv += std::to_string(entry.brake_distance) + ",";
                csv += std::to_string(entry.warning_distance) + ",";
                csv += (entry.cached_throttle ? "1" : "0") + std::string(",");
                csv += (entry.throttle_pressed ? "1" : "0") + std::string(",");
                csv += (entry.driving_forward ? "1" : "0") + std::string(",");
                csv += (entry.wants_reverse ? "1" : "0") + std::string(",");
                csv += (entry.obstacle_detected ? "1" : "0") + std::string(",");
                csv += (entry.warning_active ? "1" : "0") + std::string("\n");

                // Yield periodically to prevent watchdog timeouts
                if (i % 100 == 0)
                {
                    vTaskDelay(1); // Brief yield every 100 entries
                }
            }
            xSemaphoreGive(diagnostic_mutex_);

            DIGITOYS_LOGI("WifiMonitor", "Processed %d entries, CSV size: %d bytes", entry_count, csv.length());
        }
        else
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to acquire mutex for diagnostic CSV data");
        }

        DIGITOYS_LOGI("WifiMonitor", "getDiagnosticDataCSV completed, final size: %d bytes", csv.length());
        return csv;
    }

    std::string WifiMonitor::getDataLoggerJSON() const
    {
        DIGITOYS_LOGI("WifiMonitor", "Starting getDataLoggerJSON");

        if (!data_logger_service_)
        {
            DIGITOYS_LOGE("WifiMonitor", "DataLogger service not available");
            return "{\"error\":\"DataLogger service not available\",\"entries\":[],\"count\":0}";
        }

        auto *data_logger = data_logger_service_->getDataLogger();
        auto collected_data = data_logger->getCollectedData();

        std::string json = "{\"entries\":[";

        for (size_t i = 0; i < collected_data.size(); ++i)
        {
            const auto &entry = collected_data[i];

            if (i > 0)
                json += ",";
            json += "{";
            json += "\"key\":\"" + entry.key + "\",";
            json += "\"value\":\"" + entry.value + "\",";
            json += "\"timestamp\":" + std::to_string(entry.timestamp_us);
            json += "}";

            // Yield periodically for large datasets
            if (i % 100 == 0)
            {
                vTaskDelay(1);
            }
        }

        json += "],\"count\":" + std::to_string(collected_data.size()) + "}";

        DIGITOYS_LOGI("WifiMonitor", "getDataLoggerJSON completed, final size: %d bytes", json.length());
        return json;
    }

    std::string WifiMonitor::getDataLoggerCSV() const
    {
        DIGITOYS_LOGI("WifiMonitor", "Starting getDataLoggerCSV");

        if (!data_logger_service_)
        {
            DIGITOYS_LOGE("WifiMonitor", "DataLogger service not available");
            return "error\nDataLogger service not available\n";
        }

        auto *data_logger = data_logger_service_->getDataLogger();

        // Use logged data for export (persistent data from logging sessions)
        auto collected_data = data_logger->getLoggedData();

        if (collected_data.empty())
        {
            DIGITOYS_LOGW("WifiMonitor", "No logged data available for export");
            return "error\nNo logged data available. Start a logging session first.\n";
        }

        // CSV header: timestamp first, then source, key, value, and numeric type
        std::string csv = "timestamp_us,source,key,value,type\n";

        for (size_t i = 0; i < collected_data.size(); ++i)
        {
            const auto &entry = collected_data[i];

            csv += std::to_string(entry.timestamp_us) + ",";
            csv += (entry.source.empty() ? std::string("unknown") : entry.source) + ",";
            csv += entry.key + ",";
            csv += entry.value + ",";
            csv += std::to_string(static_cast<int>(entry.type)) + "\n";

            // Yield periodically for large datasets
            if (i % 100 == 0)
            {
                vTaskDelay(1);
            }
        }

        DIGITOYS_LOGI("WifiMonitor", "getDataLoggerCSV completed, final size: %d bytes, entries: %zu",
                      csv.length(), collected_data.size());
        return csv;
    }

    // Build a wide CSV sampled at a fixed rate with normalized units
    std::string WifiMonitor::getDataLoggerWideCSV(uint32_t rate_hz) const
    {
        if (!data_logger_service_)
        {
            return "error\nDataLogger service not available\n";
        }

        if (rate_hz == 0)
            rate_hz = 5;
        if (rate_hz > 50)
            rate_hz = 50; // sanity cap
        const uint32_t period_ms = 1000 / rate_hz;

        auto *data_logger = data_logger_service_->getDataLogger();
        auto entries = data_logger->getLoggedData(); // event-style entries across sources
        if (entries.empty())
        {
            return "error\nNo logged data available. Start a logging session first.\n";
        }

        // Determine start timestamp for relative time
        uint64_t start_ts = entries.front().timestamp_us;
        for (const auto &e : entries)
            if (e.timestamp_us < start_ts)
                start_ts = e.timestamp_us;

        // Header
        std::string csv =
            "timestamp_us,rc_input_pct,obstacle_distance_m,speed_est_kmh,brake_distance_m,safety_margin_m,"
            "is_obstacle,is_warning,driving_forward,driving_backward,wants_reverse,throttle_pressed,"
            "time_to_impact_s,deceleration_mps2,brake_events,warning_events,source\n";

        // State to carry forward values between samples
        struct RowState
        {
            float rc_input_pct = NAN;
            float dist_m = NAN;
            float speed_kmh = NAN;
            float brake_m = NAN;
            float safety_m = NAN;
            int obstacle = 0;
            int warning = 0;
            int driving_fwd = 0;
            int driving_bwd = 0;
            int wants_rev = 0;
            int throttle = 0;
            float tti_s = NAN;
            float decel_mps2 = NAN;
            uint32_t brake_events = 0;
            uint32_t warning_events = 0;
            std::string source;
        } state;

        // Helper to parse numeric safely
        auto parseF = [](const std::string &s, float &out)
        {
            char *end=nullptr; out = strtof(s.c_str(), &end); return end && *end=='\0'; };
        auto parseU = [](const std::string &s, uint32_t &out)
        {
            char *end=nullptr; unsigned long v=strtoul(s.c_str(), &end, 10); if(end&&*end=='\0'){out=(uint32_t)v; return true;} return false; };

        // Walk time in fixed steps, and for each bucket, fold all entries within [t, t+period)
        uint64_t end_ts = entries.back().timestamp_us;
        for (uint64_t t = start_ts; t <= end_ts; t += (uint64_t)period_ms * 1000ULL)
        {
            // Apply entries that fall into this bucket
            for (const auto &e : entries)
            {
                if (e.timestamp_us < t || e.timestamp_us >= t + (uint64_t)period_ms * 1000ULL)
                    continue;

                // Update source (last writer wins)
                if (!e.source.empty())
                    state.source = e.source;

                // Normalize by key
                if (e.key == "rc_input")
                {
                    float v;
                    if (parseF(e.value, v))
                        state.rc_input_pct = v * 100.0f;
                }
                else if (e.key == "obstacle_distance")
                {
                    float v;
                    if (parseF(e.value, v))
                    {
                        // Assume ControlTask may be cm; if clearly >10, convert cm->m; otherwise keep
                        state.dist_m = (v > 10.0f ? v / 100.0f : v);
                    }
                }
                else if (e.key == "calculated_speed" || e.key == "speed_estimate")
                {
                    float v;
                    if (parseF(e.value, v))
                        state.speed_kmh = v * 3.6f; // m/s -> km/h
                }
                else if (e.key == "brake_distance")
                {
                    float v;
                    if (parseF(e.value, v))
                        state.brake_m = (v > 10.0f ? v / 100.0f : v);
                }
                else if (e.key == "safety_margin")
                {
                    float v;
                    if (parseF(e.value, v))
                        state.safety_m = (v > 10.0f ? v / 100.0f : v);
                }
                else if (e.key == "is_obstacle_state" || e.key == "obstacle_detected")
                {
                    state.obstacle = (e.value == "1" || e.value == "true");
                }
                else if (e.key == "is_warning_state" || e.key == "warning_active")
                {
                    state.warning = (e.value == "1" || e.value == "true");
                }
                else if (e.key == "driving_forward")
                {
                    state.driving_fwd = (e.value == "1" || e.value == "true");
                }
                else if (e.key == "wants_reverse")
                {
                    state.wants_rev = (e.value == "1" || e.value == "true");
                }
                else if (e.key == "throttle_pressed")
                {
                    state.throttle = (e.value == "1" || e.value == "true");
                }
                else if (e.key == "time_to_impact")
                {
                    float v;
                    if (parseF(e.value, v))
                        state.tti_s = v;
                }
                else if (e.key == "deceleration")
                {
                    float v;
                    if (parseF(e.value, v))
                        state.decel_mps2 = v;
                }
                else if (e.key == "brake_events")
                {
                    uint32_t v;
                    if (parseU(e.value, v))
                        state.brake_events = v;
                }
                else if (e.key == "warning_events")
                {
                    uint32_t v;
                    if (parseU(e.value, v))
                        state.warning_events = v;
                }
            }

            // Derive driving_backward if possible: wants_reverse and not forward
            state.driving_bwd = (state.wants_rev && !state.driving_fwd) ? 1 : 0;

            // Write row using relative timestamp
            uint64_t rel_us = (t - start_ts);
            auto fmtOpt = [](float v)
            { return std::isnan(v) ? std::string("") : std::to_string(v); };
            csv += std::to_string(rel_us) + ",";
            csv += fmtOpt(state.rc_input_pct) + ",";
            csv += fmtOpt(state.dist_m) + ",";
            csv += fmtOpt(state.speed_kmh) + ",";
            csv += fmtOpt(state.brake_m) + ",";
            csv += fmtOpt(state.safety_m) + ",";
            csv += std::to_string(state.obstacle) + ",";
            csv += std::to_string(state.warning) + ",";
            csv += std::to_string(state.driving_fwd) + ",";
            csv += std::to_string(state.driving_bwd) + ",";
            csv += std::to_string(state.wants_rev) + ",";
            csv += std::to_string(state.throttle) + ",";
            csv += fmtOpt(state.tti_s) + ",";
            csv += fmtOpt(state.decel_mps2) + ",";
            csv += std::to_string(state.brake_events) + ",";
            csv += std::to_string(state.warning_events) + ",";
            csv += (state.source.empty() ? std::string("") : state.source) + "\n";
        }

        return csv;
    }

    // HTTP handlers for diagnostic logging
    esp_err_t WifiMonitor::addCorsHeaders(httpd_req_t *req)
    {
        esp_err_t ret = ESP_OK;

        ret |= httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        ret |= httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        ret |= httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

        return ret;
    }

    esp_err_t WifiMonitor::loggingControlHandler(httpd_req_t *req)
    {
        if (!instance_)
            return ESP_ERR_INVALID_STATE;

        // Add CORS headers
        addCorsHeaders(req);

        if (req->method == HTTP_POST)
        {
            // Parse request body for action
            char content[100];
            size_t recv_size = (req->content_len < sizeof(content) - 1) ? req->content_len : sizeof(content) - 1;

            if (httpd_req_recv(req, content, recv_size) <= 0)
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive request body");
                return ESP_ERR_INVALID_ARG;
            }
            content[recv_size] = '\0';

            std::string action(content);
            esp_err_t result = ESP_OK;
            DIGITOYS_LOGD("WifiMonitor", "loggingControl POST action='%s'", action.c_str());

            if (action.find("start") != std::string::npos)
            {
                result = instance_->startLogging();
            }
            else if (action.find("stop") != std::string::npos)
            {
                result = instance_->stopLogging();
            }
            else if (action.find("clear") != std::string::npos)
            {
                if (instance_->data_logger_service_)
                {
                    auto *data_logger = instance_->data_logger_service_->getDataLogger();
                    result = data_logger->clear();
                    DIGITOYS_LOGI("WifiMonitor", "DataLogger cleared");
                }
                else
                {
                    DIGITOYS_LOGE("WifiMonitor", "DataLogger service not available for clear");
                    result = ESP_ERR_INVALID_STATE;
                }
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid action");
                return ESP_ERR_INVALID_ARG;
            }

            httpd_resp_set_type(req, "application/json");
            if (result == ESP_OK)
            {
                const char *response = "{\"status\":\"success\"}";
                httpd_resp_send(req, response, strlen(response));
                DIGITOYS_LOGD("WifiMonitor", "loggingControl '%s' -> OK (logging_active=%d)", action.c_str(), (int)instance_->logging_active_);
            }
            else
            {
                const char *response = "{\"status\":\"error\",\"message\":\"Operation failed\"}";
                httpd_resp_send(req, response, strlen(response));
                DIGITOYS_LOGW("WifiMonitor", "loggingControl '%s' -> ERROR", action.c_str());
            }
        }
        else
        {
            // GET request - return current logging status with complete dashboard info
            httpd_resp_set_type(req, "application/json");

            std::string response = "{";
            bool active = instance_->isLoggingActive();
            response += "\"logging_active\":" + std::string(active ? "true" : "false");

            size_t entry_count = 0;
            size_t memory_usage_bytes = 0;
            float memory_usage_percent = 0.0f;

            if (instance_->data_logger_service_)
            {
                auto *data_logger = instance_->data_logger_service_->getDataLogger();
                entry_count = data_logger->getEntryCount(); // Logged data count
                memory_usage_bytes = data_logger->getMemoryUsage();

                // Calculate memory usage percentage
                if (data_logger->getMaxMemoryKB() > 0)
                {
                    memory_usage_percent = (float)memory_usage_bytes / (data_logger->getMaxMemoryKB() * 1024.0f) * 100.0f;
                }
            }

            response += ",\"entry_count\":" + std::to_string(entry_count);
            response += ",\"memory_usage_bytes\":" + std::to_string(memory_usage_bytes);
            response += ",\"memory_usage_percent\":" + std::to_string(memory_usage_percent);
            response += ",\"data_size_kb\":" + std::to_string(memory_usage_bytes / 1024);
            response += "}";

            httpd_resp_send(req, response.c_str(), response.length());
            DIGITOYS_LOGD("WifiMonitor", "loggingControl GET -> active=%d, entries=%u", (int)active, (unsigned)entry_count);
        }

        return ESP_OK;
    }

    esp_err_t WifiMonitor::loggingDataHandler(httpd_req_t *req)
    {
        if (!instance_)
            return ESP_ERR_INVALID_STATE;

        DIGITOYS_LOGI("WifiMonitor", "loggingDataHandler called");

        // Add CORS headers
        addCorsHeaders(req);

        // Check query parameter for format (default to csv for efficiency)
        char query[100];
        bool use_json = false;
        bool use_wide = false;
        uint32_t rate_hz = 5;
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        {
            char format[10];
            if (httpd_query_key_value(query, "format", format, sizeof(format)) == ESP_OK)
            {
                if (strcmp(format, "json") == 0)
                {
                    use_json = true;
                }
                else if (strcmp(format, "wide") == 0)
                {
                    use_wide = true;
                }
            }
            char rate_buf[10];
            if (httpd_query_key_value(query, "rate_hz", rate_buf, sizeof(rate_buf)) == ESP_OK)
            {
                int r = atoi(rate_buf);
                if (r > 0)
                    rate_hz = (uint32_t)r;
            }
        }

        if (use_json)
        {
            // JSON format
            httpd_resp_set_type(req, "application/json");
            std::string json_data = instance_->getDataLoggerJSON();
            DIGITOYS_LOGI("WifiMonitor", "About to send JSON response, size: %d bytes", json_data.length());

            if (json_data.length() > 80000)
            { // 80KB limit for JSON
                DIGITOYS_LOGW("WifiMonitor", "JSON data too large (%d bytes), sending error", json_data.length());
                const char *error_response = "{\"error\":\"Data too large, use CSV format\",\"entries\":0}";
                httpd_resp_send(req, error_response, strlen(error_response));
            }
            else
            {
                httpd_resp_send(req, json_data.c_str(), json_data.length());
                DIGITOYS_LOGI("WifiMonitor", "JSON response sent successfully");
            }
        }
        else if (use_wide)
        {
            httpd_resp_set_type(req, "text/csv");
            httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"physics_wide.csv\"");

            std::string csv_data = instance_->getDataLoggerWideCSV(rate_hz);
            httpd_resp_send(req, csv_data.c_str(), csv_data.length());
        }
        else
        {
            // CSV format (default, much more compact)
            httpd_resp_set_type(req, "text/csv");
            httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"physics_data.csv\"");

            std::string csv_data = instance_->getDataLoggerCSV();
            DIGITOYS_LOGI("WifiMonitor", "About to send CSV response, size: %d bytes", csv_data.length());

            httpd_resp_send(req, csv_data.c_str(), csv_data.length());
            DIGITOYS_LOGI("WifiMonitor", "CSV response sent successfully");
        }

        return ESP_OK;
    }

    // System log capture methods
    void WifiMonitor::addSystemLogEntry(const std::string &log_line)
    {
        if (system_log_mutex_ && xSemaphoreTake(system_log_mutex_, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            system_log_buffer_.push_back(log_line);

            // Limit buffer size to prevent memory overflow
            if (system_log_buffer_.size() > MAX_SYSTEM_LOG_ENTRIES)
            {
                system_log_buffer_.erase(system_log_buffer_.begin());
            }

            xSemaphoreGive(system_log_mutex_);
        }
    }

    std::vector<std::string> WifiMonitor::getRecentSystemLogs(size_t max_entries) const
    {
        std::vector<std::string> result;

        if (system_log_mutex_ && xSemaphoreTake(system_log_mutex_, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            size_t start_idx = 0;
            if (system_log_buffer_.size() > max_entries)
            {
                start_idx = system_log_buffer_.size() - max_entries;
            }

            for (size_t i = start_idx; i < system_log_buffer_.size(); i++)
            {
                result.push_back(system_log_buffer_[i]);
            }

            xSemaphoreGive(system_log_mutex_);
        }

        return result;
    }

    // Static log hook for capturing all ESP-IDF logs
    int WifiMonitor::logHook(const char *format, va_list args)
    {
        // Format the log message
        char buffer[512];
        int len = vsnprintf(buffer, sizeof(buffer) - 1, format, args);
        buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination

        // Add to system log buffer if instance exists
        if (instance_ && len > 0)
        {
            // Remove trailing newline if present
            std::string log_line(buffer);
            if (!log_line.empty() && log_line.back() == '\n')
            {
                log_line.pop_back();
            }

            // Only capture non-empty, meaningful log lines
            if (!log_line.empty() && log_line.find("I (") != std::string::npos)
            {
                instance_->addSystemLogEntry(log_line);
            }
        }

        // Always call the original printf to maintain normal logging
        return vprintf(format, args);
    }

    // Helper to detect if request is a proper WebSocket handshake
    bool WifiMonitor::isWebSocketFrame(httpd_req_t *req)
    {
        // Check for Upgrade: websocket header
        char upgrade[16];
        if (httpd_req_get_hdr_value_str(req, "Upgrade", upgrade, sizeof(upgrade)) == ESP_OK)
        {
            // Case-insensitive compare
            for (char *p = upgrade; *p; ++p)
                *p = (char)tolower((unsigned char)*p);
            if (strcmp(upgrade, "websocket") == 0)
                return true;
        }
        return false;
    }

} // namespace wifi_monitor
