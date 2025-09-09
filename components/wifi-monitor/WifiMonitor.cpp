#include "WifiMonitor.hpp"
#include "SystemMonitor.hpp"
#include "DataLoggerService.hpp" // Add DataLogger integration
#include "DataLogger.hpp"        // For direct DataLogger method access
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <apps/dhcpserver/dhcpserver.h> // For dhcps_lease_t
#include <nvs_flash.h>
#include <esp_mac.h>
#include <Logger.hpp>
#include <algorithm>
#include <string.h>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <sys/socket.h>
#include <errno.h>
#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <cJSON.h>

namespace wifi_monitor
{
    // Static instance for HTTP handlers
    WifiMonitor *WifiMonitor::instance_ = nullptr;

    esp_err_t WifiMonitor::initialize()
    {
        // Register with centralized logging system
        DIGITOYS_REGISTER_COMPONENT("WifiMonitor", "WIFI_MON");

        DIGITOYS_LOGI("WifiMonitor", "Initializing WiFi Monitor component");

        // Initialize NVS (Required for WiFi)
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("WifiMonitor", "Failed to initialize NVS: %s", esp_err_to_name(ret));
            return ret;
        }
        DIGITOYS_LOGI("WifiMonitor", "NVS initialized successfully");

        // Create mutexes for thread safety
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

        // Initialize telemetry data with timestamp
        telemetry_data_.timestamp = esp_timer_get_time() / 1000; // Convert to milliseconds

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

        // Clear WebSocket clients vector
        websocket_clients_.clear();

        // Clear diagnostic log
        diagnostic_log_.clear();
        logging_active_ = false;

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
        httpd_register_uri_handler(server_, &ws_uri);
        httpd_register_uri_handler(server_, &ws_data_uri);

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
            gap: 15px;
        }
        .legend-item {
            display: flex;
            align-items: center;
            gap: 6px;
            font-size: 12px;
            color: #c9d1d9;
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
        <h2 style="color: var(--accent-green); margin-bottom: 1rem;">Data Logging</h2>
        <div class="log-status">
            Status: <span id="logStatus">Stopped</span> | 
            Entries: <span id="logEntries">0</span> | 
            Size: <span id="logSize">0 KB</span> |
            Rate: <span id="dataRate">0 Hz</span>
        </div>
        <div class="log-controls">
            <button class="btn" id="startBtn">Start Logging</button>
            <button class="btn btn-danger" id="stopBtn">Stop Logging</button>
            <button class="btn btn-warning" id="exportBtn">Export Data</button>
            <button class="btn btn-secondary" id="clearBtn">Clear Data</button>
        </div>
        <div class="log-info">
            <div>Last entry: <span id="lastEntry">Never</span></div>
            <div>Session: <span id="sessionTime">00:00:00</span></div>
            <div>Memory usage: <span id="memoryUsage">0%</span></div>
        </div>
    </div>
    
    <div class="container" id="physicsCharts" style="display: none;">
        <h2 style="color: var(--accent-green); margin-bottom: 1rem;">Real-Time Physics Data</h2>
        <div class="chart-grid">
            <div class="chart-container">
                <h3>Vehicle Speed & Distance</h3>
                <canvas id="speedChart" width="280" height="160"></canvas>
                <div class="chart-legend">
                    <div class="legend-item">
                        <div class="legend-color" style="background-color: #2ea043;"></div>
                        <span>Speed (km/h)</span>
                    </div>
                    <div class="legend-item">
                        <div class="legend-color" style="background-color: #d29922;"></div>
                        <span>Distance (cm)</span>
                    </div>
                </div>
            </div>
            <div class="chart-container">
                <h3>RC Input & Safety</h3>
                <canvas id="safetyChart" width="280" height="160"></canvas>
                <div class="chart-legend">
                    <div class="legend-item">
                        <div class="legend-color" style="background-color: #2ea043;"></div>
                        <span>RC Input (%)</span>
                    </div>
                    <div class="legend-item">
                        <div class="legend-color" style="background-color: #f85149;"></div>
                        <span>Safety Margin (cm)</span>
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

        // File streaming variables
        let dataStreamWs = null;
        let currentLogFile = null;
        let logFileWriter = null;
        let logFileName = '';
        let streamingActive = false;

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
                    
                    // Request next data after 1 second
                    setTimeout(() => {
                        if (ws && ws.readyState === WebSocket.OPEN) {
                            ws.send('get_data');
                        }
                    }, 1000);
                    
                } catch (e) {
                    console.error('Error parsing WebSocket data:', e);
                }
            };
            
            ws.onclose = function(event) {
                console.log('WebSocket closed:', event.code, event.reason);
                wsConnected = false;
                document.getElementById('status').textContent = 'Disconnected - Reconnecting...';
                document.getElementById('status').style.color = 'var(--accent-orange)';
                
                // Start reconnection timer
                if (!reconnectInterval) {
                    reconnectInterval = setInterval(() => {
                        console.log('Attempting WebSocket reconnection...');
                        connectWebSocket();
                    }, 3000);
                }
                
                // Start HTTP fallback after 5 seconds
                setTimeout(() => {
                    if (!wsConnected && !httpFallbackInterval) {
                        console.log('Starting HTTP fallback...');
                        startHttpFallback();
                    }
                }, 5000);
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
            httpFallbackInterval = setInterval(fetchStatsHTTP, 2000); // Slower HTTP polling
            fetchStatsHTTP(); // Initial call
        }

        // Start with WebSocket
        connectWebSocket();
        
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
                if (streamingActive && logFileWriter) {
                    // Write incoming data to file
                    try {
                        const data = JSON.parse(event.data);
                        const csvLine = convertToCSV(data) + '\\n';
                        logFileWriter.write(csvLine);
                    } catch (error) {
                        console.error('Error writing data to file:', error);
                    }
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
                
                // Write CSV header
                const header = 'source,key,value,timestamp_us,type\\n';
                await logFileWriter.write(header);
                
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
            
            // Write CSV header
            const header = 'source,key,value,timestamp_us,type\\n';
            logFileWriter.write(header);
        }
        
        async function stopFileStreaming() {
            if (!streamingActive) return;
            
            streamingActive = false;
            
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
            const source = data.source || '';
            const key = data.key || '';
            const value = data.value || '';
            const timestamp_us = data.timestamp_us || 0;
            const type = data.type || 0;
            
            // Escape any commas in the values
            const escapedSource = source.replace(/,/g, ';');
            const escapedKey = key.replace(/,/g, ';');
            const escapedValue = value.replace(/,/g, ';');
            
            return `${escapedSource},${escapedKey},${escapedValue},${timestamp_us},${type}`;
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
                    
                    // Start periodic data refresh
                    dataRefreshTimer = setInterval(refreshLogData, 2000);
                    
                    // Update button states
                    document.getElementById('startBtn').disabled = true;
                    document.getElementById('stopBtn').disabled = false;
                    
                    // Show physics charts
                    document.getElementById('physicsCharts').style.display = 'block';
                    initPhysicsCharts();
                    
                    // Start file streaming
                    startFileStreaming();
                    
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
                    document.getElementById('dataRate').textContent = '0 Hz';
                    
                    // Check if we have logged data to decide chart visibility
                    updatePhysicsChartsVisibility();
                    
                    // Stop file streaming
                    stopFileStreaming();
                    
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
            
            // Get current logging status and entry count
            fetch('/logging/control')
            .then(response => response.json())
            .then(data => {
                if (data.logging_active) {
                    // Calculate data rate
                    var now = Date.now();
                    var timeDiff = (now - lastUpdateTime) / 1000; // seconds
                    var entryDiff = data.entry_count - lastEntryCount;
                    var dataRate = timeDiff > 0 ? (entryDiff / timeDiff) : 0;
                    
                    updateLogEntries(data.entry_count);
                    updateLogSize(Math.floor(data.entry_count * 0.1)); // Estimate size in KB
                    updateLastEntry();
                    document.getElementById('dataRate').textContent = dataRate.toFixed(1) + ' Hz';
                    document.getElementById('memoryUsage').textContent = Math.min(data.entry_count / 10, 100).toFixed(1) + '%';
                    
                    lastEntryCount = data.entry_count;
                    lastUpdateTime = now;
                    
                    // Fetch latest physics data for visualization
                    updatePhysicsDisplay();
                }
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
        
        function updatePhysicsDisplay() {
            // Simulate physics data (in real hardware, this would fetch from latest DataLogger entries)
            var currentTime = Date.now();
            
            // Simulate realistic vehicle physics data
            var rcInput = Math.sin(currentTime / 2000) * 50 + 50; // 0-100 range
            var distance = Math.abs(Math.sin(currentTime / 3000)) * 200 + 50; // 50-250 cm
            var safetyMargin = Math.max(0, distance - 100); // Safety calculation
            var speed = rcInput * 0.5; // Estimated speed from RC input
            
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
            
            // Update charts
            drawChart(speedChart.ctx, [
                { data: speedData, color: '#2ea043', label: 'Speed' },
                { data: distanceData, color: '#d29922', label: 'Distance' }
            ], { min: 0, max: 250 });
            
            drawChart(safetyChart.ctx, [
                { data: rcInputData, color: '#2ea043', label: 'RC Input' },
                { data: safetyData, color: '#f85149', label: 'Safety' }
            ], { min: 0, max: 150 });
            
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
        
        function drawChart(ctx, datasets, range) {
            const canvas = ctx.canvas;
            const width = canvas.width;
            const height = canvas.height;
            const padding = 40; // Space for Y-axis labels
            const chartWidth = width - padding;
            const chartHeight = height - 20; // Space for bottom padding
            
            // Clear canvas
            ctx.fillStyle = '#000';
            ctx.fillRect(0, 0, width, height);
            
            // Draw Y-axis scale and labels
            ctx.fillStyle = '#c9d1d9';
            ctx.font = '10px Arial';
            ctx.textAlign = 'right';
            ctx.textBaseline = 'middle';
            
            const steps = 5;
            for (let i = 0; i <= steps; i++) {
                const value = range.min + (range.max - range.min) * (i / steps);
                const y = chartHeight - (i / steps) * chartHeight + 10;
                
                // Draw tick mark
                ctx.strokeStyle = '#333';
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.moveTo(padding - 5, y);
                ctx.lineTo(padding, y);
                ctx.stroke();
                
                // Draw label
                ctx.fillText(value.toFixed(0), padding - 8, y);
                
                // Draw grid line
                if (i > 0 && i < steps) {
                    ctx.strokeStyle = '#222';
                    ctx.lineWidth = 1;
                    ctx.beginPath();
                    ctx.moveTo(padding, y);
                    ctx.lineTo(width, y);
                    ctx.stroke();
                }
            }
            
            // Draw vertical grid lines
            ctx.strokeStyle = '#222';
            ctx.lineWidth = 1;
            for (let i = 1; i < 10; i++) {
                const x = padding + (i / 10) * chartWidth;
                ctx.beginPath();
                ctx.moveTo(x, 10);
                ctx.lineTo(x, chartHeight + 10);
                ctx.stroke();
            }
            
            // Draw datasets
            datasets.forEach(dataset => {
                if (dataset.data.length < 2) return;
                
                ctx.strokeStyle = dataset.color;
                ctx.lineWidth = 2;
                ctx.beginPath();
                
                dataset.data.forEach((value, index) => {
                    const x = padding + (index / (maxDataPoints - 1)) * chartWidth;
                    const y = (chartHeight + 10) - ((value - range.min) / (range.max - range.min)) * chartHeight;
                    
                    if (index === 0) {
                        ctx.moveTo(x, y);
                    } else {
                        ctx.lineTo(x, y);
                    }
                });
                
                ctx.stroke();
            });
            
            // Draw chart border
            ctx.strokeStyle = '#555';
            ctx.lineWidth = 1;
            ctx.strokeRect(padding, 10, chartWidth, chartHeight);
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
            DIGITOYS_LOGI("WifiMonitor", "Received WebSocket text: %.*s", ws_pkt.len, (char *)ws_pkt.payload);

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
                DIGITOYS_LOGI("WifiMonitor", "Added data streaming client: %d", sockfd);
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
                    DIGITOYS_LOGI("WifiMonitor", "Removed data streaming client: %d", sockfd);
                }
                xSemaphoreGive(instance_->ws_clients_mutex_);
            }
        }

        if (buf) {
            free(buf);
        }
        return ESP_OK;
    }

    esp_err_t WifiMonitor::broadcastDataEntry(const std::string& data_json)
    {
        if (xSemaphoreTake(ws_clients_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            return ESP_ERR_TIMEOUT;
        }

        // Create WebSocket frame
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)data_json.c_str();
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
        if (!data_logger_service_) {
            DIGITOYS_LOGW("WifiMonitor", "No DataLogger service available for streaming setup");
            return;
        }

        auto* data_logger = data_logger_service_->getDataLogger();
        if (!data_logger) {
            DIGITOYS_LOGW("WifiMonitor", "No DataLogger instance available for streaming setup");
            return;
        }

        // Set up streaming callback
        auto streaming_callback = [this](const digitoys::datalogger::DataEntry& entry, const std::string& source_name) {
            // Convert DataEntry to JSON and broadcast
            cJSON* json = cJSON_CreateObject();
            cJSON_AddStringToObject(json, "source", source_name.c_str());
            cJSON_AddStringToObject(json, "key", entry.key.c_str());
            cJSON_AddStringToObject(json, "value", entry.value.c_str());
            cJSON_AddNumberToObject(json, "timestamp_us", entry.timestamp_us);
            cJSON_AddNumberToObject(json, "type", static_cast<int>(entry.type));

            char* json_string = cJSON_Print(json);
            if (json_string) {
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
        if (data_logger_service_)
        {
            auto *data_logger = data_logger_service_->getDataLogger();
            if (data_logger)
            {
                // Enable streaming mode for real-time data streaming to browser
                esp_err_t ret = data_logger->setStreamingMode(true);
                if (ret != ESP_OK)
                {
                    DIGITOYS_LOGE("WifiMonitor", "Failed to enable DataLogger streaming mode: %s", esp_err_to_name(ret));
                    return ret;
                }

                // Switch from monitoring mode to full logging mode
                ret = data_logger->setMonitoringMode(false);
                if (ret == ESP_OK)
                {
                    DIGITOYS_LOGI("WifiMonitor", "DataLogger switched to full logging mode with streaming enabled");
                }
                else
                {
                    DIGITOYS_LOGE("WifiMonitor", "Failed to switch DataLogger to logging mode: %s", esp_err_to_name(ret));
                }
                return ret;
            }
        }

        DIGITOYS_LOGE("WifiMonitor", "DataLogger service not available");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t WifiMonitor::stopLogging()
    {
        if (data_logger_service_)
        {
            auto *data_logger = data_logger_service_->getDataLogger();
            if (data_logger)
            {
                // Disable streaming mode
                esp_err_t ret = data_logger->setStreamingMode(false);
                if (ret != ESP_OK)
                {
                    DIGITOYS_LOGW("WifiMonitor", "Failed to disable DataLogger streaming mode: %s", esp_err_to_name(ret));
                }

                // Switch from full logging mode back to monitoring mode
                ret = data_logger->setMonitoringMode(true);
                if (ret == ESP_OK)
                {
                    DIGITOYS_LOGI("WifiMonitor", "DataLogger switched back to monitoring mode with streaming disabled");
                }
                else
                {
                    DIGITOYS_LOGE("WifiMonitor", "Failed to switch DataLogger to monitoring mode: %s", esp_err_to_name(ret));
                }
                return ret;
            }
        }

        DIGITOYS_LOGE("WifiMonitor", "DataLogger service not available");
        return ESP_ERR_INVALID_STATE;
    }

    bool WifiMonitor::isLoggingActive() const
    {
        if (data_logger_service_)
        {
            auto *data_logger = data_logger_service_->getDataLogger();
            if (data_logger)
            {
                // Logging is active when DataLogger is running AND not in monitoring mode
                return data_logger->isRunning() && !data_logger->isMonitoringMode();
            }
        }
        return false;
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

        // CSV header
        std::string csv = "timestamp_us,key,value\n";

        for (size_t i = 0; i < collected_data.size(); ++i)
        {
            const auto &entry = collected_data[i];

            csv += std::to_string(entry.timestamp_us) + ",";
            csv += entry.key + ",";
            csv += entry.value + "\n";

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

            if (result == ESP_OK)
            {
                httpd_resp_set_type(req, "application/json");
                const char *response = "{\"status\":\"success\"}";
                httpd_resp_send(req, response, strlen(response));
            }
            else
            {
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Operation failed");
            }
        }
        else
        {
            // GET request - return current logging status with complete dashboard info
            httpd_resp_set_type(req, "application/json");

            std::string response = "{";
            response += "\"logging_active\":" + std::string(instance_->isLoggingActive() ? "true" : "false");

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
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
        {
            char format[10];
            if (httpd_query_key_value(query, "format", format, sizeof(format)) == ESP_OK)
            {
                if (strcmp(format, "json") == 0)
                {
                    use_json = true;
                }
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

} // namespace wifi_monitor
