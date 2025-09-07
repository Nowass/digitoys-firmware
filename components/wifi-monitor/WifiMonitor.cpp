#include "WifiMonitor.hpp"
#include "SystemMonitor.hpp"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <apps/dhcpserver/dhcpserver.h> // For dhcps_lease_t
#include <nvs_flash.h>
#include <esp_mac.h>
#include <Logger.hpp>
#include <string.h>
#include <cmath>
#include <ctime>
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

        // Clear WebSocket clients vector
        websocket_clients_.clear();

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
        httpd_register_uri_handler(server_, &ws_uri);

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
        <div id="telemetryInfo">Loading telemetry data...</div>
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
            <button class="btn btn-warning" id="exportBtn">Export Data</button>
        </div>
        <div class="log-info">
            <div>Last entry: <span id="lastEntry">Never</span></div>
            <div>Session: <span id="sessionTime">00:00:00</span></div>
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
            const telemetryInfo = document.getElementById('telemetryInfo');
            
            if (!telemetryData) {
                telemetryInfo.innerHTML = '<div style="color: var(--accent-orange);">No telemetry data available</div>';
                return;
            }
            
            let html = '';
            html += `<div style="margin: 0.5rem 0;">Distance: <strong>${telemetryData.distance.toFixed(2)} m</strong></div>`;
            html += `<div style="margin: 0.5rem 0;">Speed: <strong>${telemetryData.speed_est.toFixed(1)} km/h</strong></div>`;
            
            const obstacleColor = telemetryData.obstacle ? 'var(--accent-red)' : 'var(--accent-green)';
            const obstacleText = telemetryData.obstacle ? 'DETECTED' : 'Clear';
            html += `<div style="margin: 0.5rem 0;">Obstacle: <strong style="color: ${obstacleColor};">${obstacleText}</strong></div>`;
            
            const warningColor = telemetryData.warning ? 'var(--accent-orange)' : 'var(--accent-green)';
            const warningText = telemetryData.warning ? 'ACTIVE' : 'Clear';
            html += `<div style="margin: 0.5rem 0;">Warning: <strong style="color: ${warningColor};">${warningText}</strong></div>`;
            
            telemetryInfo.innerHTML = html;
        }

        // WebSocket-first real-time monitoring with HTTP fallback
        // updateCounter already declared above
        let wsConnected = false;
        let ws = null;
        let reconnectInterval = null;
        let httpFallbackInterval = null;

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
        
        // Logging functionality - Phase 2
        var loggingActive = false;
        var logEntries = 0;
        var sessionStartTime = null;
        var sessionTimer = null;
        
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
            loggingActive = true;
            sessionStartTime = new Date();
            
            updateLogStatus('Active');
            updateLogEntries(0);
            updateLogSize(0);
            
            // Start session timer
            sessionTimer = setInterval(updateSessionTime, 1000);
            
            // Update button states
            document.getElementById('startBtn').disabled = true;
            document.getElementById('stopBtn').disabled = false;
            
            // Simulate logging activity (Phase 3 will replace this with real backend calls)
            setTimeout(function() {
                if (loggingActive) {
                    updateLogEntries(logEntries + 1);
                    updateLogSize(Math.floor(logEntries * 0.5));
                    updateLastEntry();
                }
            }, 2000);
        }
        
        function stopLogging() {
            if (!loggingActive) return;
            
            console.log('Stopping data logging...');
            loggingActive = false;
            sessionStartTime = null;
            
            updateLogStatus('Stopped');
            
            // Stop session timer
            if (sessionTimer) {
                clearInterval(sessionTimer);
                sessionTimer = null;
            }
            
            // Update button states
            document.getElementById('startBtn').disabled = false;
            document.getElementById('stopBtn').disabled = true;
            
            document.getElementById('sessionTime').textContent = '00:00:00';
        }
        
        function exportData() {
            console.log('Exporting log data...');
            
            if (logEntries === 0) {
                alert('No data to export. Start logging first.');
                return;
            }
            
            // Phase 3 will implement real export functionality
            alert('Export feature coming in Phase 3! Currently have ' + logEntries + ' entries.');
        }
        
        // Initialize logging UI
        function initLogging() {
            var startBtn = document.getElementById('startBtn');
            var stopBtn = document.getElementById('stopBtn');
            var exportBtn = document.getElementById('exportBtn');
            
            if (startBtn) startBtn.addEventListener('click', startLogging);
            if (stopBtn) stopBtn.addEventListener('click', stopLogging);
            if (exportBtn) exportBtn.addEventListener('click', exportData);
            
            // Initial button states
            if (stopBtn) stopBtn.disabled = true;
            
            console.log('Logging UI initialized');
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

    // WebSocket task function (placeholder for future implementation)
    void WifiMonitor::webSocketTaskFunction(void *param)
    {
        // TODO: Implement periodic WebSocket broadcast to all connected clients
    }

} // namespace wifi_monitor
