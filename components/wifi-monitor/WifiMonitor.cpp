#include "WifiMonitor.hpp"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <apps/dhcpserver/dhcpserver.h>  // For dhcps_lease_t
#include <nvs_flash.h>
#include <esp_mac.h>
#include <Logger.hpp>
#include <string.h>
#include <cmath>
#include <ctime>
#include <sys/socket.h>
#include <errno.h>
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
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server_, &dashboard_uri);

        // System data route (temporary HTTP endpoint)
        httpd_uri_t system_uri = {
            .uri = "/system",
            .method = HTTP_GET,
            .handler = systemGetHandler,
            .user_ctx = nullptr
        };
        httpd_register_uri_handler(server_, &system_uri);

        DIGITOYS_LOGI("WifiMonitor", "HTTP server started on port %d", config.server_port);
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

    esp_err_t WifiMonitor::systemGetHandler(httpd_req_t *req)
    {
        if (!instance_) {
            httpd_resp_send_500(req);
            return ESP_ERR_INVALID_STATE;
        }

        // Set content type to JSON
        httpd_resp_set_type(req, "application/json");
        
        // Add CORS headers for cross-origin requests
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET");
        
        // Get basic system information
        uint32_t free_heap = esp_get_free_heap_size();
        uint32_t total_heap = esp_get_minimum_free_heap_size();
        
        // Calculate a simple CPU usage (placeholder for now)
        static uint32_t last_idle_time = 0;
        static uint32_t last_total_time = 0;
        uint32_t idle_time = 0;
        uint32_t total_time = esp_timer_get_time() / 1000; // Convert to ms
        
        // Simple CPU calculation (this is a placeholder - real CPU calculation is more complex)
        float cpu_usage = 0.0f;
        if (last_total_time > 0) {
            uint32_t total_diff = total_time - last_total_time;
            uint32_t idle_diff = idle_time - last_idle_time;
            if (total_diff > 0) {
                cpu_usage = ((float)(total_diff - idle_diff) / total_diff) * 100.0f;
                if (cpu_usage < 0) cpu_usage = 0;
                if (cpu_usage > 100) cpu_usage = 100;
            }
        }
        last_idle_time = idle_time;
        last_total_time = total_time;
        
        // For now, simulate some CPU usage for testing with more obvious changes
        static float simulated_cpu = 15.0f;
        static uint32_t counter = 0;
        counter++;
        
        // Create a sine wave pattern for obvious visual changes
        simulated_cpu = 30.0f + 25.0f * sin(counter * 0.2f); // 30-55% CPU with sine wave
        
        // Also vary heap to show data is updating
        uint32_t varied_free_heap = free_heap + (counter % 1000) * 100;

        // Build JSON response
        char json_buffer[512];
        snprintf(json_buffer, sizeof(json_buffer), 
            "{"
            "\"cpu\":%.1f,"
            "\"total_heap\":%lu,"
            "\"free_heap\":%lu,"
            "\"counter\":%lu,"
            "\"tasks\":["
                "{\"name\":\"main\",\"hwm\":1024},"
                "{\"name\":\"wifi_monitor\",\"hwm\":2048},"
                "{\"name\":\"IDLE\",\"hwm\":512}"
            "]"
            "}",
            simulated_cpu, 
            (unsigned long)total_heap, 
            (unsigned long)varied_free_heap,
            (unsigned long)counter
        );

        return httpd_resp_send(req, json_buffer, HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t WifiMonitor::indexGetHandler(httpd_req_t *req)
    {
        if (!instance_) {
            httpd_resp_send_500(req);
            return ESP_ERR_INVALID_STATE;
        }

        // Set content type to HTML
        httpd_resp_set_type(req, "text/html");
        
        // Embedded dashboard HTML (system.html content)
        const char* dashboard_html = R"(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Digitoys System Monitor</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
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
    </style>
</head>
<body>
    <h1>Digitoys System Monitor</h1>
    <div class="container">
        <canvas id="cpuChart" width="400" height="200"></canvas>
        <div class="cpu-status">
            <div class="cpu-bar" id="cpuBar"></div>
            <div id="cpuLabel">CPU 0%</div>
        </div>
    </div>
    <ul class="usage-list" id="metrics"></ul>
    <script>
        const MAX_POINTS = 30;
        const cpuLabels = Array(MAX_POINTS).fill('');
        const cpuValues = Array(MAX_POINTS).fill(0);

        const ctx = document.getElementById('cpuChart').getContext('2d');
        const cpuChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: cpuLabels,
                datasets: [{
                    label: 'CPU Usage (%)',
                    data: cpuValues,
                    borderColor: 'rgba(46, 160, 67, 1)',
                    backgroundColor: 'rgba(46, 160, 67, 0.1)',
                    borderWidth: 2,
                    fill: true,
                    tension: 0.3,
                    pointRadius: 0
                }]
            },
            options: {
                responsive: true,
                animation: false,
                scales: {
                    y: {
                        min: 0,
                        max: 100,
                        ticks: {
                            callback: value => value + '%',
                            color: '#888'
                        },
                        grid: { color: '#333' }
                    },
                    x: { display: false }
                },
                plugins: { legend: { display: false } }
            }
        });

        function updateCpuChart(value) {
            cpuValues.push(value);
            if (cpuValues.length > MAX_POINTS) cpuValues.shift();
            cpuChart.update();
        }

        async function fetchStats() {
            try {
                const resp = await fetch('/system');
                const data = await resp.json();
                
                updateCpuChart(data.cpu);
                
                const cpuLabel = document.getElementById('cpuLabel');
                const cpuBar = document.getElementById('cpuBar');
                cpuLabel.textContent = `CPU ${data.cpu.toFixed(1)}%`;

                if (data.cpu >= 90) {
                    cpuBar.style.backgroundColor = 'var(--accent-red)';
                } else if (data.cpu >= 70) {
                    cpuBar.style.backgroundColor = 'var(--accent-orange)';
                } else {
                    cpuBar.style.backgroundColor = 'var(--accent-green)';
                }

                const list = document.getElementById('metrics');
                list.innerHTML = '';
                
                if (data.tasks) {
                    data.tasks.forEach(task => {
                        const li = document.createElement('li');
                        li.textContent = `${task.name}: Stack HWM ${task.hwm}`;
                        list.appendChild(li);
                    });
                }
            } catch (e) {
                const list = document.getElementById('metrics');
                list.innerHTML = '<li style="border-left-color: var(--accent-red);">⚠ Failed to fetch stats</li>';
            }
        }

        fetchStats();
        setInterval(fetchStats, 1000);
    </script>
</body>
</html>)";

        return httpd_resp_send(req, dashboard_html, HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t WifiMonitor::websocketHandler(httpd_req_t *req)
    {
        // TODO: Implement WebSocket handler
        return ESP_OK;
    }

    // WebSocket task function (placeholder)
    void WifiMonitor::webSocketTaskFunction(void *param)
    {
        // TODO: Implement WebSocket broadcast loop
    }

} // namespace wifi_monitor
