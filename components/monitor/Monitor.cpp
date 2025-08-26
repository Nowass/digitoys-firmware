#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <Logger.hpp>
#include <string.h>
#include <cmath>

namespace monitor
{
    Monitor *Monitor::instance_ = nullptr;

    esp_err_t Monitor::initialize()
    {
        DIGITOYS_LOGI("Monitor", "MONITOR", "Initializing Monitor component");

        mutex_ = xSemaphoreCreateMutex();
        if (mutex_ == nullptr)
        {
            DIGITOYS_LOGE("Monitor", "MONITOR", "Failed to create telemetry mutex");
            return ESP_ERR_NO_MEM;
        }

        setState(digitoys::core::ComponentState::INITIALIZED);
        return ESP_OK;
    }

    esp_err_t Monitor::start()
    {
        if (getState() != digitoys::core::ComponentState::INITIALIZED && getState() != digitoys::core::ComponentState::STOPPED)
        {
            DIGITOYS_LOGW("Monitor", "MONITOR", "Monitor not in correct state to start");
            return ESP_ERR_INVALID_STATE;
        }

        DIGITOYS_LOGI("Monitor", "MONITOR", "Starting Monitor component");
        instance_ = this;

        esp_err_t ret = init_wifi();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("Monitor", "MONITOR", "Failed to initialize WiFi: %s", esp_err_to_name(ret));
            setState(digitoys::core::ComponentState::ERROR);
            return ret;
        }

        ret = start_http_server();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("Monitor", "MONITOR", "Failed to start HTTP server: %s", esp_err_to_name(ret));
            setState(digitoys::core::ComponentState::ERROR);
            return ret;
        }

        setState(digitoys::core::ComponentState::RUNNING);
        DIGITOYS_LOGI("Monitor", "MONITOR", "Monitor component started successfully");
        return ESP_OK;
    }

    esp_err_t Monitor::stop()
    {
        if (getState() != digitoys::core::ComponentState::RUNNING)
        {
            DIGITOYS_LOGW("Monitor", "MONITOR", "Monitor not running, cannot stop");
            return ESP_ERR_INVALID_STATE;
        }

        DIGITOYS_LOGI("Monitor", "MONITOR", "Stopping Monitor component");

        esp_err_t ret = stop_http_server();
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("Monitor", "MONITOR", "Failed to stop HTTP server: %s", esp_err_to_name(ret));
        }

        if (mutex_)
        {
            vSemaphoreDelete(mutex_);
            mutex_ = nullptr;
        }

        instance_ = nullptr;
        setState(digitoys::core::ComponentState::STOPPED);
        DIGITOYS_LOGI("Monitor", "MONITOR", "Monitor component stopped");
        return ESP_OK;
    }

    esp_err_t Monitor::shutdown()
    {
        if (getState() == digitoys::core::ComponentState::RUNNING)
        {
            esp_err_t ret = stop();
            if (ret != ESP_OK)
            {
                DIGITOYS_LOGW("Monitor", "MONITOR", "Failed to stop during shutdown: %s", esp_err_to_name(ret));
            }
        }

        setState(digitoys::core::ComponentState::UNINITIALIZED);
        DIGITOYS_LOGI("Monitor", "MONITOR", "Monitor component shutdown complete");
        return ESP_OK;
    }

    void Monitor::updateTelemetry(bool obstacle, float distance, float speed_est, bool warning)
    {
        if (mutex_ && xSemaphoreTake(mutex_,
                                     pdMS_TO_TICKS(digitoys::constants::monitor::HTTP_TELEMETRY_TIMEOUT_MS)))
        {
            data_.obstacle = obstacle;
            data_.warning = warning;
            data_.distance = distance;
            data_.speed_est = speed_est;
            xSemaphoreGive(mutex_);
        }
    }

    esp_err_t Monitor::init_wifi()
    {
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        wifi_config_t wifi_config = {};
#ifdef CONFIG_MONITOR_WIFI_SSID
        strncpy(reinterpret_cast<char *>(wifi_config.sta.ssid), CONFIG_MONITOR_WIFI_SSID, sizeof(wifi_config.sta.ssid));
#endif
#ifdef CONFIG_MONITOR_WIFI_PASS
        strncpy(reinterpret_cast<char *>(wifi_config.sta.password), CONFIG_MONITOR_WIFI_PASS, sizeof(wifi_config.sta.password));
#endif
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
#ifdef CONFIG_MONITOR_WIFI_TX_POWER_DBM
        ESP_ERROR_CHECK(
            esp_wifi_set_max_tx_power(CONFIG_MONITOR_WIFI_TX_POWER_DBM * 4));
#endif
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
        ESP_ERROR_CHECK(esp_wifi_connect());
        return ESP_OK;
    }

    esp_err_t Monitor::telemetry_get_handler(httpd_req_t *req)
    {
        Telemetry data{};
        if (instance_->mutex_ && xSemaphoreTake(instance_->mutex_,
                                                pdMS_TO_TICKS(digitoys::constants::monitor::HTTP_TELEMETRY_TIMEOUT_MS)))
        {
            data = instance_->data_;
            xSemaphoreGive(instance_->mutex_);
        }

        // Clamp distance if it's infinity
        float distance = std::isinf(data.distance) ? digitoys::constants::monitor::DEFAULT_DISTANCE_CLAMP : data.distance;

        char resp[digitoys::constants::monitor::HTTP_RESPONSE_BUFFER_SIZE];
        int len = snprintf(resp, sizeof(resp),
                           "{\"obstacle\":%s,\"distance\":%.2f,\"speed\":%.2f,\"warning\":%s}",
                           data.obstacle ? "true" : "false", distance, data.speed_est, data.warning ? "true" : "false");

        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, resp, len);
        return ESP_OK;
    }

    static const char INDEX_HTML[] = R"HTML(
        <!DOCTYPE html>
        <html lang=\"en\">
        <head>
            <meta charset=\"UTF-8\">
            <title>Digitoys Telemetry</title>
            <link rel=\"icon\" href=\"data:,\">
            <style>
                body { font-family: Arial, sans-serif; margin: 2em; }
                #status { font-size: 1.2em; }
            </style>
        </head>
        <body>
            <h1>Digitoys Telemetry</h1>
            <div id=\"status\">Loading...</div>
            <script>
            async function update() {
                try {
                    const res = await fetch('/telemetry');
                    const data = await res.json();
                    document.getElementById('status').textContent =
                        `Obstacle: ${data.obstacle}  Distance: ${data.distance.toFixed(2)} m  Speed: ${data.speed.toFixed(2)} Warning: ${data.warning}`;
                } catch (e) {
                    document.getElementById('status').textContent = 'Error fetching telemetry';
                }
            }
            setInterval(update, 1000);
            update();
            </script>
        </body>
        </html>
        )HTML";

    esp_err_t Monitor::index_get_handler(httpd_req_t *req)
    {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    esp_err_t Monitor::start_http_server()
    {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.stack_size = digitoys::constants::monitor::HTTP_SERVER_STACK_SIZE;
        config.server_port = digitoys::constants::monitor::HTTP_SERVER_PORT;

        esp_err_t ret = httpd_start(&server_, &config);
        if (ret != ESP_OK)
        {
            DIGITOYS_LOGE("Monitor", "MONITOR", "Failed to start HTTP server: %s", esp_err_to_name(ret));
            return ret;
        }

        httpd_uri_t uri = {
            .uri = "/telemetry",
            .method = HTTP_GET,
            .handler = telemetry_get_handler,
            .user_ctx = nullptr};
        ESP_ERROR_CHECK(httpd_register_uri_handler(server_, &uri));

        httpd_uri_t index = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = index_get_handler,
            .user_ctx = nullptr};
        ESP_ERROR_CHECK(httpd_register_uri_handler(server_, &index));

        httpd_uri_t sys_uri = {
            .uri = "/system",
            .method = HTTP_GET,
            .handler = SystemMonitor::stats_get_handler,
            .user_ctx = nullptr};
        ESP_ERROR_CHECK(httpd_register_uri_handler(server_, &sys_uri));
        DIGITOYS_LOGI("Monitor", "MONITOR", "HTTP server started on port %d", digitoys::constants::monitor::HTTP_SERVER_PORT);
        return ESP_OK;
    }

    esp_err_t Monitor::stop_http_server()
    {
        if (server_)
        {
            esp_err_t ret = httpd_stop(server_);
            server_ = nullptr;
            if (ret != ESP_OK)
            {
                DIGITOYS_LOGE("Monitor", "MONITOR", "Failed to stop HTTP server: %s", esp_err_to_name(ret));
                return ret;
            }
            DIGITOYS_LOGI("Monitor", "MONITOR", "HTTP server stopped");
        }
        return ESP_OK;
    }

} // namespace monitor
