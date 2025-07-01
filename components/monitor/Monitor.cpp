#include "Monitor.hpp"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <string.h>
#include <cmath>

namespace monitor
{
    static const char *TAG = "Monitor";
    Monitor *Monitor::instance_ = nullptr;

    esp_err_t Monitor::start()
    {
        instance_ = this;
        mutex_ = xSemaphoreCreateMutex();
        ESP_ERROR_CHECK(init_wifi());
        ESP_ERROR_CHECK(start_http_server());
        return ESP_OK;
    }

    void Monitor::updateTelemetry(bool obstacle, float distance, float speed_est)
    {
        if (mutex_ && xSemaphoreTake(mutex_, portMAX_DELAY))
        {
            data_.obstacle = obstacle;
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
        if (instance_->mutex_ && xSemaphoreTake(instance_->mutex_, pdMS_TO_TICKS(100)))
        {
            data = instance_->data_;
            xSemaphoreGive(instance_->mutex_);
        }

        // Clamp distance if it's infinity
        float distance = std::isinf(data.distance) ? 999.99f : data.distance;

        char resp[128];
        int len = snprintf(resp, sizeof(resp),
                           "{\"obstacle\":%s,\"distance\":%.2f,\"speed\":%.2f}",
                           data.obstacle ? "true" : "false", distance, data.speed_est);

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
                        `Obstacle: ${data.obstacle}  Distance: ${data.distance.toFixed(2)} m  Speed: ${data.speed.toFixed(2)}`;
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
        config.stack_size = 4096;
        ESP_ERROR_CHECK(httpd_start(&server_, &config));
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
        ESP_LOGI(TAG, "HTTP server started");
        return ESP_OK;
    }

} // namespace monitor
