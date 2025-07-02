#include "SystemMonitor.hpp"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <string.h>

namespace monitor {
    static const char *TAG = "SystemMonitor";
    SystemMonitor *SystemMonitor::instance_ = nullptr;

    esp_err_t SystemMonitor::start() {
        instance_ = this;
        ESP_ERROR_CHECK(init_wifi());
        ESP_ERROR_CHECK(start_http_server());
        return ESP_OK;
    }

    esp_err_t SystemMonitor::init_wifi() {
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
        ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(CONFIG_MONITOR_WIFI_TX_POWER_DBM * 4));
#endif
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
        ESP_ERROR_CHECK(esp_wifi_connect());
        return ESP_OK;
    }

    static float calculate_cpu_load(uint32_t &prev_idle, uint32_t &prev_total) {
        const int MAX_TASKS = 20;
        TaskStatus_t status[MAX_TASKS];
        uint32_t total_time = 0;
        UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, &total_time);
        uint32_t idle_time = 0;
        for (UBaseType_t i = 0; i < count; ++i) {
            if (strstr(status[i].pcTaskName, "IDLE") != nullptr) {
                idle_time += status[i].ulRunTimeCounter;
            }
        }
        float load = 0.0f;
        if (prev_total != 0 && total_time > prev_total) {
            uint32_t diff_total = total_time - prev_total;
            uint32_t diff_idle = idle_time - prev_idle;
            load = 100.0f * (1.0f - (float)diff_idle / (float)diff_total);
        }
        prev_total = total_time;
        prev_idle = idle_time;
        return load;
    }

    esp_err_t SystemMonitor::stats_get_handler(httpd_req_t *req) {
        float cpu = calculate_cpu_load(instance_->prev_idle_time_, instance_->prev_total_time_);
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

        const int MAX_TASKS = 20;
        TaskStatus_t status[MAX_TASKS];
        uint32_t total_time = 0;
        UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, &total_time);

        char tasks[256];
        int off = 0;
        off += snprintf(tasks + off, sizeof(tasks) - off, "[");
        for (UBaseType_t i = 0; i < count; ++i) {
            off += snprintf(tasks + off, sizeof(tasks) - off,
                            "{\"name\":\"%s\",\"hwm\":%u}%s",
                            status[i].pcTaskName, status[i].usStackHighWaterMark,
                            (i + 1 == count) ? "" : ",");
            if (off >= (int)sizeof(tasks) - 1) {
                break;
            }
        }
        off += snprintf(tasks + off, sizeof(tasks) - off, "]");

        char resp[512];
        int len = snprintf(resp, sizeof(resp),
                           "{\"cpu\":%.2f,\"total_heap\":%u,\"free_heap\":%u,\"tasks\":%s}",
                           cpu, (unsigned)total_heap, (unsigned)free_heap, tasks);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, resp, len);
        return ESP_OK;
    }

    esp_err_t SystemMonitor::start_http_server() {
        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
        config.stack_size = 4096;
        ESP_ERROR_CHECK(httpd_start(&server_, &config));

        httpd_uri_t uri = {
            .uri = "/system",
            .method = HTTP_GET,
            .handler = stats_get_handler,
            .user_ctx = nullptr};
        ESP_ERROR_CHECK(httpd_register_uri_handler(server_, &uri));
        ESP_LOGI(TAG, "HTTP server started");
        return ESP_OK;
    }

} // namespace monitor
