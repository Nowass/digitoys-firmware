#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

namespace monitor {
    class SystemMonitor {
    public:
        esp_err_t start();

    private:
        static SystemMonitor *instance_;
        httpd_handle_t server_ = nullptr;

        uint32_t prev_idle_time_ = 0;
        uint32_t prev_total_time_ = 0;

        esp_err_t init_wifi();
        esp_err_t start_http_server();
        static esp_err_t stats_get_handler(httpd_req_t *req);
    };
} // namespace monitor
