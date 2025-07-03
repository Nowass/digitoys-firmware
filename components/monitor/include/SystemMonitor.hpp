#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>

namespace monitor
{
    class SystemMonitor
    {
    public:
        esp_err_t start();
        static esp_err_t stats_get_handler(httpd_req_t *req);

    private:
        static SystemMonitor *instance_;

        uint32_t prev_idle_time_ = 0;
        uint32_t prev_total_time_ = 0;
    };
} // namespace monitor