#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdint.h>
#include <ComponentBase.hpp>
#include <Constants.hpp>

namespace monitor
{
    class SystemMonitor : public digitoys::core::ComponentBase
    {
    public:
        SystemMonitor() : ComponentBase("SystemMonitor") {}
        ~SystemMonitor() override = default;

        // Disable copy constructor and assignment operator
        SystemMonitor(const SystemMonitor &) = delete;
        SystemMonitor &operator=(const SystemMonitor &) = delete;

        // IComponent interface implementation
        esp_err_t initialize() override;
        esp_err_t start() override;
        esp_err_t stop() override;
        esp_err_t shutdown() override;

        static esp_err_t stats_get_handler(httpd_req_t *req);

    private:
        static SystemMonitor *instance_;

        uint32_t prev_idle_time_ = 0;
        uint32_t prev_total_time_ = 0;
    };
} // namespace monitor