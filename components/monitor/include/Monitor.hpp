#pragma once

#include <esp_err.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace monitor
{
    struct Telemetry
    {
        bool obstacle = false;
        bool warning = false;
        float distance = 0.0f;
        float speed_est = 0.0f;
    };

    class Monitor
    {
    public:
        esp_err_t start();
        void updateTelemetry(bool obstacle, float distance, float speed_est, bool warning);

    private:
        esp_err_t init_wifi();
        esp_err_t start_http_server();
        static esp_err_t telemetry_get_handler(httpd_req_t *req);
        static esp_err_t index_get_handler(httpd_req_t *req);

        static Monitor *instance_;
        Telemetry data_{};
        httpd_handle_t server_ = nullptr;
        SemaphoreHandle_t mutex_ = nullptr;
    };

} // namespace monitor
