#include "SystemMonitor.hpp"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <string.h>
#include <cJSON.h>

namespace monitor
{
    static const char *TAG = "SystemMonitor";
    SystemMonitor *SystemMonitor::instance_ = nullptr;

    esp_err_t SystemMonitor::start()
    {
        instance_ = this;
        return ESP_OK;
    }

    static float calculate_cpu_load(uint32_t &prev_idle, uint32_t &prev_total)
    {
        const int MAX_TASKS = 20;
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

    esp_err_t SystemMonitor::stats_get_handler(httpd_req_t *req)
    {
        float cpu = calculate_cpu_load(instance_->prev_idle_time_, instance_->prev_total_time_);
        size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
        size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);

        const int MAX_TASKS = 20;
        TaskStatus_t status[MAX_TASKS];
        uint32_t total_time = 0;
        UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, &total_time);

        // Create root JSON object
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "cpu", cpu);
        cJSON_AddNumberToObject(root, "total_heap", (double)total_heap);
        cJSON_AddNumberToObject(root, "free_heap", (double)free_heap);

        cJSON *tasks = cJSON_CreateArray();
        for (UBaseType_t i = 0; i < count; ++i)
        {
            cJSON *task = cJSON_CreateObject();
            cJSON_AddStringToObject(task, "name", status[i].pcTaskName);
            cJSON_AddNumberToObject(task, "hwm", status[i].usStackHighWaterMark);
            cJSON_AddItemToArray(tasks, task);
        }
        cJSON_AddItemToObject(root, "tasks", tasks);

        // Convert to string
        char *resp = cJSON_PrintUnformatted(root); // Use _Unformatted to keep output compact
        cJSON_Delete(root);

        if (resp)
        {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_send(req, resp, strlen(resp));
            free(resp);
            return ESP_OK;
        }
        else
        {
            return ESP_FAIL;
        }
    }

} // namespace monitor