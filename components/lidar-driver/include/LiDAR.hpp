#pragma once

#include "LiDARConfig.hpp"
#include "uart-hal.hpp"
#include "motor-hal.hpp"
#include "frame-parser.hpp"
#include <ComponentBase.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <mutex>
#include <limits>

namespace lidar
{

    struct ObstacleInfo
    {
        bool obstacle = false;
        bool warning = false;
        float distance = std::numeric_limits<float>::infinity();
    };

    class LiDAR : public digitoys::core::ComponentBase
    {
    public:
        explicit LiDAR(const LiDARConfig &cfg);
        ~LiDAR();

        // IComponent interface
        esp_err_t initialize() override;
        esp_err_t start() override;
        esp_err_t stop() override;
        esp_err_t shutdown() override;

        ObstacleInfo getObstacleInfo() const;

    private:
        static void taskEntry(void *arg);
        void taskLoop();
        ObstacleInfo evaluateFrame(const Points2D &frame) const;

        LiDARConfig cfg_;
        UART_HAL uart_;
        Motor_HAL motor_;
        FramePraser parser_;

        mutable std::mutex info_mutex_;
        ObstacleInfo info_{};
        TaskHandle_t task_handle_ = nullptr;
    };

} // namespace lidar
