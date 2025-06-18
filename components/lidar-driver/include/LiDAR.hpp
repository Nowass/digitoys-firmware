/**
 * @file LiDAR.hpp
 * @brief High level LiDAR driver that publishes obstacle information.
 */
#pragma once

#include "LiDARConfig.hpp"
#include "uart-hal.hpp"
#include "motor-hal.hpp"
#include "frame-parser.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <mutex>

namespace lidar {

/// Result of obstacle detection
struct ObstacleInfo {
    bool obstacle = false; ///< true if object is within obstacleThreshold
    bool warning = false;  ///< true if within warningThreshold
};

/**
 * @brief High level task that reads LiDAR data and publishes obstacle info.
 */
class LiDAR {
public:
    /// Construct driver with configuration
    explicit LiDAR(const LiDARConfig &cfg);
    ~LiDAR();

    /// Initialize hardware and start the processing task
    esp_err_t initialize();
    /// Stop task and deinitialize hardware
    void shutdown();

    /// Thread-safe accessor for the latest obstacle information
    ObstacleInfo getObstacleInfo() const;

private:
    /// FreeRTOS entry trampoline
    static void taskEntry(void *arg);
    /// Main loop reading data from the sensor
    void taskLoop();
    /// Analyse a complete frame and return obstacle info
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
