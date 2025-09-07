#pragma once

#include "ControlState.hpp"
#include "ObstacleDetector.hpp"
#include "RCInputProcessor.hpp"
#include "SpeedController.hpp"
#include "adas_pwm_driver.hpp"
#include "LiDAR.hpp"
#include <ComponentBase.hpp>
#include <IMonitor.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace control
{
    /**
     * @brief Control task context with monitor interface
     */
    struct ControlContext
    {
        lidar::LiDAR *lidar;
        adas::PwmDriver *pwm_driver;
        digitoys::core::IMonitor *monitor;
    };

    /**
     * @brief Main control task that orchestrates all control logic
     */
    class ControlTask : public digitoys::core::ComponentBase
    {
    public:
        /**
         * @brief Constructor
         * @param ctx Control context with hardware interfaces
         */
        explicit ControlTask(ControlContext *ctx);

        // IComponent interface
        esp_err_t initialize() override;
        esp_err_t start() override;
        esp_err_t stop() override;
        esp_err_t shutdown() override;

        /**
         * @brief Main task execution loop (for internal use)
         */
        void run();

    private:
        // Hardware interfaces
        ControlContext *ctx_;

        // Control components
        ControlState state_;
        ObstacleDetector obstacle_detector_;
        RCInputProcessor rc_processor_;
        SpeedController speed_controller_;

        // Task management
        TaskHandle_t task_handle_ = nullptr;

        /**
         * @brief Process a single frame of control logic
         */
        void processFrame();

        /**
         * @brief Update telemetry with current state
         * @param lidar_info LiDAR obstacle information
         * @param rc_status RC input status
         */
        void updateTelemetry(const lidar::ObstacleInfo &lidar_info,
                             const RCInputProcessor::RCStatus &rc_status);

        /**
         * @brief Log diagnostic information periodically
         * @param rc_status RC input status
         * @param dynamic_brake_distance Current brake distance
         * @param dynamic_warning_distance Current warning distance
         * @param distance Current obstacle distance
         */
        void logDiagnostics(const RCInputProcessor::RCStatus &rc_status,
                            float dynamic_brake_distance, float dynamic_warning_distance,
                            float distance);
    };

} // namespace control

/**
 * @brief FreeRTOS task wrapper function
 * @param pv Pointer to ControlTask instance
 */
extern "C" void ControlTaskWrapper(void *pv);
