#pragma once

#include "ControlState.hpp"
#include "ObstacleDetector.hpp"
#include "RCInputProcessor.hpp"
#include "SpeedController.hpp"
#include "adas_pwm_driver.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"

namespace control
{

    /**
     * @brief Control task context - same as original but in namespace
     */
    struct ControlContext
    {
        lidar::LiDAR *lidar;
        adas::PwmDriver *pwm_driver;
        monitor::Monitor *monitor;
    };

    /**
     * @brief Main control task that orchestrates all control logic
     */
    class ControlTask
    {
    public:
        /**
         * @brief Constructor
         * @param ctx Control context with hardware interfaces
         */
        explicit ControlTask(ControlContext *ctx);

        /**
         * @brief Main task execution loop
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
