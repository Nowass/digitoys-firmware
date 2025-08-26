#include "ControlTask.hpp"
#include <Constants.hpp>
#include <Logger.hpp>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace control
{

    ControlTask::ControlTask(ControlContext *ctx)
        : ComponentBase("ControlTask"), ctx_(ctx)
    {
        // Constructor - all members are initialized with default constructors
    }

    esp_err_t ControlTask::initialize()
    {
        if (getState() != digitoys::core::ComponentState::UNINITIALIZED)
        {
            DIGITOYS_LOGW("ControlTask", "CONTROL", "Component already initialized");
            return ESP_ERR_INVALID_STATE;
        }

        // Validate context
        if (!ctx_ || !ctx_->lidar || !ctx_->pwm_driver || !ctx_->monitor)
        {
            DIGITOYS_LOGE("ControlTask", "CONTROL", "Invalid control context - null pointers");
            setState(digitoys::core::ComponentState::ERROR);
            return ESP_ERR_INVALID_ARG;
        }

        setState(digitoys::core::ComponentState::INITIALIZED);
        DIGITOYS_LOGI("ControlTask", "CONTROL", "Control task component initialized successfully");
        return ESP_OK;
    }

    esp_err_t ControlTask::start()
    {
        if (getState() != digitoys::core::ComponentState::INITIALIZED)
        {
            DIGITOYS_LOGW("ControlTask", "CONTROL", "Component not initialized or already running");
            return ESP_ERR_INVALID_STATE;
        }

        BaseType_t rc = xTaskCreate(
            [](void *arg)
            { static_cast<ControlTask *>(arg)->run(); },
            "ControlTask",
            digitoys::constants::control_task::TASK_STACK_SIZE,
            this,
            digitoys::constants::control_task::TASK_PRIORITY,
            &task_handle_);

        if (rc != pdPASS)
        {
            DIGITOYS_LOGE("ControlTask", "CONTROL", "Failed to create control task");
            setState(digitoys::core::ComponentState::ERROR);
            return ESP_FAIL;
        }

        setState(digitoys::core::ComponentState::RUNNING);
        DIGITOYS_LOGI("ControlTask", "CONTROL", "Control task component started successfully");
        return ESP_OK;
    }

    esp_err_t ControlTask::stop()
    {
        if (getState() != digitoys::core::ComponentState::RUNNING)
        {
            DIGITOYS_LOGW("ControlTask", "CONTROL", "Component not running");
            return ESP_ERR_INVALID_STATE;
        }

        if (task_handle_)
        {
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }

        setState(digitoys::core::ComponentState::STOPPED);
        DIGITOYS_LOGI("ControlTask", "CONTROL", "Control task component stopped");
        return ESP_OK;
    }

    esp_err_t ControlTask::shutdown()
    {
        if (getState() == digitoys::core::ComponentState::RUNNING)
        {
            stop();
        }

        if (task_handle_)
        {
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }

        setState(digitoys::core::ComponentState::UNINITIALIZED);
        DIGITOYS_LOGI("ControlTask", "CONTROL", "Control task component shutdown complete");
        return ESP_OK;
    }

    void ControlTask::run()
    {
        DIGITOYS_LOGI("ControlTask", "CONTROL", "Control task started");

        while (true)
        {
            processFrame();
            vTaskDelay(pdMS_TO_TICKS(digitoys::constants::timing::CONTROL_LOOP_DELAY_MS));
        }
    }

    void ControlTask::processFrame()
    {
        // Get LiDAR obstacle information
        auto lidar_info = ctx_->lidar->getObstacleInfo();

        // Process RC input
        auto rc_status = rc_processor_.processRCInput(*ctx_->pwm_driver);

        // Calculate dynamic thresholds
        float dynamic_brake_distance = obstacle_detector_.calculateBrakeDistance(rc_status.current_input);
        float dynamic_warning_distance = obstacle_detector_.calculateWarningDistance(rc_status.current_input);

        // Override LiDAR's hardcoded thresholds with our dynamic ones
        bool dynamic_obstacle = obstacle_detector_.isDynamicObstacle(lidar_info.distance, rc_status.current_input);
        bool dynamic_warning = obstacle_detector_.isDynamicWarning(lidar_info.distance, rc_status.current_input);

        // Determine action based on current state and inputs
        SpeedController::Action action = SpeedController::Action::NO_ACTION;
        uint32_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds

        if (rc_status.throttle_pressed)
        {
            if (rc_status.wants_reverse)
            {
                action = speed_controller_.handleReverseMotion(state_);
            }
            else if (rc_status.driving_forward && dynamic_obstacle)
            {
                action = speed_controller_.handleObstacleState(state_, rc_status, current_time,
                                                               *ctx_->pwm_driver, rc_processor_);
            }
            else if (rc_status.driving_forward && dynamic_warning)
            {
                action = speed_controller_.handleWarningState(state_, rc_status, lidar_info.distance,
                                                              current_time, *ctx_->pwm_driver, rc_processor_,
                                                              obstacle_detector_);
            }
            else if (rc_status.driving_forward)
            {
                action = speed_controller_.handleClearPath(state_);
            }
            // Neutral position - maintain current state (no action)
        }
        else
        {
            // Throttle not pressed (neutral) - clear any active brake/warning states
            action = speed_controller_.handleNeutralThrottle(state_);
        }

        // Execute the determined action
        speed_controller_.executeAction(action, *ctx_->pwm_driver, state_);

        // Update telemetry and logging
        updateTelemetry(lidar_info, rc_status);
        logDiagnostics(rc_status, dynamic_brake_distance, dynamic_warning_distance, lidar_info.distance);
    }

    void ControlTask::updateTelemetry(const lidar::ObstacleInfo &lidar_info,
                                      const RCInputProcessor::RCStatus &rc_status)
    {
        // Update monitor with current telemetry
        ctx_->monitor->updateTelemetry(lidar_info.obstacle, lidar_info.distance,
                                       ctx_->pwm_driver->lastDuty(0), lidar_info.warning);
    }

    void ControlTask::logDiagnostics(const RCInputProcessor::RCStatus &rc_status,
                                     float dynamic_brake_distance, float dynamic_warning_distance,
                                     float distance)
    {
        // Log RC diagnostics periodically
        rc_processor_.logDiagnostics(rc_status, state_.getDutyTestLogCounter());

        // Debug log dynamic thresholds (every 2 seconds) when driving forward
        if (rc_status.driving_forward && ++state_.getThresholdLogCounter() >= digitoys::constants::control_task::THRESHOLD_LOG_INTERVAL)
        {
            state_.getThresholdLogCounter() = 0;
            DIGITOYS_LOGI("ControlTask", "CONTROL", "Speed: %.4f, BrakeDist: %.2fm, WarnDist: %.2fm, ActualDist: %.2fm",
                     rc_status.current_input, dynamic_brake_distance, dynamic_warning_distance, distance);
        }
    }

} // namespace control

// FreeRTOS task wrapper function
extern "C" void ControlTaskWrapper(void *pv)
{
    auto *task = static_cast<control::ControlTask *>(pv);
    task->run();
}
