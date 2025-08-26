#include "LiDAR.hpp"
#include <Constants.hpp>
#include <esp_log.h>
#include <limits>
#include <array>

namespace lidar
{

    static const char *TAG = "LiDAR";

    LiDAR::LiDAR(const LiDARConfig &cfg)
        : ComponentBase("LiDAR"), cfg_(cfg) {}

    LiDAR::~LiDAR()
    {
        shutdown();
    }

    esp_err_t LiDAR::initialize()
    {
        if (getState() != digitoys::core::ComponentState::UNINITIALIZED)
        {
            ESP_LOGW(TAG, "Component already initialized");
            return ESP_ERR_INVALID_STATE;
        }

        esp_err_t err = uart_.init(cfg_);
        if (err != ESP_OK)
        {
            setState(digitoys::core::ComponentState::ERROR);
            return err;
        }

        err = motor_.init(cfg_);
        if (err != ESP_OK)
        {
            setState(digitoys::core::ComponentState::ERROR);
            return err;
        }

        setState(digitoys::core::ComponentState::INITIALIZED);
        ESP_LOGI(TAG, "LiDAR component initialized successfully");
        return ESP_OK;
    }

    esp_err_t LiDAR::start()
    {
        if (getState() != digitoys::core::ComponentState::INITIALIZED)
        {
            ESP_LOGW(TAG, "Component not initialized or already running");
            return ESP_ERR_INVALID_STATE;
        }

        esp_err_t err = motor_.start();
        if (err != ESP_OK)
        {
            setState(digitoys::core::ComponentState::ERROR);
            return err;
        }

        BaseType_t rc = xTaskCreate(taskEntry, "lidar_task",
                                    digitoys::constants::lidar_const::TASK_STACK_SIZE,
                                    this, digitoys::constants::lidar_const::TASK_PRIORITY,
                                    &task_handle_);
        if (rc != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to create LiDAR task");
            setState(digitoys::core::ComponentState::ERROR);
            return ESP_FAIL;
        }

        setState(digitoys::core::ComponentState::RUNNING);
        ESP_LOGI(TAG, "LiDAR component started successfully");
        return ESP_OK;
    }

    esp_err_t LiDAR::stop()
    {
        if (getState() != digitoys::core::ComponentState::RUNNING)
        {
            ESP_LOGW(TAG, "Component not running");
            return ESP_ERR_INVALID_STATE;
        }

        if (task_handle_)
        {
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }
        motor_.stop();

        setState(digitoys::core::ComponentState::STOPPED);
        ESP_LOGI(TAG, "LiDAR component stopped");
        return ESP_OK;
    }

    esp_err_t LiDAR::shutdown()
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
        motor_.stop();
        motor_.deinit();
        uart_.deinit();

        setState(digitoys::core::ComponentState::UNINITIALIZED);
        ESP_LOGI(TAG, "LiDAR component shutdown complete");
        return ESP_OK;
    }

    void LiDAR::taskEntry(void *arg)
    {
        static_cast<LiDAR *>(arg)->taskLoop();
    }

    ObstacleInfo LiDAR::getObstacleInfo() const
    {
        std::lock_guard<std::mutex> lg(info_mutex_);
        return info_;
    }

    ObstacleInfo LiDAR::evaluateFrame(const Points2D &frame) const
    {
        float closest = std::numeric_limits<float>::infinity();
        auto inRange = [this](float angle)
        {
            if (cfg_.angleMinDeg <= cfg_.angleMaxDeg)
                return angle >= cfg_.angleMinDeg && angle <= cfg_.angleMaxDeg;
            return angle >= cfg_.angleMinDeg || angle <= cfg_.angleMaxDeg;
        };
        float d = std::numeric_limits<float>::infinity();

        for (const auto &pt : frame)
        {
            if (pt.intensity < digitoys::constants::lidar_const::MINIMUM_INTENSITY)
                continue;
            if (!inRange(pt.angle))
                continue;
            d = pt.distance / digitoys::constants::lidar_const::DISTANCE_UNIT_CONVERSION;
            if (d < closest)
                closest = d;
        }

        ObstacleInfo out{};
        out.obstacle = closest <= cfg_.obstacleThreshold;
        out.warning = !out.obstacle && closest <= cfg_.warningThreshold;
        out.distance = closest;
        return out;
    }

    void LiDAR::taskLoop()
    {
        std::array<std::byte, digitoys::constants::lidar_const::READ_BUFFER_SIZE> buffer{};
        while (true)
        {
            std::size_t bytes = 0;
            if (uart_.read(buffer, bytes, digitoys::constants::lidar_const::UART_READ_TIMEOUT_MS) == ESP_OK && bytes > 0)
            {
                parser_.CommReadCallback(reinterpret_cast<const char *>(buffer.data()), bytes);
                if (parser_.IsFrameReady())
                {
                    auto frame = parser_.GetLaserScanData();
                    parser_.ResetFrameReady();
                    ObstacleInfo info = evaluateFrame(frame);
                    {
                        std::lock_guard<std::mutex> lg(info_mutex_);
                        info_ = info;
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(digitoys::constants::lidar_const::TASK_DELAY_MS));
        }
    }

} // namespace lidar
