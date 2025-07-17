#include "LiDAR.hpp"
#include <esp_log.h>
#include <limits>
#include <array>

namespace lidar
{

    static const char *TAG = "LiDAR";

    LiDAR::LiDAR(const LiDARConfig &cfg) : cfg_(cfg) {}

    LiDAR::~LiDAR()
    {
        shutdown();
    }

    esp_err_t LiDAR::initialize()
    {
        esp_err_t err = uart_.init(cfg_);
        if (err != ESP_OK)
            return err;

        err = motor_.init(cfg_);
        if (err != ESP_OK)
            return err;

        err = motor_.start();
        if (err != ESP_OK)
            return err;

        BaseType_t rc = xTaskCreate(taskEntry, "lidar_task", 4096, this,
                                    tskIDLE_PRIORITY + 1, &task_handle_);
        if (rc != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to create LiDAR task");
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    void LiDAR::shutdown()
    {
        if (task_handle_)
        {
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }
        motor_.stop();
        motor_.deinit();
        uart_.deinit();
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

float LiDAR::getDistance() const
{
    std::lock_guard<std::mutex> lg(info_mutex_);
    return info_.distance;
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
            if (pt.intensity < 100)
                continue;
            if (!inRange(pt.angle))
                continue;
            d = pt.distance / 1000.0f;
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
        constexpr size_t BUF_SIZE = 512;
        std::array<std::byte, BUF_SIZE> buffer{};
        while (true)
        {
            std::size_t bytes = 0;
            if (uart_.read(buffer, bytes, 20) == ESP_OK && bytes > 0)
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
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

} // namespace lidar
