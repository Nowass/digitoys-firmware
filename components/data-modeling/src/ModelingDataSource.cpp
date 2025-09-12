#include "ModelingDataSource.hpp"
#include <esp_timer.h>

namespace digitoys::datamodeling
{
    using digitoys::datalogger::DataEntry;

    static inline uint64_t now_us()
    {
        return esp_timer_get_time();
    }

    esp_err_t ModelingDataSource::collectData(std::vector<DataEntry> &entries)
    {
        if (!modeling_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        const auto ts = now_us();

        // Get a fresh behavior point by reading sensors directly
        BehaviorDataPoint dp = modeling_->collectRealTimeData();

        // Minimal essential fields for live monitoring + CSV
        entries.emplace_back("rc_input", dp.rc_control.rc_input, ts);
        entries.emplace_back("obstacle_distance", dp.safety_data.obstacle_distance, ts);
        entries.emplace_back("is_obstacle_state", dp.safety_data.is_obstacle_state, ts);
        entries.emplace_back("is_warning_state", dp.safety_data.is_warning_state, ts);

        // Useful physics and safety distances
        entries.emplace_back("calculated_speed", dp.physics_data.calculated_speed, ts);
        entries.emplace_back("brake_distance", dp.safety_data.brake_distance, ts);
        entries.emplace_back("warning_distance", dp.safety_data.warning_distance, ts);

        return ESP_OK;
    }

} // namespace digitoys::datamodeling
