#pragma once

namespace digitoys {
namespace core {

    /**
     * @brief Base interface for telemetry monitors
     * Allows components to work with different monitor implementations
     * without tight coupling to specific monitor types.
     */
    class IMonitor
    {
    public:
        virtual ~IMonitor() = default;
        
        /**
         * @brief Update telemetry data
         * @param obstacle True if obstacle is detected
         * @param distance Distance to obstacle in meters
         * @param speed_est Estimated speed
         * @param warning True if warning condition exists
         */
        virtual void updateTelemetry(bool obstacle, float distance, float speed_est, bool warning) = 0;
    };

} // namespace core
} // namespace digitoys
