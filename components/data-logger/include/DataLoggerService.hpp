#pragma once

#include "DataLogger.hpp"
#include "ControlTaskDataSource.hpp"
#include <esp_err.h>
#include <memory>

// Forward declarations
namespace control
{
    class ControlTask;
}

namespace digitoys::datalogger
{

    /**
     * @brief Production-ready DataLogger service for real hardware
     *
     * This service provides a clean integration layer following the standard
     * component pattern (constructor -> initialize() -> start()).
     *
     * Key features:
     * - Standard component lifecycle management
     * - Real ControlTask integration only (no test sources)
     * - Configurable via DataLoggerConfig.hpp
     * - Minimal memory footprint for hardware trials
     */
    class DataLoggerService
    {
    public:
        /**
         * @brief Constructor
         * @param control_task Pointer to the ControlTask for real data collection
         * @param enable_physics_analysis Enable advanced physics analysis (default: true)
         */
        explicit DataLoggerService(control::ControlTask *control_task,
                                   bool enable_physics_analysis = true);

        /**
         * @brief Initialize the data logging service
         * @return ESP_OK on success
         */
        esp_err_t initialize();

        /**
         * @brief Start data collection
         * @return ESP_OK on success
         */
        esp_err_t start();

        /**
         * @brief Stop data collection
         * @return ESP_OK on success
         */
        esp_err_t stop();

        /**
         * @brief Get service status
         * @return true if service is running
         */
        bool isRunning() const;

        /**
         * @brief Print current statistics and physics report
         * Lightweight logging output suitable for hardware debugging
         */
        void printStatus() const;

        /**
         * @brief Cleanup and shutdown the service
         * @return ESP_OK on success
         */
        esp_err_t shutdown();

        /**
         * @brief Get current memory usage
         * @return Memory usage in bytes
         */
        size_t getMemoryUsage() const;

        /**
         * @brief Get access to the underlying DataLogger instance
         * @return Pointer to DataLogger (nullptr if not initialized)
         */
        DataLogger *getDataLogger() const;

    private:
        control::ControlTask *control_task_;
        bool physics_analysis_enabled_;

        std::unique_ptr<DataLogger> logger_;
        std::shared_ptr<ControlTaskDataSource> control_source_;

        bool is_initialized_;
        bool is_running_;
    };

} // namespace digitoys::datalogger
