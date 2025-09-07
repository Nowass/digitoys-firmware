#pragma once

/**
 * @brief Configuration options for DataLogger in production builds
 *
 * These settings can be modified to control DataLogger behavior during
 * hardware trials and production deployment.
 */

namespace digitoys::datalogger::config
{

    // Enable/disable data logging completely
    // Set to false to disable all data logging for production
    constexpr bool ENABLE_DATA_LOGGING = true;

    // Enable/disable physics analysis
    // Physics analysis includes advanced calculations but uses more CPU/memory
    constexpr bool ENABLE_PHYSICS_ANALYSIS = true;

    // Data collection sample rate (milliseconds)
    // Lower values = more frequent data collection = higher CPU usage
    constexpr uint32_t SAMPLE_RATE_MS = 500;

    // Memory allocation for data logging (KB)
    // Adjust based on available RAM and required data retention
    constexpr uint32_t MAX_MEMORY_KB = 64;

    // Maximum number of data entries to store
    // Higher values = longer data retention = more memory usage
    constexpr uint32_t MAX_ENTRIES = 500;

    // Flush interval for data persistence (milliseconds)
    // How often to write data to persistent storage (if implemented)
    constexpr uint32_t FLUSH_INTERVAL_MS = 10000;

    // Minimum vehicle speed threshold for data collection
    // Prevents logging when vehicle is stationary (0.0 to 1.0)
    constexpr float SPEED_THRESHOLD = 0.1f;

    // Status reporting interval (milliseconds)
    // How often to print status information during trials
    constexpr uint32_t STATUS_REPORT_INTERVAL_MS = 60000;

} // namespace digitoys::datalogger::config
