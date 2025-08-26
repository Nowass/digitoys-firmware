#pragma once

#include <cstdint>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>

/**
 * @file digitoys_constants.hpp
 * @brief Centralized constants for the DigiToys firmware
 *
 * This file contains all hardcoded values, magic numbers, and configuration
 * constants used throughout the firmware. By centralizing these values,
 * we ensure consistency and make the code more maintainable.
 */

namespace digitoys
{
    namespace constants
    {

        // =============================================================================
        // PWM and RC Control Constants
        // =============================================================================
        namespace pwm
        {
            // PWM duty cycle values (measured from actual RC system)
            constexpr float NEUTRAL_DUTY = 0.0856f;       // Measured neutral position from RC
            constexpr float BRAKE_DUTY = 0.058f;          // Full brake position
            constexpr float DUTY_STEP = 0.005f;           // Step size for gradual changes
            constexpr float DIRECTION_TOLERANCE = 0.005f; // Forward/reverse detection tolerance

            // PWM signal characteristics
            constexpr uint32_t DEFAULT_FREQUENCY_HZ = 50; // Standard servo frequency (50Hz)
            constexpr uint32_t HIGH_FREQUENCY_HZ = 62;    // High frequency mode for some servos

            // PWM input signal timing (microseconds)
            constexpr uint32_t MIN_PULSE_WIDTH_US = 1000;     // Minimum pulse width (1ms)
            constexpr uint32_t MAX_PULSE_WIDTH_US = 2000;     // Maximum pulse width (2ms)
            constexpr uint32_t NEUTRAL_PULSE_WIDTH_US = 1500; // Neutral position (1.5ms)

            // Throttle detection parameters
            constexpr float THROTTLE_NEUTRAL_CENTER = NEUTRAL_DUTY;
            constexpr float THROTTLE_NEUTRAL_RANGE = 0.008f;        // ±0.8% for neutral band
            constexpr float HIGH_SPEED_DUTY = NEUTRAL_DUTY + 0.03f; // ~3% above neutral
        }

        // =============================================================================
        // Timing Constants
        // =============================================================================
        namespace timing
        {
            // Task execution intervals
            constexpr uint32_t CONTROL_LOOP_MS = 50;       // Main control loop period (20Hz)
            constexpr uint32_t RC_CHECK_INTERVAL_MS = 100; // RC input check during brake/warning
            constexpr uint32_t RC_READ_DELAY_MS = 20;      // Brief delay for fresh RC reading

            // Communication timeouts
            constexpr uint32_t UART_TIMEOUT_MS = 20;   // UART read timeout
            constexpr uint32_t MUTEX_TIMEOUT_MS = 100; // Mutex acquisition timeout
            constexpr uint32_t QUEUE_TIMEOUT_MS = 100; // Queue operations timeout

            // Logging intervals (in control loop cycles)
            constexpr int DUTY_TEST_LOG_INTERVAL = 40;       // 40 * 50ms = 2 seconds
            constexpr int THRESHOLD_LOG_INTERVAL = 40;       // 40 * 50ms = 2 seconds
            constexpr int SYSTEM_MONITOR_INTERVAL_MS = 5000; // System stats every 5 seconds

            // Initialization delays
            constexpr uint32_t COMPONENT_INIT_DELAY_MS = 100; // Delay between component initializations
            constexpr uint32_t MOTOR_STARTUP_DELAY_MS = 300;  // Motor initialization delay
        }

        // =============================================================================
        // Memory and Buffer Constants
        // =============================================================================
        namespace memory
        {
            // Task stack sizes
            constexpr uint32_t DEFAULT_TASK_STACK_SIZE = 4096; // Default FreeRTOS task stack
            constexpr uint32_t CONTROL_TASK_STACK_SIZE = 8192; // Control task (needs more space)
            constexpr uint32_t LIDAR_TASK_STACK_SIZE = 4096;   // LiDAR processing task
            constexpr uint32_t MONITOR_TASK_STACK_SIZE = 4096; // System monitoring task

            // Buffer sizes
            constexpr size_t UART_BUFFER_SIZE = 1024;         // UART communication buffer
            constexpr size_t UART_DMA_BUFFER_SIZE = 2048;     // DMA buffer for high-speed UART
            constexpr size_t RMT_BUFFER_SIZE = 64;            // RMT symbol buffer
            constexpr size_t LIDAR_FRAME_BUFFER_SIZE = 512;   // LiDAR frame processing buffer
            constexpr size_t HTTP_RESPONSE_BUFFER_SIZE = 128; // HTTP response buffer

            // Queue sizes
            constexpr size_t RMT_QUEUE_SIZE = 3;    // RMT event queue
            constexpr size_t MAX_SYSTEM_TASKS = 20; // Maximum tasks for monitoring
        }

        // =============================================================================
        // LiDAR Sensor Constants
        // =============================================================================
        namespace lidar
        {
            // Communication settings
            constexpr uint32_t DEFAULT_BAUD_RATE = 230400;        // Standard LiDAR baud rate
            constexpr uart_port_t DEFAULT_UART_PORT = UART_NUM_1; // Default UART port

            // Data processing
            constexpr uint16_t MIN_INTENSITY_THRESHOLD = 100; // Minimum point intensity
            constexpr float DISTANCE_SCALE_FACTOR = 1000.0f;  // mm to m conversion
            constexpr uint16_t MAX_VALID_DISTANCE_MM = 5000;  // Maximum valid distance (5m)

            // Motor control
            constexpr uint32_t MOTOR_DEFAULT_FREQ_HZ = 50000; // Motor PWM frequency
            constexpr uint8_t MOTOR_DEFAULT_DUTY_PCT = 50;    // Motor default duty cycle

            // Frame parsing
            constexpr uint8_t PKG_HEADER = 0x54;     // Frame header byte
            constexpr uint8_t PKG_VER_LEN = 0x2C;    // Version/length byte
            constexpr int POINTS_PER_PACK = 12;      // Points per LiDAR packet
            constexpr int POINT_FREQUENCY_HZ = 1000; // Point measurement frequency

            // Scan parameters
            constexpr float SCAN_FREQUENCY_HZ = 10.0f;      // LiDAR rotation frequency
            constexpr int INTENSITY_SINGLE_THRESHOLD = 180; // Single point intensity
            constexpr int INTENSITY_LOW_THRESHOLD = 200;    // Low intensity threshold
        }

        // =============================================================================
        // Safety and Control Constants
        // =============================================================================
        namespace safety
        {
            // Dynamic braking distances
            constexpr float MIN_BRAKE_DISTANCE_M = 0.5f;   // Minimum brake distance (safety)
            constexpr float MAX_BRAKE_DISTANCE_M = 3.5f;   // Maximum brake distance
            constexpr float MIN_WARNING_DISTANCE_M = 1.0f; // Minimum warning distance
            constexpr float MAX_WARNING_DISTANCE_M = 5.0f; // Maximum warning distance

            // Default thresholds (fallback values)
            constexpr float DEFAULT_OBSTACLE_THRESHOLD_M = 0.8f; // Default obstacle detection
            constexpr float DEFAULT_WARNING_THRESHOLD_M = 1.2f;  // Default warning threshold

            // Angle ranges (degrees)
            constexpr float DEFAULT_ANGLE_MIN_DEG = 0.0f;    // Minimum scan angle
            constexpr float DEFAULT_ANGLE_MAX_DEG = 360.0f;  // Maximum scan angle (full circle)
            constexpr float FORWARD_SECTOR_MIN_DEG = 257.5f; // Forward sector minimum
            constexpr float FORWARD_SECTOR_MAX_DEG = 282.5f; // Forward sector maximum
        }

        // =============================================================================
        // Network and Monitoring Constants
        // =============================================================================
        namespace network
        {
            // WiFi configuration
            constexpr char DEFAULT_SSID[] = "DigiToys_AP";     // Default access point name
            constexpr char DEFAULT_PASSWORD[] = "digitoys123"; // Default WiFi password
            constexpr uint8_t DEFAULT_CHANNEL = 1;             // Default WiFi channel
            constexpr uint8_t MAX_CONNECTIONS = 4;             // Maximum concurrent connections

            // HTTP server
            constexpr uint16_t HTTP_SERVER_PORT = 80;          // HTTP server port
            constexpr size_t HTTP_SERVER_STACK_SIZE = 4096;    // HTTP server task stack
            constexpr uint32_t HTTP_UPDATE_INTERVAL_MS = 1000; // Dashboard update interval

            // Telemetry
            constexpr float TELEMETRY_INVALID_DISTANCE = 999.99f; // Invalid distance placeholder
        }

        // =============================================================================
        // Hardware Pin Assignments (Default)
        // =============================================================================
        namespace pins
        {
            // Default GPIO assignments (can be overridden in configuration)
            namespace defaults
            {
                // LiDAR
                constexpr gpio_num_t LIDAR_TX_PIN = GPIO_NUM_10;
                constexpr gpio_num_t LIDAR_RX_PIN = GPIO_NUM_11;
                constexpr gpio_num_t LIDAR_MOTOR_PIN = GPIO_NUM_3;

                // PWM/RC
                constexpr gpio_num_t RC_INPUT_PIN = GPIO_NUM_18;
                constexpr gpio_num_t RC_OUTPUT_PIN = GPIO_NUM_19;

                // Status LEDs (if used)
                constexpr gpio_num_t STATUS_LED_PIN = GPIO_NUM_2;
                constexpr gpio_num_t ERROR_LED_PIN = GPIO_NUM_NC;
            }

            // Pin validation
            constexpr gpio_num_t INVALID_GPIO = GPIO_NUM_NC;
        }

        // =============================================================================
        // Task Priorities
        // =============================================================================
        namespace priorities
        {
            // FreeRTOS task priorities (higher number = higher priority)
            constexpr UBaseType_t CONTROL_TASK_PRIORITY = tskIDLE_PRIORITY + 3; // Highest priority
            constexpr UBaseType_t LIDAR_TASK_PRIORITY = tskIDLE_PRIORITY + 2;   // High priority
            constexpr UBaseType_t RMT_TASK_PRIORITY = tskIDLE_PRIORITY + 1;     // Medium priority
            constexpr UBaseType_t MONITOR_TASK_PRIORITY = tskIDLE_PRIORITY + 1; // Medium priority
            constexpr UBaseType_t SYSTEM_TASK_PRIORITY = tskIDLE_PRIORITY;      // Low priority
        }

        // =============================================================================
        // Version Information
        // =============================================================================
        namespace version
        {
            constexpr const char *FIRMWARE_VERSION = "2.0.0";
            constexpr const char *BUILD_DATE = __DATE__;
            constexpr const char *BUILD_TIME = __TIME__;
            constexpr const char *COMPONENT_API_VERSION = "1.0.0";
        }

    }
} // namespace digitoys::constants
