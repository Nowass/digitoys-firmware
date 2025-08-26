#pragma once

#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/ledc.h>

namespace digitoys::constants {

/**
 * @brief PWM-related constants
 */
namespace pwm {
    constexpr float BRAKE_DUTY = 0.058f;           ///< Full brake duty cycle (5.8%)
    constexpr float NEUTRAL_DUTY = 0.0856f;        ///< Neutral/zero speed duty cycle (8.56%)
    constexpr float DUTY_STEP = 0.005f;            ///< PWM duty cycle increment step (0.5%)
    constexpr float DIRECTION_TOLERANCE = 0.005f;   ///< Tolerance for forward/reverse detection
    constexpr float HIGH_SPEED_DUTY = NEUTRAL_DUTY + 0.03f; ///< High speed reference (~3% above neutral)
    
    // Throttle detection parameters
    constexpr float DEFAULT_THROTTLE_CENTER = NEUTRAL_DUTY;  ///< Default throttle center position
    constexpr float DEFAULT_THROTTLE_RANGE = 0.008f;         ///< Default throttle detection range
}

/**
 * @brief Timing-related constants
 */
namespace timing {
    constexpr uint32_t CONTROL_LOOP_DELAY_MS = 50;    ///< Main control loop delay (20Hz)
    constexpr uint32_t RC_CHECK_INTERVAL_MS = 100;    ///< RC input check interval during brake
    constexpr uint32_t RC_READ_DELAY_MS = 20;         ///< Brief delay for fresh RC reading
    constexpr uint32_t UART_TIMEOUT_MS = 20;          ///< UART read timeout
    constexpr uint32_t PWM_READ_TIMEOUT_MS = 100;     ///< PWM input read timeout
    constexpr uint32_t MOTOR_STARTUP_DELAY_MS = 300;  ///< Motor startup delay
}

/**
 * @brief Distance and obstacle detection constants
 */
namespace distances {
    constexpr float MIN_BRAKE_DISTANCE_M = 0.5f;      ///< Minimum brake distance (safety)
    constexpr float MAX_BRAKE_DISTANCE_M = 3.5f;      ///< Maximum brake distance
    constexpr float MIN_WARNING_DISTANCE_M = 1.0f;    ///< Minimum warning distance
    constexpr float MAX_WARNING_DISTANCE_M = 5.0f;    ///< Maximum warning distance
    
    // LiDAR filtering
    constexpr float DEFAULT_OBSTACLE_THRESHOLD_M = 0.8f;  ///< Default obstacle detection threshold
    constexpr float DEFAULT_WARNING_THRESHOLD_M = 1.2f;   ///< Default warning threshold
}

/**
 * @brief Hardware pin definitions
 * 
 * Centralized pin assignments for all hardware interfaces.
 * Update these values to change hardware configuration.
 */
namespace pins {
    // LiDAR pins
    constexpr gpio_num_t LIDAR_TX = GPIO_NUM_10;      ///< LiDAR UART TX pin
    constexpr gpio_num_t LIDAR_RX = GPIO_NUM_11;      ///< LiDAR UART RX pin
    constexpr gpio_num_t LIDAR_MOTOR = GPIO_NUM_3;    ///< LiDAR motor control pin
    
    // PWM controller pins
    constexpr gpio_num_t PWM_RX = GPIO_NUM_18;        ///< PWM input (from RC receiver)
    constexpr gpio_num_t PWM_TX = GPIO_NUM_19;        ///< PWM output (to ESC)
}

/**
 * @brief Hardware configuration constants
 */
namespace hardware {
    // UART configuration
    constexpr uart_port_t LIDAR_UART_PORT = UART_NUM_1;    ///< LiDAR UART port
    constexpr int LIDAR_UART_BAUD = 230400;                ///< LiDAR UART baud rate
    constexpr int LIDAR_DMA_BUFFER_SIZE = 2048;            ///< LiDAR DMA buffer size
    
    // LEDC configuration
    constexpr ledc_channel_t LIDAR_MOTOR_CHANNEL = LEDC_CHANNEL_0;  ///< LiDAR motor PWM channel
    constexpr ledc_channel_t PWM_OUTPUT_CHANNEL = LEDC_CHANNEL_1;   ///< PWM output channel
    constexpr ledc_timer_t LIDAR_MOTOR_TIMER = LEDC_TIMER_0;        ///< LiDAR motor PWM timer
    constexpr ledc_timer_t PWM_OUTPUT_TIMER = LEDC_TIMER_1;         ///< PWM output timer
    
    // Frequencies
    constexpr uint32_t LIDAR_MOTOR_FREQ_HZ = 30000;     ///< LiDAR motor PWM frequency
    constexpr uint32_t PWM_OUTPUT_FREQ_HZ = 62;         ///< PWM output frequency (typical RC frequency)
    
    // Motor control
    constexpr int LIDAR_MOTOR_DUTY_PCT = 50;            ///< LiDAR motor duty cycle percentage
}

/**
 * @brief LiDAR-specific constants
 */
namespace lidar_const {
    // Scan angle configuration (for obstacle detection sector)
    constexpr float DEFAULT_ANGLE_MIN_DEG = 257.5f;     ///< Default minimum scan angle
    constexpr float DEFAULT_ANGLE_MAX_DEG = 282.5f;     ///< Default maximum scan angle
    
    // Frame parsing constants
    constexpr int PKG_HEADER = 0x54;                    ///< LiDAR frame header
    constexpr int PKG_VER_LEN = 0x2C;                   ///< LiDAR frame version/length
    constexpr int POINT_PER_PACK = 12;                  ///< Points per LiDAR frame
    constexpr int POINT_FREQUENCY = 4500;               ///< Point sampling frequency
    
    // Filtering constants
    constexpr float SCAN_FREQUENCY_HZ = 10.0f;          ///< LiDAR scan frequency
    constexpr int INTENSITY_SINGLE_THRESHOLD = 180;     ///< Single point intensity threshold
    constexpr int INTENSITY_LOW_THRESHOLD = 200;        ///< Low intensity threshold
}

/**
 * @brief System limits and constraints
 */
namespace limits {
    constexpr size_t MAX_COMPONENTS = 16;               ///< Maximum number of components
    constexpr size_t MAX_COMPONENT_NAME_LEN = 32;       ///< Maximum component name length
    constexpr uint32_t WATCHDOG_TIMEOUT_MS = 5000;     ///< System watchdog timeout
    constexpr size_t TASK_STACK_SIZE_DEFAULT = 8192;    ///< Default task stack size
}

} // namespace digitoys::constants
