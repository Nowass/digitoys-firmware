#pragma once
#include "MPU6050.hpp"
#include "I2C.hpp"
#include <esp_err.h>

namespace MPU6050Example
{

    class Example
    {
    public:
        Example();

        /// Initialize the MPU6050 sensor
        esp_err_t initialize();

        /// Perform self-test
        esp_err_t self_test();

        /// Calibrate sensor (gyroscope bias and temperature offset)
        esp_err_t calibrate();

        /// Read and log sensor data
        esp_err_t read_and_log_data();

        /// Get the latest sensor data
        bool getLatestData(MPU6050::SensorData &data);

        /// Get current acceleration (m/s²)
        float getCurrentAcceleration() const { return currentAcceleration_; }

        /// Get raw X-axis acceleration value (ADC units)
        int16_t getRawAccelX() const { return rawAccelX_; }

        /// Get current velocity (m/s) - simplified calculation
        float getCurrentVelocity() const { return currentVelocity_; }

        /// Get current speed (km/h)
        float getCurrentSpeed() const { return abs(currentVelocity_) * 3.6f; }

        /// Check if impact/collision is detected
        bool isImpactDetected() const { return impactDetected_; }

        /// Check if hard braking is detected
        bool isHardBrakingDetected() const { return hardBrakingDetected_; }

        /// Get peak acceleration in current event
        float getPeakAcceleration() const { return peakAcceleration_; }

        /// Reset impact detection
        void resetImpactDetection();

        /// Reset velocity calculation (e.g., when car stops)
        void resetVelocity();

        /// Show velocity calculation details for debugging
        void logVelocityCalculation() const;

        /// Check if sensor is ready
        bool isReady() const { return initialized_; }

    private:
        I2C i2c_;
        MPU6050 sensor_;
        MPU6050::SensorData lastData_;
        bool initialized_;

        // Calibration offsets
        float gyroOffsetX_;
        float gyroOffsetY_;
        float gyroOffsetZ_;
        float accelOffsetX_;
        float accelOffsetY_;
        float accelOffsetZ_;
        float tempOffset_;
        bool calibrated_;

        // Automotive motion detection variables
        float currentAcceleration_; // Current filtered acceleration (m/s²)
        float currentVelocity_;     // Current velocity (m/s)
        int16_t rawAccelX_;         // Raw X-axis acceleration (ADC units)
        uint32_t lastUpdateTime_;   // Last update timestamp (ms)

        // Adaptive bias correction
        uint32_t stationaryTime_;  // Time sensor has been stationary (ms)
        float stationaryAccelSum_; // Sum of accelerations during stationary period
        int stationaryCount_;      // Count of readings during stationary period

        // Impact and sudden movement detection
        bool impactDetected_;      // Sudden impact detected flag
        bool hardBrakingDetected_; // Hard braking detected flag
        float peakAcceleration_;   // Peak acceleration in current event
        uint32_t impactTime_;      // Time when impact was detected

        // Configuration constants for automotive front assist
        static constexpr float IMPACT_THRESHOLD = 8.0f;     // m/s² sudden impact detection
        static constexpr float HARD_BRAKE_THRESHOLD = 4.0f; // m/s² hard braking detection
        static constexpr float ACCEL_FILTER_ALPHA = 0.2f;   // Light filtering for readable data
        static constexpr float VELOCITY_DECAY_RATE = 0.95f; // Velocity decay when no acceleration
        static constexpr float MIN_ACCEL_THRESHOLD = 0.3f;  // Minimum acceleration to integrate (m/s²)
        static constexpr float MAX_VELOCITY = 20.0f;        // Maximum believable velocity (m/s, ~72 km/h)

        // Adaptive bias correction constants
        static constexpr float STATIONARY_THRESHOLD = 0.1f; // m/s² - consider stationary below this
        static constexpr uint32_t BIAS_UPDATE_TIME = 5000;  // ms - update bias after 5s stationary
        static constexpr int BIAS_UPDATE_SAMPLES = 50;      // Number of samples for bias update

        // MPU6050 conversion constants for ±2g range
        static constexpr float ACCEL_SCALE_FACTOR = 9.81f / 16384.0f; // Convert raw to m/s²

        static constexpr gpio_num_t SDA_PIN = GPIO_NUM_4;
        static constexpr gpio_num_t SCL_PIN = GPIO_NUM_5;
        static constexpr uint32_t I2C_FREQ = 400000; // 400kHz

        // Apply calibration to sensor data
        void applyCalibratedData(MPU6050::SensorData &data);

        // Update speed and braking calculations
        void updateMotionDetection(const MPU6050::SensorData &data);
    };

} // namespace MPU6050Example
