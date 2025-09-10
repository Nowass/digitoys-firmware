#pragma once

#include "DataModelingTypes.hpp"
#include <esp_err.h>
#include <vector>

namespace digitoys::datamodeling
{
    /**
     * @brief Advanced physics analysis for RC car behavior
     * 
     * Performs:
     * - Speed calculation from distance deltas
     * - Braking distance analysis
     * - Deceleration calculations
     * - Physics model validation
     * - Performance predictions
     */
    class PhysicsAnalyzer
    {
    public:
        PhysicsAnalyzer();
        ~PhysicsAnalyzer() = default;

        /**
         * @brief Analyze and enhance a behavior data point with physics calculations
         * @param data_point Data point to analyze (modified in-place)
         * @param sample_rate_ms Sampling rate for calculations
         * @return ESP_OK on success
         */
        esp_err_t analyzeDataPoint(BehaviorDataPoint& data_point, uint32_t sample_rate_ms);

        /**
         * @brief Calculate speed from distance delta and sample rate
         * @param distance_delta Change in distance (meters)
         * @param sample_rate_ms Time between samples (milliseconds)
         * @return Calculated speed (m/s)
         */
        float calculateSpeed(float distance_delta, uint32_t sample_rate_ms);

        /**
         * @brief Analyze braking performance for a data point
         * @param data_point Current data point
         * @return ESP_OK on success
         */
        esp_err_t analyzeBrakingPerformance(BehaviorDataPoint& data_point);

        /**
         * @brief Validate physics calculations against real measurements
         * @param data_point Data point with both calculated and measured values
         * @return Accuracy ratio (1.0 = perfect, <1.0 = underestimate, >1.0 = overestimate)
         */
        float validatePhysicsAccuracy(const BehaviorDataPoint& data_point);

        /**
         * @brief Predict stopping distance based on current conditions
         * @param current_speed Current speed (m/s)
         * @param rc_input RC brake input strength (0.0-1.0)
         * @return Predicted stopping distance (meters)
         */
        float predictStoppingDistance(float current_speed, float rc_input);

        /**
         * @brief Get physics analysis statistics
         * @param avg_accuracy Average accuracy across all calculations
         * @param total_analyses Total number of analyses performed
         * @param brake_analyses Number of braking analyses
         */
        void getAnalysisStats(float& avg_accuracy, uint32_t& total_analyses, uint32_t& brake_analyses) const;

        /**
         * @brief Reset analysis statistics
         */
        void resetStats();

        /**
         * @brief Set physics model parameters for calibration
         * @param brake_coefficient Braking effectiveness coefficient
         * @param friction_coefficient Surface friction coefficient
         * @param reaction_time_ms Average reaction time in milliseconds
         */
        void setModelParameters(float brake_coefficient, float friction_coefficient, uint32_t reaction_time_ms);

    private:
        static const char* TAG;

        // Previous data point for delta calculations
        BehaviorDataPoint previous_data_point_;
        bool has_previous_data_ = false;

        // Physics model parameters
        float brake_coefficient_ = 0.8f;      // Braking effectiveness (0.0-1.0)
        float friction_coefficient_ = 0.7f;   // Surface friction (0.0-1.0)
        uint32_t reaction_time_ms_ = 100;     // Average reaction time

        // Analysis statistics
        float total_accuracy_sum_ = 0.0f;
        uint32_t total_analyses_ = 0;
        uint32_t brake_analyses_ = 0;

        // Braking event tracking
        bool in_braking_event_ = false;
        uint64_t brake_start_time_ = 0;
        float brake_start_distance_ = 0.0f;
        float brake_start_speed_ = 0.0f;

        /**
         * @brief Calculate deceleration from speed change
         * @param speed_delta Change in speed (m/s)
         * @param time_delta Time between measurements (seconds)
         * @return Deceleration (m/s²)
         */
        float calculateDeceleration(float speed_delta, float time_delta);

        /**
         * @brief Estimate time to impact at current conditions
         * @param current_speed Current speed (m/s)
         * @param distance_to_obstacle Distance to obstacle (meters)
         * @param current_deceleration Current deceleration (m/s²)
         * @return Time to impact (seconds), or -1.0 if no collision predicted
         */
        float calculateTimeToImpact(float current_speed, float distance_to_obstacle, float current_deceleration);

        /**
         * @brief Detect start/end of braking events
         * @param data_point Current data point
         */
        void updateBrakingEventState(BehaviorDataPoint& data_point);
    };

} // namespace digitoys::datamodeling
