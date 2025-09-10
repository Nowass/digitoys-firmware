#pragma once

#include "DataModelingTypes.hpp"
#include <esp_err.h>
#include <vector>

namespace digitoys::datamodeling
{
    /**
     * @brief RC car behavior model for pattern analysis and predictions
     * 
     * Analyzes:
     * - RC input vs actual movement correlation
     * - Braking performance patterns
     * - Speed vs stopping distance relationships
     * - Safety margin optimization
     * - Performance trends across sessions
     */
    class BehaviorModel
    {
    public:
        BehaviorModel();
        ~BehaviorModel() = default;

        /**
         * @brief Analyze behavior patterns from a data set
         * @param data_points Vector of behavior data points to analyze
         * @return ESP_OK on success
         */
        esp_err_t analyzeDataSet(const std::vector<BehaviorDataPoint>& data_points);

        /**
         * @brief Get RC input correlation analysis
         * @param correlation Correlation coefficient between RC input and actual speed
         * @param response_time Average response time to RC input changes
         * @param linearity Linearity of RC input response (0.0-1.0)
         */
        void getRCInputCorrelation(float& correlation, float& response_time, float& linearity) const;

        /**
         * @brief Get braking performance analysis
         * @param avg_stopping_distance Average stopping distance across all brake events
         * @param avg_reaction_time Average time from obstacle detection to brake activation
         * @param consistency Braking consistency score (0.0-1.0, higher = more consistent)
         */
        void getBrakingPerformance(float& avg_stopping_distance, float& avg_reaction_time, float& consistency) const;

        /**
         * @brief Get speed vs distance relationship
         * @param speed_bins Speed ranges for analysis
         * @param avg_stopping_distances Average stopping distance for each speed bin
         * @return Number of speed bins analyzed
         */
        uint32_t getSpeedDistanceRelationship(std::vector<float>& speed_bins, std::vector<float>& avg_stopping_distances) const;

        /**
         * @brief Get safety margin analysis
         * @param avg_safety_margin Average safety margin maintained
         * @param min_safety_margin Minimum safety margin observed
         * @param safety_violations Number of times safety margin was breached
         */
        void getSafetyMarginAnalysis(float& avg_safety_margin, float& min_safety_margin, uint32_t& safety_violations) const;

        /**
         * @brief Predict performance for given conditions
         * @param speed Current speed (m/s)
         * @param rc_input RC input strength (0.0-1.0)
         * @param obstacle_distance Distance to obstacle (meters)
         * @return Predicted outcome (stopping distance, safety margin, etc.)
         */
        struct PerformancePrediction
        {
            float predicted_stopping_distance = 0.0f;
            float predicted_reaction_time = 0.0f;
            float predicted_safety_margin = 0.0f;
            float confidence_level = 0.0f; // 0.0-1.0
        };
        PerformancePrediction predictPerformance(float speed, float rc_input, float obstacle_distance) const;

        /**
         * @brief Get model accuracy and validation metrics
         * @param prediction_accuracy Average accuracy of predictions vs actual results
         * @param data_points_analyzed Total data points used for analysis
         * @param brake_events_analyzed Total brake events analyzed
         */
        void getModelAccuracy(float& prediction_accuracy, uint32_t& data_points_analyzed, uint32_t& brake_events_analyzed) const;

        /**
         * @brief Reset model and clear all analysis data
         */
        void resetModel();

    private:
        static const char* TAG;

        // Analysis results
        float rc_correlation_ = 0.0f;
        float avg_response_time_ = 0.0f;
        float rc_linearity_ = 0.0f;
        
        float avg_stopping_distance_ = 0.0f;
        float avg_reaction_time_ = 0.0f;
        float braking_consistency_ = 0.0f;
        
        float avg_safety_margin_ = 0.0f;
        float min_safety_margin_ = 0.0f;
        uint32_t safety_violations_ = 0;
        
        float prediction_accuracy_ = 0.0f;
        uint32_t data_points_analyzed_ = 0;
        uint32_t brake_events_analyzed_ = 0;

        // Speed-distance relationship bins
        struct SpeedBin
        {
            float min_speed = 0.0f;
            float max_speed = 0.0f;
            float avg_stopping_distance = 0.0f;
            uint32_t sample_count = 0;
        };
        std::vector<SpeedBin> speed_bins_;

        /**
         * @brief Analyze RC input correlation
         * @param data_points Data to analyze
         */
        void analyzeRCCorrelation(const std::vector<BehaviorDataPoint>& data_points);

        /**
         * @brief Analyze braking performance
         * @param data_points Data to analyze
         */
        void analyzeBrakingPerformance(const std::vector<BehaviorDataPoint>& data_points);

        /**
         * @brief Analyze speed vs stopping distance relationship
         * @param data_points Data to analyze
         */
        void analyzeSpeedDistanceRelationship(const std::vector<BehaviorDataPoint>& data_points);

        /**
         * @brief Analyze safety margin patterns
         * @param data_points Data to analyze
         */
        void analyzeSafetyMargins(const std::vector<BehaviorDataPoint>& data_points);

        /**
         * @brief Calculate correlation coefficient between two data series
         * @param x First data series
         * @param y Second data series
         * @return Correlation coefficient (-1.0 to 1.0)
         */
        float calculateCorrelation(const std::vector<float>& x, const std::vector<float>& y) const;

        /**
         * @brief Calculate standard deviation of a data series
         * @param data Data series
         * @param mean Mean value of the series
         * @return Standard deviation
         */
        float calculateStandardDeviation(const std::vector<float>& data, float mean) const;
    };

} // namespace digitoys::datamodeling
