#include "BehaviorModel.hpp"
#include <esp_log.h>
#include <cmath>
#include <algorithm>

namespace digitoys::datamodeling
{
    const char* BehaviorModel::TAG = "BehaviorModel";

    BehaviorModel::BehaviorModel()
    {
        // Initialize speed bins (0-0.5 m/s, 0.5-1.0 m/s, 1.0-1.5 m/s, 1.5+ m/s)
        speed_bins_.push_back({0.0f, 0.5f, 0.0f, 0});
        speed_bins_.push_back({0.5f, 1.0f, 0.0f, 0});
        speed_bins_.push_back({1.0f, 1.5f, 0.0f, 0});
        speed_bins_.push_back({1.5f, 999.0f, 0.0f, 0});
        
        ESP_LOGI(TAG, "BehaviorModel initialized with %d speed bins", speed_bins_.size());
    }

    esp_err_t BehaviorModel::analyzeDataSet(const std::vector<BehaviorDataPoint>& data_points)
    {
        if (data_points.empty())
        {
            ESP_LOGW(TAG, "No data points to analyze");
            return ESP_ERR_INVALID_ARG;
        }

        // Reset previous analysis
        resetModel();

        // Perform all analyses
        analyzeRCCorrelation(data_points);
        analyzeBrakingPerformance(data_points);
        analyzeSpeedDistanceRelationship(data_points);
        analyzeSafetyMargins(data_points);

        data_points_analyzed_ = data_points.size();
        
        ESP_LOGI(TAG, "Analyzed %d data points", data_points_analyzed_);
        return ESP_OK;
    }

    void BehaviorModel::getRCInputCorrelation(float& correlation, float& response_time, float& linearity) const
    {
        correlation = rc_correlation_;
        response_time = avg_response_time_;
        linearity = rc_linearity_;
    }

    void BehaviorModel::getBrakingPerformance(float& avg_stopping_distance, float& avg_reaction_time, float& consistency) const
    {
        avg_stopping_distance = avg_stopping_distance_;
        avg_reaction_time = avg_reaction_time_;
        consistency = braking_consistency_;
    }

    uint32_t BehaviorModel::getSpeedDistanceRelationship(std::vector<float>& speed_bins, std::vector<float>& avg_stopping_distances) const
    {
        speed_bins.clear();
        avg_stopping_distances.clear();

        for (const auto& bin : speed_bins_)
        {
            if (bin.sample_count > 0)
            {
                speed_bins.push_back((bin.min_speed + bin.max_speed) / 2.0f); // Bin center
                avg_stopping_distances.push_back(bin.avg_stopping_distance);
            }
        }

        return speed_bins.size();
    }

    void BehaviorModel::getSafetyMarginAnalysis(float& avg_safety_margin, float& min_safety_margin, uint32_t& safety_violations) const
    {
        avg_safety_margin = avg_safety_margin_;
        min_safety_margin = min_safety_margin_;
        safety_violations = safety_violations_;
    }

    BehaviorModel::PerformancePrediction BehaviorModel::predictPerformance(float speed, float rc_input, float obstacle_distance) const
    {
        PerformancePrediction prediction;

        // Find appropriate speed bin for prediction
        for (const auto& bin : speed_bins_)
        {
            if (speed >= bin.min_speed && speed < bin.max_speed && bin.sample_count > 0)
            {
                prediction.predicted_stopping_distance = bin.avg_stopping_distance;
                prediction.confidence_level = std::min(1.0f, bin.sample_count / 10.0f); // Higher confidence with more samples
                break;
            }
        }

        // Predict reaction time based on historical average
        prediction.predicted_reaction_time = avg_reaction_time_;

        // Predict safety margin
        prediction.predicted_safety_margin = obstacle_distance - prediction.predicted_stopping_distance;

        return prediction;
    }

    void BehaviorModel::getModelAccuracy(float& prediction_accuracy, uint32_t& data_points_analyzed, uint32_t& brake_events_analyzed) const
    {
        prediction_accuracy = prediction_accuracy_;
        data_points_analyzed = data_points_analyzed_;
        brake_events_analyzed = brake_events_analyzed_;
    }

    void BehaviorModel::resetModel()
    {
        rc_correlation_ = 0.0f;
        avg_response_time_ = 0.0f;
        rc_linearity_ = 0.0f;
        
        avg_stopping_distance_ = 0.0f;
        avg_reaction_time_ = 0.0f;
        braking_consistency_ = 0.0f;
        
        avg_safety_margin_ = 0.0f;
        min_safety_margin_ = 999.0f;
        safety_violations_ = 0;
        
        prediction_accuracy_ = 0.0f;
        data_points_analyzed_ = 0;
        brake_events_analyzed_ = 0;

        // Reset speed bins
        for (auto& bin : speed_bins_)
        {
            bin.avg_stopping_distance = 0.0f;
            bin.sample_count = 0;
        }
    }

    void BehaviorModel::analyzeRCCorrelation(const std::vector<BehaviorDataPoint>& data_points)
    {
        std::vector<float> rc_inputs;
        std::vector<float> speeds;

        for (const auto& point : data_points)
        {
            if (point.physics_data.calculated_speed > 0.1f) // Only consider when moving
            {
                rc_inputs.push_back(point.rc_control.rc_input);
                speeds.push_back(point.physics_data.calculated_speed);
            }
        }

        if (rc_inputs.size() > 1)
        {
            rc_correlation_ = calculateCorrelation(rc_inputs, speeds);
            
            // Calculate average response time (simplified)
            float total_response_time = 0.0f;
            uint32_t response_samples = 0;
            
            for (size_t i = 1; i < data_points.size(); i++)
            {
                if (data_points[i].rc_control.rc_input != data_points[i-1].rc_control.rc_input)
                {
                    // RC input changed, measure response time
                    total_response_time += 200.0f; // Simplified: assume 200ms average response
                    response_samples++;
                }
            }
            
            avg_response_time_ = (response_samples > 0) ? (total_response_time / response_samples) : 0.0f;
            
            // Calculate linearity (how well RC input correlates with speed change)
            rc_linearity_ = std::abs(rc_correlation_);
        }
    }

    void BehaviorModel::analyzeBrakingPerformance(const std::vector<BehaviorDataPoint>& data_points)
    {
        std::vector<float> stopping_distances;
        std::vector<float> reaction_times;

        for (const auto& point : data_points)
        {
            if (point.physics_data.actual_stopping_distance > 0.0f)
            {
                stopping_distances.push_back(point.physics_data.actual_stopping_distance);
                brake_events_analyzed_++;
            }
            
            if (point.physics_data.stopping_time > 0.0f)
            {
                reaction_times.push_back(point.physics_data.stopping_time);
            }
        }

        // Calculate average stopping distance
        if (!stopping_distances.empty())
        {
            float sum = 0.0f;
            for (float dist : stopping_distances)
            {
                sum += dist;
            }
            avg_stopping_distance_ = sum / stopping_distances.size();

            // Calculate consistency (inverse of standard deviation)
            float std_dev = calculateStandardDeviation(stopping_distances, avg_stopping_distance_);
            braking_consistency_ = 1.0f / (1.0f + std_dev); // Higher consistency = lower std dev
        }

        // Calculate average reaction time
        if (!reaction_times.empty())
        {
            float sum = 0.0f;
            for (float time : reaction_times)
            {
                sum += time;
            }
            avg_reaction_time_ = sum / reaction_times.size();
        }
    }

    void BehaviorModel::analyzeSpeedDistanceRelationship(const std::vector<BehaviorDataPoint>& data_points)
    {
        for (const auto& point : data_points)
        {
            if (point.physics_data.actual_stopping_distance > 0.0f)
            {
                float speed = point.physics_data.calculated_speed;
                
                // Find appropriate speed bin
                for (auto& bin : speed_bins_)
                {
                    if (speed >= bin.min_speed && speed < bin.max_speed)
                    {
                        // Update running average
                        float total = bin.avg_stopping_distance * bin.sample_count;
                        bin.sample_count++;
                        bin.avg_stopping_distance = (total + point.physics_data.actual_stopping_distance) / bin.sample_count;
                        break;
                    }
                }
            }
        }
    }

    void BehaviorModel::analyzeSafetyMargins(const std::vector<BehaviorDataPoint>& data_points)
    {
        float total_margin = 0.0f;
        uint32_t margin_samples = 0;

        for (const auto& point : data_points)
        {
            float margin = point.safety_data.safety_margin;
            
            total_margin += margin;
            margin_samples++;
            
            if (margin < min_safety_margin_)
            {
                min_safety_margin_ = margin;
            }
            
            if (margin < 0.1f) // Less than 10cm safety margin
            {
                safety_violations_++;
            }
        }

        avg_safety_margin_ = (margin_samples > 0) ? (total_margin / margin_samples) : 0.0f;
    }

    float BehaviorModel::calculateCorrelation(const std::vector<float>& x, const std::vector<float>& y) const
    {
        if (x.size() != y.size() || x.size() < 2)
        {
            return 0.0f;
        }

        // Calculate means
        float mean_x = 0.0f, mean_y = 0.0f;
        for (size_t i = 0; i < x.size(); i++)
        {
            mean_x += x[i];
            mean_y += y[i];
        }
        mean_x /= x.size();
        mean_y /= y.size();

        // Calculate correlation coefficient
        float numerator = 0.0f, sum_sq_x = 0.0f, sum_sq_y = 0.0f;
        for (size_t i = 0; i < x.size(); i++)
        {
            float dx = x[i] - mean_x;
            float dy = y[i] - mean_y;
            numerator += dx * dy;
            sum_sq_x += dx * dx;
            sum_sq_y += dy * dy;
        }

        float denominator = std::sqrt(sum_sq_x * sum_sq_y);
        return (denominator > 0.0f) ? (numerator / denominator) : 0.0f;
    }

    float BehaviorModel::calculateStandardDeviation(const std::vector<float>& data, float mean) const
    {
        if (data.size() < 2)
        {
            return 0.0f;
        }

        float sum_sq_diff = 0.0f;
        for (float value : data)
        {
            float diff = value - mean;
            sum_sq_diff += diff * diff;
        }

        return std::sqrt(sum_sq_diff / (data.size() - 1));
    }

} // namespace digitoys::datamodeling
