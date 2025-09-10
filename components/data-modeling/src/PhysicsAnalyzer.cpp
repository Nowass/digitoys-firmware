#include "PhysicsAnalyzer.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <cmath>

namespace digitoys::datamodeling
{
    const char* PhysicsAnalyzer::TAG = "PhysicsAnalyzer";

    PhysicsAnalyzer::PhysicsAnalyzer()
    {
        ESP_LOGI(TAG, "PhysicsAnalyzer initialized");
    }

    esp_err_t PhysicsAnalyzer::analyzeDataPoint(BehaviorDataPoint& data_point, uint32_t sample_rate_ms)
    {
        // Calculate speed from distance delta if we have previous data
        if (has_previous_data_)
        {
            data_point.physics_data.distance_delta = 
                data_point.safety_data.obstacle_distance - previous_data_point_.safety_data.obstacle_distance;
            
            data_point.physics_data.calculated_speed = calculateSpeed(
                std::abs(data_point.physics_data.distance_delta), sample_rate_ms);
            
            data_point.physics_data.speed_delta = 
                data_point.physics_data.calculated_speed - previous_data_point_.physics_data.calculated_speed;

            // Calculate deceleration
            float time_delta = sample_rate_ms / 1000.0f; // Convert to seconds
            data_point.physics_data.deceleration = calculateDeceleration(
                data_point.physics_data.speed_delta, time_delta);

            // Calculate time to impact
            data_point.physics_data.time_to_impact = calculateTimeToImpact(
                data_point.physics_data.calculated_speed,
                data_point.safety_data.obstacle_distance,
                data_point.physics_data.deceleration);
        }

        // Analyze braking performance
        analyzeBrakingPerformance(data_point);

        // Update braking event state
        updateBrakingEventState(data_point);

        // Calculate physics accuracy if we have measured data
        if (data_point.physics_data.actual_stopping_distance > 0.0f)
        {
            data_point.physics_data.physics_accuracy = validatePhysicsAccuracy(data_point);
            total_accuracy_sum_ += data_point.physics_data.physics_accuracy;
        }

        // Store current data for next iteration
        previous_data_point_ = data_point;
        has_previous_data_ = true;
        total_analyses_++;

        return ESP_OK;
    }

    float PhysicsAnalyzer::calculateSpeed(float distance_delta, uint32_t sample_rate_ms)
    {
        if (sample_rate_ms == 0) return 0.0f;
        
        float time_delta = sample_rate_ms / 1000.0f; // Convert to seconds
        return distance_delta / time_delta; // m/s
    }

    esp_err_t PhysicsAnalyzer::analyzeBrakingPerformance(BehaviorDataPoint& data_point)
    {
        // Calculate safety margin
        data_point.safety_data.safety_margin = 
            data_point.safety_data.obstacle_distance - data_point.safety_data.brake_distance;

        // If we're in a braking event, update stopping calculations
        if (in_braking_event_)
        {
            // Check if we've stopped (speed below threshold)
            if (data_point.physics_data.calculated_speed < 0.05f) // 5cm/s threshold
            {
                data_point.physics_data.actual_stopping_distance = 
                    brake_start_distance_ - data_point.safety_data.obstacle_distance;
                
                data_point.physics_data.stopping_time = 
                    (data_point.session_data.absolute_timestamp_us - brake_start_time_) / 1000000.0f; // Convert to seconds
                
                brake_analyses_++;
            }
        }

        return ESP_OK;
    }

    float PhysicsAnalyzer::validatePhysicsAccuracy(const BehaviorDataPoint& data_point)
    {
        if (data_point.physics_data.actual_stopping_distance <= 0.0f)
        {
            return 1.0f; // No measured data to compare
        }

        float predicted = predictStoppingDistance(
            data_point.physics_data.calculated_speed, 
            data_point.rc_control.rc_input);
        
        if (predicted <= 0.0f) return 0.0f;
        
        return data_point.physics_data.actual_stopping_distance / predicted;
    }

    float PhysicsAnalyzer::predictStoppingDistance(float current_speed, float rc_input)
    {
        if (current_speed <= 0.0f) return 0.0f;
        
        // Simple physics model: d = v²/(2*a)
        // where a = brake_coefficient * friction_coefficient * rc_input * 9.81 m/s²
        float effective_deceleration = brake_coefficient_ * friction_coefficient_ * rc_input * 9.81f;
        
        if (effective_deceleration <= 0.0f) return 0.0f;
        
        return (current_speed * current_speed) / (2.0f * effective_deceleration);
    }

    void PhysicsAnalyzer::getAnalysisStats(float& avg_accuracy, uint32_t& total_analyses, uint32_t& brake_analyses) const
    {
        avg_accuracy = (total_analyses_ > 0) ? (total_accuracy_sum_ / total_analyses_) : 0.0f;
        total_analyses = total_analyses_;
        brake_analyses = brake_analyses_;
    }

    void PhysicsAnalyzer::resetStats()
    {
        total_accuracy_sum_ = 0.0f;
        total_analyses_ = 0;
        brake_analyses_ = 0;
        has_previous_data_ = false;
        in_braking_event_ = false;
        
        ESP_LOGI(TAG, "Physics analyzer stats reset");
    }

    void PhysicsAnalyzer::setModelParameters(float brake_coefficient, float friction_coefficient, uint32_t reaction_time_ms)
    {
        brake_coefficient_ = brake_coefficient;
        friction_coefficient_ = friction_coefficient;
        reaction_time_ms_ = reaction_time_ms;
        
        ESP_LOGI(TAG, "Physics model parameters updated: brake=%.2f, friction=%.2f, reaction=%lums",
                 brake_coefficient_, friction_coefficient_, reaction_time_ms_);
    }

    float PhysicsAnalyzer::calculateDeceleration(float speed_delta, float time_delta)
    {
        if (time_delta <= 0.0f) return 0.0f;
        return speed_delta / time_delta; // m/s²
    }

    float PhysicsAnalyzer::calculateTimeToImpact(float current_speed, float distance_to_obstacle, float current_deceleration)
    {
        if (current_speed <= 0.0f || distance_to_obstacle <= 0.0f) return -1.0f;
        
        // If decelerating, use kinematic equation: t = (-v + sqrt(v² + 2*a*d)) / a
        if (current_deceleration > 0.1f) // Minimum deceleration threshold
        {
            float discriminant = current_speed * current_speed + 2.0f * current_deceleration * distance_to_obstacle;
            if (discriminant >= 0)
            {
                return (-current_speed + std::sqrt(discriminant)) / current_deceleration;
            }
        }
        
        // If not decelerating, simple time = distance / speed
        return distance_to_obstacle / current_speed;
    }

    void PhysicsAnalyzer::updateBrakingEventState(BehaviorDataPoint& data_point)
    {
        // Detect start of braking event
        if (!in_braking_event_ && data_point.safety_data.is_obstacle_state)
        {
            in_braking_event_ = true;
            brake_start_time_ = data_point.session_data.absolute_timestamp_us;
            brake_start_distance_ = data_point.safety_data.obstacle_distance;
            brake_start_speed_ = data_point.physics_data.calculated_speed;
            
            ESP_LOGD(TAG, "Braking event started: speed=%.2f m/s, distance=%.2f m",
                     brake_start_speed_, brake_start_distance_);
        }
        
        // Detect end of braking event (when no longer in obstacle state and stopped)
        if (in_braking_event_ && !data_point.safety_data.is_obstacle_state && 
            data_point.physics_data.calculated_speed < 0.05f)
        {
            in_braking_event_ = false;
            ESP_LOGD(TAG, "Braking event ended: stopping_distance=%.2f m, stopping_time=%.2f s",
                     data_point.physics_data.actual_stopping_distance,
                     data_point.physics_data.stopping_time);
        }
    }

} // namespace digitoys::datamodeling
