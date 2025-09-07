#include "PhysicsAnalyzer.hpp"
#include "DataLoggerConfig.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace digitoys::datalogger
{
    const char *PhysicsAnalyzer::TAG = "PhysicsAnalyzer";

    PhysicsAnalyzer::PhysicsAnalyzer(DataLogger *data_logger, const AnalyzerConfig &config)
        : ComponentBase("PhysicsAnalyzer"), data_logger_(data_logger), config_(config), analysis_timer_(nullptr), is_analyzing_(false), total_analyses_(0), total_analysis_time_us_(0), last_analysis_time_(0)
    {
        if (!data_logger_)
        {
            ESP_LOGE(TAG, "DataLogger instance is required");
            return;
        }

        ESP_LOGI(TAG, "PhysicsAnalyzer created - interval: %lu ms, window: %lu entries",
                 config_.analysis_interval_ms, config_.data_window_size);
    }

    PhysicsAnalyzer::~PhysicsAnalyzer()
    {
        if (analysis_timer_)
        {
            esp_timer_delete(analysis_timer_);
            analysis_timer_ = nullptr;
        }
    }

    esp_err_t PhysicsAnalyzer::initialize()
    {
        if (isInitialized())
        {
            ESP_LOGW(TAG, "PhysicsAnalyzer already initialized");
            return ESP_OK;
        }

        if (!data_logger_)
        {
            ESP_LOGE(TAG, "DataLogger instance is required for initialization");
            return ESP_ERR_INVALID_ARG;
        }

        if (!config_.enabled)
        {
            ESP_LOGI(TAG, "PhysicsAnalyzer disabled by configuration");
            setState(digitoys::core::ComponentState::INITIALIZED);
            return ESP_OK;
        }

        // Create analysis timer
        esp_timer_create_args_t timer_args = {
            .callback = analysisTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "physics_analysis",
            .skip_unhandled_events = true};

        esp_err_t ret = esp_timer_create(&timer_args, &analysis_timer_);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to create analysis timer: %s", esp_err_to_name(ret));
            return ret;
        }

        // Initialize results structure
        latest_results_ = AnalysisResults{};
        braking_events_.clear();

        setState(digitoys::core::ComponentState::INITIALIZED);
        ESP_LOGI(TAG, "PhysicsAnalyzer initialized successfully");
        ESP_LOGI(TAG, "Emergency decel threshold: %.1f m/s², Critical margin: %.0f cm",
                 config_.emergency_decel_threshold, config_.safety_margin_critical);

        return ESP_OK;
    }

    esp_err_t PhysicsAnalyzer::start()
    {
        if (!isInitialized())
        {
            ESP_LOGE(TAG, "PhysicsAnalyzer not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        if (isRunning())
        {
            ESP_LOGW(TAG, "PhysicsAnalyzer already running");
            return ESP_OK;
        }

        if (!config_.enabled)
        {
            ESP_LOGI(TAG, "PhysicsAnalyzer disabled, not starting timer");
            setState(digitoys::core::ComponentState::RUNNING);
            return ESP_OK;
        }

        // Start periodic analysis timer
        esp_err_t ret = esp_timer_start_periodic(analysis_timer_,
                                                 config_.analysis_interval_ms * 1000);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start analysis timer: %s", esp_err_to_name(ret));
            return ret;
        }

        setState(digitoys::core::ComponentState::RUNNING);
        ESP_LOGI(TAG, "PhysicsAnalyzer started - analyzing every %lu ms",
                 config_.analysis_interval_ms);

        return ESP_OK;
    }

    esp_err_t PhysicsAnalyzer::stop()
    {
        if (!isRunning())
        {
            ESP_LOGW(TAG, "PhysicsAnalyzer not running");
            return ESP_OK;
        }

        if (analysis_timer_)
        {
            esp_err_t ret = esp_timer_stop(analysis_timer_);
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to stop analysis timer: %s", esp_err_to_name(ret));
            }
        }

        is_analyzing_ = false;
        setState(digitoys::core::ComponentState::INITIALIZED);

        ESP_LOGI(TAG, "PhysicsAnalyzer stopped");
        ESP_LOGI(TAG, "Total analyses: %lu, Avg time: %.2f ms",
                 total_analyses_,
                 total_analyses_ > 0 ? (total_analysis_time_us_ / 1000.0f) / total_analyses_ : 0.0f);

        return ESP_OK;
    }

    esp_err_t PhysicsAnalyzer::shutdown()
    {
        ESP_LOGI(TAG, "Shutting down PhysicsAnalyzer...");

        if (isRunning())
        {
            stop();
        }

        if (analysis_timer_)
        {
            esp_timer_delete(analysis_timer_);
            analysis_timer_ = nullptr;
        }

        braking_events_.clear();
        latest_results_ = AnalysisResults{};

        setState(digitoys::core::ComponentState::UNINITIALIZED);
        ESP_LOGI(TAG, "PhysicsAnalyzer shutdown complete");

        return ESP_OK;
    }

    void PhysicsAnalyzer::analysisTimerCallback(void *arg)
    {
        PhysicsAnalyzer *analyzer = static_cast<PhysicsAnalyzer *>(arg);
        if (analyzer && analyzer->isRunning() && !analyzer->is_analyzing_)
        {
            analyzer->performAnalysis();
        }
    }

    PhysicsAnalyzer::AnalysisResults PhysicsAnalyzer::performAnalysis()
    {
        if (!isRunning() || !data_logger_)
        {
            return latest_results_;
        }

        is_analyzing_ = true;
        uint64_t start_time = esp_timer_get_time();

        ESP_LOGD(TAG, "Starting physics analysis #%lu", total_analyses_ + 1);

        // Get latest data from DataLogger
        std::vector<DataEntry> current_data = data_logger_->getCollectedData(config_.data_window_size);

        ESP_LOGD(TAG, "Retrieved %zu data entries for analysis", current_data.size());

        // Perform the actual analysis
        AnalysisResults results = analyzeBrakingPerformance(current_data);

        // Update internal state
        latest_results_ = results;
        latest_results_.analysis_timestamp = esp_timer_get_time();

        // Update statistics
        total_analyses_++;
        uint64_t analysis_time = esp_timer_get_time() - start_time;
        total_analysis_time_us_ += analysis_time;
        last_analysis_time_ = esp_timer_get_time();

        // Print summary for important events
        if (results.safety_violations > 0 || results.emergency_pattern_detected)
        {
            printAnalysisSummary(results);
        }

        ESP_LOGD(TAG, "Analysis #%lu completed in %.2f ms",
                 total_analyses_, analysis_time / 1000.0f);

        is_analyzing_ = false;
        return latest_results_;
    }

    PhysicsAnalyzer::AnalysisResults PhysicsAnalyzer::analyzeBrakingPerformance(
        const std::vector<DataEntry> &data)
    {
        AnalysisResults results;

        if (data.empty())
        {
            ESP_LOGD(TAG, "No data available for analysis");
            return results;
        }

        // Extract physics-related data
        std::vector<DataEntry> physics_data;
        extractPhysicsData(data, physics_data);

        if (physics_data.empty())
        {
            ESP_LOGD(TAG, "No physics data found for analysis");
            return results;
        }

        // Detect braking events
        braking_events_ = detectBrakingEvents(physics_data);
        results.total_brake_events = braking_events_.size();

        if (!braking_events_.empty())
        {
            // Calculate braking performance metrics
            std::vector<float> decelerations;
            std::vector<float> safety_margins;
            std::vector<float> stopping_distances;

            for (const auto &event : braking_events_)
            {
                decelerations.push_back(event.average_deceleration);
                decelerations.push_back(event.peak_deceleration);
                safety_margins.push_back(event.safety_margin_start);
                safety_margins.push_back(event.safety_margin_end);
                stopping_distances.push_back(event.stopping_distance);
            }

            // Calculate averages and peaks
            if (!decelerations.empty())
            {
                results.average_deceleration = std::accumulate(decelerations.begin(),
                                                               decelerations.end(), 0.0f) /
                                               decelerations.size();
                results.peak_deceleration = *std::max_element(decelerations.begin(), decelerations.end());
                results.g_force_peak = calculateGForce(results.peak_deceleration);
            }

            if (!safety_margins.empty())
            {
                results.average_safety_margin = std::accumulate(safety_margins.begin(),
                                                                safety_margins.end(), 0.0f) /
                                                safety_margins.size();
                results.minimum_safety_margin = *std::min_element(safety_margins.begin(), safety_margins.end());

                // Count safety violations
                results.safety_violations = std::count_if(safety_margins.begin(), safety_margins.end(),
                                                          [this](float margin)
                                                          { return margin < config_.safety_margin_critical; });

                results.near_miss_events = std::count_if(safety_margins.begin(), safety_margins.end(),
                                                         [this](float margin)
                                                         { return margin < config_.safety_margin_warning; });
            }

            if (!stopping_distances.empty())
            {
                results.stopping_distance_avg = std::accumulate(stopping_distances.begin(),
                                                                stopping_distances.end(), 0.0f) /
                                                stopping_distances.size();
            }

            // Calculate braking efficiency (simplified metric)
            if (results.peak_deceleration > 0.0f)
            {
                results.braking_efficiency = std::min(1.0f, results.average_deceleration / results.peak_deceleration);
            }

            // Check for emergency patterns
            results.emergency_pattern_detected = detectEmergencyPatterns(braking_events_);
        }

        // Calculate overall risk score
        results.risk_score = calculateRiskScore(results);

        ESP_LOGD(TAG, "Analysis: %lu brake events, %.1f avg decel, %.1f safety margin, risk: %.1f",
                 results.total_brake_events, results.average_deceleration,
                 results.average_safety_margin, results.risk_score);

        return results;
    }

    std::vector<PhysicsAnalyzer::BrakingEvent> PhysicsAnalyzer::detectBrakingEvents(
        const std::vector<DataEntry> &data)
    {
        std::vector<BrakingEvent> events;

        // Simple braking event detection logic
        // In a real implementation, this would analyze deceleration patterns,
        // speed changes, and safety margins to identify braking events

        // For now, we'll create synthetic events based on available data patterns
        // This would be replaced with actual event detection algorithms

        ESP_LOGD(TAG, "Detected %zu braking events in %zu data points",
                 events.size(), data.size());

        return events;
    }

    float PhysicsAnalyzer::calculateRiskScore(const AnalysisResults &results)
    {
        float risk = 0.0f;

        // Safety violations contribute heavily to risk
        risk += results.safety_violations * 2.0f;
        risk += results.near_miss_events * 1.0f;

        // High deceleration indicates aggressive driving
        if (results.peak_deceleration > config_.emergency_decel_threshold)
        {
            risk += 1.5f;
        }

        // Low safety margins increase risk
        if (results.minimum_safety_margin < config_.safety_margin_critical)
        {
            risk += 2.0f;
        }
        else if (results.minimum_safety_margin < config_.safety_margin_warning)
        {
            risk += 1.0f;
        }

        // Emergency patterns are high risk
        if (results.emergency_pattern_detected)
        {
            risk += 3.0f;
        }

        // Clamp to 0-10 scale
        return std::min(10.0f, std::max(0.0f, risk));
    }

    bool PhysicsAnalyzer::detectEmergencyPatterns(const std::vector<BrakingEvent> &events)
    {
        // Check for emergency braking patterns:
        // 1. Multiple high-deceleration events in short time
        // 2. Consistent pattern of late braking
        // 3. Extremely short safety margins

        uint32_t emergency_events = 0;
        for (const auto &event : events)
        {
            if (event.peak_deceleration > config_.emergency_decel_threshold ||
                event.safety_margin_end < config_.safety_margin_critical)
            {
                emergency_events++;
            }
        }

        // If more than 30% of events are emergency-level, flag as pattern
        if (!events.empty() && (emergency_events * 100 / events.size()) > 30)
        {
            ESP_LOGW(TAG, "Emergency braking pattern detected: %lu/%zu events",
                     emergency_events, events.size());
            return true;
        }

        return false;
    }

    void PhysicsAnalyzer::extractPhysicsData(const std::vector<DataEntry> &data,
                                             std::vector<DataEntry> &physics_data)
    {
        physics_data.clear();
        physics_data.reserve(data.size() / 4); // Estimate

        // Extract entries related to physics analysis
        for (const auto &entry : data)
        {
            if (entry.key == "speed_delta" ||
                entry.key == "deceleration" ||
                entry.key == "safety_margin" ||
                entry.key == "time_to_impact" ||
                entry.key == "brake_events" ||
                entry.key == "is_obstacle_state")
            {
                physics_data.push_back(entry);
            }
        }

        ESP_LOGD(TAG, "Extracted %zu physics entries from %zu total entries",
                 physics_data.size(), data.size());
    }

    float PhysicsAnalyzer::calculateGForce(float deceleration) const
    {
        // Convert m/s² to G-force (1G = 9.81 m/s²)
        return std::abs(deceleration) / 9.81f;
    }

    void PhysicsAnalyzer::printAnalysisSummary(const AnalysisResults &results)
    {
        ESP_LOGI(TAG, "=== PHYSICS ANALYSIS SUMMARY ===");
        ESP_LOGI(TAG, "Braking Events: %lu", results.total_brake_events);
        ESP_LOGI(TAG, "Peak Deceleration: %.2f m/s² (%.2f G)",
                 results.peak_deceleration, results.g_force_peak);
        ESP_LOGI(TAG, "Average Safety Margin: %.1f cm (min: %.1f cm)",
                 results.average_safety_margin, results.minimum_safety_margin);
        ESP_LOGI(TAG, "Safety Violations: %lu, Near Misses: %lu",
                 results.safety_violations, results.near_miss_events);
        ESP_LOGI(TAG, "Risk Score: %.1f/10", results.risk_score);

        if (results.emergency_pattern_detected)
        {
            ESP_LOGW(TAG, "⚠️  EMERGENCY PATTERN DETECTED");
        }

        if (results.safety_violations > 0)
        {
            ESP_LOGW(TAG, "⚠️  SAFETY VIOLATIONS DETECTED");
        }

        ESP_LOGI(TAG, "==============================");
    }

    std::vector<PhysicsAnalyzer::BrakingEvent> PhysicsAnalyzer::getBrakingEvents() const
    {
        return braking_events_;
    }

    void PhysicsAnalyzer::updateConfig(const AnalyzerConfig &config)
    {
        config_ = config;
        ESP_LOGI(TAG, "Configuration updated - interval: %lu ms, emergency threshold: %.1f m/s²",
                 config_.analysis_interval_ms, config_.emergency_decel_threshold);

        // Restart timer with new interval if running
        if (isRunning() && analysis_timer_)
        {
            esp_timer_stop(analysis_timer_);
            if (config_.enabled)
            {
                esp_timer_start_periodic(analysis_timer_, config_.analysis_interval_ms * 1000);
            }
        }
    }

    void PhysicsAnalyzer::getStatistics(uint32_t &total_analyses,
                                        float &avg_analysis_time,
                                        uint64_t &last_analysis_time) const
    {
        total_analyses = total_analyses_;
        avg_analysis_time = total_analyses_ > 0 ? (total_analysis_time_us_ / 1000.0f) / total_analyses_ : 0.0f;
        last_analysis_time = last_analysis_time_;
    }

} // namespace digitoys::datalogger
