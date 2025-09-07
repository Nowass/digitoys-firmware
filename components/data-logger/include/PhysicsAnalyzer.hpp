#pragma once

#include "DataLogger.hpp"
#include "ComponentBase.hpp"
#include <vector>
#include <memory>
#include <esp_timer.h>

namespace digitoys::datalogger
{
    /**
     * @brief Advanced physics analysis for vehicle safety systems
     * 
     * Analyzes collected physics data to provide:
     * - Braking performance metrics
     * - G-force trend analysis
     * - Safety margin evaluation
     * - Predictive collision warnings
     * - Emergency braking pattern recognition
     */
    class PhysicsAnalyzer : public digitoys::core::ComponentBase
    {
    public:
        /**
         * @brief Physics analysis results structure
         */
        struct AnalysisResults
        {
            // Braking Performance
            float average_deceleration = 0.0f;     ///< Average deceleration during braking events (m/s²)
            float peak_deceleration = 0.0f;        ///< Maximum deceleration recorded (m/s²)
            float braking_efficiency = 0.0f;       ///< Braking efficiency score (0-1)
            uint32_t total_brake_events = 0;       ///< Total number of braking events
            
            // Safety Analysis
            float average_safety_margin = 0.0f;    ///< Average safety margin (cm)
            float minimum_safety_margin = 0.0f;    ///< Minimum safety margin recorded (cm)
            uint32_t safety_violations = 0;        ///< Number of safety margin violations
            uint32_t near_miss_events = 0;         ///< Near collision events
            
            // Performance Trends
            float reaction_time_avg = 0.0f;        ///< Average reaction time (ms)
            float stopping_distance_avg = 0.0f;    ///< Average stopping distance (cm)
            float g_force_peak = 0.0f;             ///< Peak G-force during analysis period
            
            // Risk Assessment
            float risk_score = 0.0f;               ///< Overall risk score (0-10, lower is better)
            bool emergency_pattern_detected = false; ///< Emergency braking pattern flag
            uint64_t analysis_timestamp = 0;       ///< When analysis was performed
        };

        /**
         * @brief Braking event details
         */
        struct BrakingEvent
        {
            uint64_t start_timestamp = 0;          ///< Event start time (μs)
            uint64_t end_timestamp = 0;            ///< Event end time (μs)
            float initial_speed = 0.0f;            ///< Speed when braking started
            float final_speed = 0.0f;              ///< Speed when braking ended
            float peak_deceleration = 0.0f;        ///< Maximum deceleration during event
            float average_deceleration = 0.0f;     ///< Average deceleration during event
            float stopping_distance = 0.0f;        ///< Distance traveled during braking
            float safety_margin_start = 0.0f;      ///< Safety margin at event start
            float safety_margin_end = 0.0f;        ///< Safety margin at event end
            bool emergency_brake = false;          ///< Whether this was emergency braking
            float reaction_time = 0.0f;            ///< Reaction time before braking (ms)
        };

        /**
         * @brief Configuration for physics analysis
         */
        struct AnalyzerConfig
        {
            bool enabled = true;                    ///< Enable/disable analysis
            uint32_t analysis_interval_ms = 5000;  ///< How often to run analysis
            uint32_t data_window_size = 100;       ///< Number of data points to analyze
            float emergency_decel_threshold = 5.0f; ///< Deceleration threshold for emergency (m/s²)
            float safety_margin_critical = 20.0f;  ///< Critical safety margin (cm)
            float safety_margin_warning = 50.0f;   ///< Warning safety margin (cm)
            float g_force_warning = 0.3f;          ///< G-force warning threshold
            float g_force_critical = 0.5f;         ///< G-force critical threshold
        };

        /**
         * @brief Constructor
         * @param data_logger Pointer to the DataLogger instance
         * @param config Analysis configuration
         */
        explicit PhysicsAnalyzer(DataLogger* data_logger, 
                                const AnalyzerConfig& config);

        /**
         * @brief Destructor
         */
        virtual ~PhysicsAnalyzer();

        // ComponentBase interface
        esp_err_t initialize() override;
        esp_err_t start() override;
        esp_err_t stop() override;
        esp_err_t shutdown() override;

        /**
         * @brief Perform physics analysis on current data
         * @return Analysis results structure
         */
        AnalysisResults performAnalysis();

        /**
         * @brief Get the latest analysis results
         * @return Most recent analysis results
         */
        const AnalysisResults& getLatestResults() const { return latest_results_; }

        /**
         * @brief Get detected braking events
         * @return Vector of braking events found in data
         */
        std::vector<BrakingEvent> getBrakingEvents() const;

        /**
         * @brief Update analyzer configuration
         * @param config New configuration
         */
        void updateConfig(const AnalyzerConfig& config);

        /**
         * @brief Get current configuration
         * @return Current analyzer configuration
         */
        const AnalyzerConfig& getConfig() const { return config_; }

        /**
         * @brief Check if analysis is currently running
         * @return True if analysis is active
         */
        bool isAnalyzing() const { return is_analyzing_; }

        /**
         * @brief Get analysis statistics
         * @param total_analyses Total number of analyses performed
         * @param avg_analysis_time Average analysis time in milliseconds
         * @param last_analysis_time Time of last analysis
         */
        void getStatistics(uint32_t& total_analyses, 
                          float& avg_analysis_time,
                          uint64_t& last_analysis_time) const;

    private:
        static const char* TAG;

        DataLogger* data_logger_;               ///< DataLogger instance
        AnalyzerConfig config_;                 ///< Analysis configuration
        AnalysisResults latest_results_;        ///< Latest analysis results
        std::vector<BrakingEvent> braking_events_; ///< Detected braking events
        
        esp_timer_handle_t analysis_timer_;     ///< Timer for periodic analysis
        bool is_analyzing_;                     ///< Analysis state flag
        
        // Statistics
        uint32_t total_analyses_;               ///< Total analyses performed
        uint64_t total_analysis_time_us_;       ///< Total time spent analyzing
        uint64_t last_analysis_time_;           ///< Timestamp of last analysis

        /**
         * @brief Timer callback for periodic analysis
         * @param arg Pointer to PhysicsAnalyzer instance
         */
        static void analysisTimerCallback(void* arg);

        /**
         * @brief Analyze braking performance from data
         * @param data Vector of data entries to analyze
         * @return Analysis results
         */
        AnalysisResults analyzeBrakingPerformance(const std::vector<DataEntry>& data);

        /**
         * @brief Detect braking events in the data
         * @param data Vector of data entries to analyze
         * @return Vector of detected braking events
         */
        std::vector<BrakingEvent> detectBrakingEvents(const std::vector<DataEntry>& data);

        /**
         * @brief Calculate risk score based on analysis
         * @param results Analysis results to evaluate
         * @return Risk score (0-10, lower is better)
         */
        float calculateRiskScore(const AnalysisResults& results);

        /**
         * @brief Check for emergency braking patterns
         * @param events Vector of braking events to analyze
         * @return True if emergency patterns detected
         */
        bool detectEmergencyPatterns(const std::vector<BrakingEvent>& events);

        /**
         * @brief Extract physics data from data entries
         * @param data All data entries
         * @param physics_data Output vector for physics-related entries
         */
        void extractPhysicsData(const std::vector<DataEntry>& data,
                               std::vector<DataEntry>& physics_data);

        /**
         * @brief Calculate G-force from deceleration
         * @param deceleration Deceleration in m/s²
         * @return G-force value
         */
        float calculateGForce(float deceleration) const;

        /**
         * @brief Print analysis summary to console
         * @param results Analysis results to print
         */
        void printAnalysisSummary(const AnalysisResults& results);
    };

} // namespace digitoys::datalogger
