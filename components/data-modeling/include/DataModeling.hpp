#pragma once

#include "DataModelingTypes.hpp"
#include "TestSession.hpp"
#include "PhysicsAnalyzer.hpp"
#include "BehaviorModel.hpp"
#include "IDataSource.hpp" // For DataEntry
#include <ComponentBase.hpp>
#include <esp_err.h>
#include <memory>
#include <vector>
#include <functional>

namespace digitoys::datamodeling
{
    /**
     * @brief Main data modeling component coordinator
     * 
     * Coordinates:
     * - Data collection from data-logger
     * - Physics analysis and enhancement
     * - Test session management
     * - Data export and formatting
     * - Integration with wifi-monitor dashboard
     */
    class DataModeling : public digitoys::core::ComponentBase
    {
    public:
        DataModeling();
        virtual ~DataModeling();

        // IComponent interface implementation
        esp_err_t initialize() override;
        esp_err_t start() override;
        esp_err_t stop() override;
        esp_err_t shutdown() override;

        /**
         * @brief Process raw data from data-logger and enhance with modeling
         * @param raw_data Raw data entries from data-logger
         * @return Enhanced behavior data point
         */
        BehaviorDataPoint processRawData(const std::vector<digitoys::datalogger::DataEntry>& raw_data);

        /**
         * @brief Start a new test session
         * @param description Session description
         * @return Session ID or 0 on error
         */
        uint32_t startTestSession(const std::string& description = "");

        /**
         * @brief Stop current test session
         * @return ESP_OK on success
         */
        esp_err_t stopTestSession();

        /**
         * @brief Mark manual stop event
         * @return Stop event ID or 0 if no active session
         */
        uint32_t markStopEvent();

        /**
         * @brief Get current session information
         * @return Pointer to current session or nullptr
         */
        const TestSessionData* getCurrentSession() const;

        /**
         * @brief Get all behavior data for a session
         * @param session_id Session ID (0 = current session)
         * @return Vector of behavior data points
         */
        std::vector<BehaviorDataPoint> getSessionData(uint32_t session_id = 0) const;

        /**
         * @brief Export session data to CSV format
         * @param config Export configuration
         * @return CSV string
         */
        std::string exportToCSV(const ExportConfig& config) const;

        /**
         * @brief Export session data to JSON format
         * @param config Export configuration  
         * @return JSON string
         */
        std::string exportToJSON(const ExportConfig& config) const;

        /**
         * @brief Get modeling statistics
         * @param total_sessions Total number of sessions
         * @param total_data_points Total behavior data points
         * @param physics_accuracy Average physics calculation accuracy
         */
        void getStatistics(uint32_t& total_sessions, uint32_t& total_data_points, float& physics_accuracy) const;

        /**
         * @brief Set data collection callback for real-time processing
         * @param callback Function to call when new data is processed
         */
        void setDataCallback(std::function<void(const BehaviorDataPoint&)> callback);

        /**
         * @brief Configure physics model parameters
         * @param brake_coefficient Braking effectiveness (0.0-1.0)
         * @param friction_coefficient Surface friction (0.0-1.0)
         * @param reaction_time_ms Average reaction time
         */
        void configurePhysicsModel(float brake_coefficient, float friction_coefficient, uint32_t reaction_time_ms);

        /**
         * @brief Get next session ID for preview
         * @return Next session ID that would be assigned
         */
        uint32_t getNextSessionId() const;

        /**
         * @brief Clear all data (for testing/reset)
         */
        void clearAllData();

    private:
        static const char* TAG;

        // Component instances
                std::unique_ptr<TestSessionManager> session_manager_;
        std::unique_ptr<PhysicsAnalyzer> physics_analyzer_;

        // Data storage
        std::vector<BehaviorDataPoint> behavior_data_;
        uint32_t sample_rate_ms_ = 200; // Default sampling rate

        // Callback for real-time data
        std::function<void(const BehaviorDataPoint&)> data_callback_;

        // Statistics
        uint32_t total_data_points_ = 0;

        /**
         * @brief Convert raw data entries to behavior data point
         * @param raw_data Raw data from data-logger
         * @return Basic behavior data point (before physics analysis)
         */
        BehaviorDataPoint convertRawData(const std::vector<digitoys::datalogger::DataEntry>& raw_data);

        /**
         * @brief Generate CSV header for export
         * @param config Export configuration
         * @return CSV header string
         */
        std::string generateCSVHeader(const ExportConfig& config) const;

        /**
         * @brief Convert behavior data point to CSV row
         * @param data_point Behavior data point
         * @param config Export configuration
         * @return CSV row string
         */
        std::string behaviorDataToCSV(const BehaviorDataPoint& data_point, const ExportConfig& config) const;

        /**
         * @brief Generate session info header for export
         * @param session_id Session ID
         * @return Session info string
         */
        std::string generateSessionHeader(uint32_t session_id) const;
    };

} // namespace digitoys::datamodeling
