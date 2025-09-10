#include "DataModeling.hpp"
#include "IDataSource.hpp" // For DataEntry
#include <esp_log.h>
#include <esp_timer.h>
#include <sstream>
#include <iomanip>

namespace digitoys::datamodeling
{
    const char* DataModeling::TAG = "DataModeling";

    DataModeling::DataModeling() 
        : ComponentBase("DataModeling")
    {
        session_manager_ = std::make_unique<TestSessionManager>();
        physics_analyzer_ = std::make_unique<PhysicsAnalyzer>();
    }

    DataModeling::~DataModeling() = default;

    esp_err_t DataModeling::initialize()
    {
        ESP_LOGI(TAG, "Initializing DataModeling component");
        setState(digitoys::core::ComponentState::INITIALIZED);
        return ESP_OK;
    }

    esp_err_t DataModeling::start()
    {
        if (getState() != digitoys::core::ComponentState::INITIALIZED &&
            getState() != digitoys::core::ComponentState::STOPPED)
        {
            ESP_LOGW(TAG, "Component not in correct state to start");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Starting DataModeling component");
        setState(digitoys::core::ComponentState::RUNNING);
        return ESP_OK;
    }

    esp_err_t DataModeling::stop()
    {
        if (getState() != digitoys::core::ComponentState::RUNNING)
        {
            ESP_LOGW(TAG, "Component not running, cannot stop");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG, "Stopping DataModeling component");
        
        // Stop any active session
        if (session_manager_->isSessionActive())
        {
            session_manager_->stopSession();
        }
        
        setState(digitoys::core::ComponentState::STOPPED);
        return ESP_OK;
    }

    esp_err_t DataModeling::shutdown()
    {
        if (getState() == digitoys::core::ComponentState::RUNNING)
        {
            esp_err_t ret = stop();
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "Failed to stop during shutdown");
            }
        }

        ESP_LOGI(TAG, "Shutting down DataModeling component");
        setState(digitoys::core::ComponentState::STOPPED);
        return ESP_OK;
    }

    BehaviorDataPoint DataModeling::processRawData(const std::vector<digitoys::datalogger::DataEntry>& raw_data)
    {
        // Convert raw data to behavior data point
        BehaviorDataPoint data_point = convertRawData(raw_data);
        
        // Add session data
        data_point.session_data.session_id = session_manager_->getCurrentSessionId();
        data_point.session_data.absolute_timestamp_us = esp_timer_get_time();
        data_point.session_data.session_relative_ms = session_manager_->getSessionRelativeTime(
            data_point.session_data.absolute_timestamp_us);

        // Enhance with physics analysis
        if (physics_analyzer_)
        {
            physics_analyzer_->analyzeDataPoint(data_point, sample_rate_ms_);
        }

        // Update session statistics
        if (session_manager_->isSessionActive())
        {
            session_manager_->updateSessionStats(data_point);
        }

        // Store the data point
        behavior_data_.push_back(data_point);
        total_data_points_++;

        // Call data callback if set
        if (data_callback_)
        {
            data_callback_(data_point);
        }

        return data_point;
    }

    uint32_t DataModeling::startTestSession(const std::string& description)
    {
        return session_manager_->startSession(description);
    }

    esp_err_t DataModeling::stopTestSession()
    {
        return session_manager_->stopSession();
    }

    uint32_t DataModeling::markStopEvent()
    {
        return session_manager_->markStopEvent();
    }

        const TestSessionData* DataModeling::getCurrentSession() const
    {
        return session_manager_->getCurrentSession();
    }

    std::vector<BehaviorDataPoint> DataModeling::getSessionData(uint32_t session_id) const
    {
        std::vector<BehaviorDataPoint> session_data;
        
        uint32_t target_session = (session_id == 0) ? session_manager_->getCurrentSessionId() : session_id;
        
        for (const auto& data_point : behavior_data_)
        {
            if (data_point.session_data.session_id == target_session)
            {
                session_data.push_back(data_point);
            }
        }
        
        return session_data;
    }

    std::string DataModeling::exportToCSV(const ExportConfig& config) const
    {
        std::ostringstream csv;
        
        // Add session header if requested
        if (config.include_session_header && !config.session_ids.empty())
        {
            csv << generateSessionHeader(config.session_ids[0]);
            csv << "\n";
        }
        
        // Add CSV header
        csv << generateCSVHeader(config) << "\n";
        
        // Add data rows
        for (const auto& data_point : behavior_data_)
        {
            // Filter by session IDs if specified
            if (!config.session_ids.empty())
            {
                bool include = false;
                for (uint32_t session_id : config.session_ids)
                {
                    if (data_point.session_data.session_id == session_id)
                    {
                        include = true;
                        break;
                    }
                }
                if (!include) continue;
            }
            
            csv << behaviorDataToCSV(data_point, config) << "\n";
        }
        
        return csv.str();
    }

    std::string DataModeling::exportToJSON(const ExportConfig& config) const
    {
        // Basic JSON export (simplified implementation)
        std::ostringstream json;
        json << "{\n";
        json << "  \"export_config\": {\n";
        json << "    \"format\": \"json\",\n";
        json << "    \"timestamp\": " << esp_timer_get_time() << "\n";
        json << "  },\n";
        json << "  \"data_points\": [\n";
        
        bool first = true;
        for (const auto& data_point : behavior_data_)
        {
            // Apply session filtering
            if (!config.session_ids.empty())
            {
                bool include = false;
                for (uint32_t session_id : config.session_ids)
                {
                    if (data_point.session_data.session_id == session_id)
                    {
                        include = true;
                        break;
                    }
                }
                if (!include) continue;
            }
            
            if (!first) json << ",\n";
            first = false;
            
            json << "    {\n";
            json << "      \"session_id\": " << data_point.session_data.session_id << ",\n";
            json << "      \"timestamp_ms\": " << data_point.session_data.session_relative_ms << ",\n";
            json << "      \"rc_input\": " << data_point.rc_control.rc_input << ",\n";
            json << "      \"calculated_speed\": " << data_point.physics_data.calculated_speed << ",\n";
            json << "      \"obstacle_distance\": " << data_point.safety_data.obstacle_distance << ",\n";
            json << "      \"is_braking\": " << (data_point.safety_data.is_obstacle_state ? "true" : "false") << "\n";
            json << "    }";
        }
        
        json << "\n  ]\n";
        json << "}";
        
        return json.str();
    }

    void DataModeling::getStatistics(uint32_t& total_sessions, uint32_t& total_data_points, float& physics_accuracy) const
    {
        total_sessions = session_manager_->getAllSessions().size();
        total_data_points = total_data_points_;
        
        float avg_accuracy;
        uint32_t total_analyses, brake_analyses;
        physics_analyzer_->getAnalysisStats(avg_accuracy, total_analyses, brake_analyses);
        physics_accuracy = avg_accuracy;
    }

    void DataModeling::setDataCallback(std::function<void(const BehaviorDataPoint&)> callback)
    {
        data_callback_ = callback;
    }

    void DataModeling::configurePhysicsModel(float brake_coefficient, float friction_coefficient, uint32_t reaction_time_ms)
    {
        physics_analyzer_->setModelParameters(brake_coefficient, friction_coefficient, reaction_time_ms);
    }

    uint32_t DataModeling::getNextSessionId() const
    {
        return session_manager_->getNextSessionId();
    }

    void DataModeling::clearAllData()
    {
        behavior_data_.clear();
        total_data_points_ = 0;
        session_manager_->clearAllSessions();
        physics_analyzer_->resetStats();
        
        ESP_LOGI(TAG, "Cleared all data");
    }

    BehaviorDataPoint DataModeling::convertRawData(const std::vector<digitoys::datalogger::DataEntry>& raw_data)
    {
        BehaviorDataPoint data_point;
        
        // Convert raw data entries to structured data
        for (const auto& entry : raw_data)
        {
            if (entry.key == "rc_input")
            {
                data_point.rc_control.rc_input = std::stof(entry.value);
            }
            else if (entry.key == "driving_forward")
            {
                data_point.rc_control.driving_forward = (entry.value == "true" || entry.value == "1");
            }
            else if (entry.key == "obstacle_distance")
            {
                data_point.safety_data.obstacle_distance = std::stof(entry.value);
            }
            else if (entry.key == "brake_distance")
            {
                data_point.safety_data.brake_distance = std::stof(entry.value);
            }
            else if (entry.key == "is_obstacle_state")
            {
                data_point.safety_data.is_obstacle_state = (entry.value == "true" || entry.value == "1");
            }
            else if (entry.key == "is_warning_state")
            {
                data_point.safety_data.is_warning_state = (entry.value == "true" || entry.value == "1");
            }
            // Add more field mappings as needed
        }
        
        return data_point;
    }

    std::string DataModeling::generateCSVHeader(const ExportConfig& config) const
    {
        std::ostringstream header;
        
        // Always include basic fields
        header << "session_id,timestamp_ms";
        
        if (config.export_rc_control)
        {
            header << ",rc_input,driving_forward,wants_reverse,throttle_pressed";
        }
        
        if (config.export_safety_data)
        {
            header << ",obstacle_distance,brake_distance,warning_distance,safety_margin,is_braking,is_warning";
        }
        
        if (config.export_physics_data)
        {
            header << ",calculated_speed,deceleration,stopping_distance,stopping_time,physics_accuracy";
        }
        
        if (config.export_session_data)
        {
            header << ",brake_event_id,stop_event_id,scenario_description";
        }
        
        return header.str();
    }

    std::string DataModeling::behaviorDataToCSV(const BehaviorDataPoint& data_point, const ExportConfig& config) const
    {
        std::ostringstream row;
        
        // Always include basic fields
        row << data_point.session_data.session_id << "," 
            << data_point.session_data.session_relative_ms;
        
        if (config.export_rc_control)
        {
            row << "," << std::fixed << std::setprecision(4) << data_point.rc_control.rc_input
                << "," << (data_point.rc_control.driving_forward ? "1" : "0")
                << "," << (data_point.rc_control.wants_reverse ? "1" : "0")
                << "," << (data_point.rc_control.throttle_pressed ? "1" : "0");
        }
        
        if (config.export_safety_data)
        {
            row << "," << std::fixed << std::setprecision(3) << data_point.safety_data.obstacle_distance
                << "," << data_point.safety_data.brake_distance
                << "," << data_point.safety_data.warning_distance
                << "," << data_point.safety_data.safety_margin
                << "," << (data_point.safety_data.is_obstacle_state ? "1" : "0")
                << "," << (data_point.safety_data.is_warning_state ? "1" : "0");
        }
        
        if (config.export_physics_data)
        {
            row << "," << std::fixed << std::setprecision(4) << data_point.physics_data.calculated_speed
                << "," << data_point.physics_data.deceleration
                << "," << data_point.physics_data.actual_stopping_distance
                << "," << data_point.physics_data.stopping_time
                << "," << data_point.physics_data.physics_accuracy;
        }
        
        if (config.export_session_data)
        {
            row << "," << data_point.session_data.brake_event_id
                << "," << data_point.session_data.stop_event_id
                << ",\"" << data_point.session_data.scenario_description << "\"";
        }
        
        return row.str();
    }

    std::string DataModeling::generateSessionHeader(uint32_t session_id) const
    {
        const auto* session = session_manager_->getSession(session_id);
        if (!session)
        {
            return "# Session " + std::to_string(session_id) + " - No data available";
        }
        
        std::ostringstream header;
        header << "# Session " << session->session_id << ": " << session->description << "\n";
        header << "# Start: " << session->start_timestamp_us << " us\n";
        header << "# Duration: " << session->duration_ms << " ms\n";
        header << "# Samples: " << session->sample_count << "\n";
        header << "# Brake Events: " << session->brake_event_count << "\n";
        header << "# Stop Events: " << session->stop_event_count << "\n";
        header << "# Max Speed: " << std::fixed << std::setprecision(2) << session->max_speed << " m/s\n";
        header << "# Total Distance: " << std::fixed << std::setprecision(1) << session->total_distance << " m";
        
        return header.str();
    }

} // namespace digitoys::datamodeling
