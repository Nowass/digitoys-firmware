#include "TestSession.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <algorithm>

namespace digitoys::datamodeling
{
    const char* TestSessionManager::TAG = "TestSessionManager";

    TestSessionManager::TestSessionManager()
    {
        ESP_LOGI(TAG, "TestSession manager initialized");
    }

    uint32_t TestSessionManager::startSession(const std::string& description)
    {
        // Stop any existing session first
        if (isSessionActive())
        {
            ESP_LOGW(TAG, "Stopping previous session %lu before starting new one", current_session_id_);
            stopSession();
        }

        // Create new session
        digitoys::datamodeling::TestSessionData new_session;
        new_session.session_id = next_session_id_;
        new_session.description = description;
        new_session.start_timestamp_us = esp_timer_get_time();
        new_session.is_active = true;

        // Reset event counters
        current_brake_event_id_ = 0;
        current_stop_event_id_ = 0;

        // Add to sessions list
        sessions_.push_back(new_session);
        current_session_id_ = next_session_id_;
        next_session_id_++;

        ESP_LOGI(TAG, "Started session %lu: '%s'", current_session_id_, description.c_str());
        return current_session_id_;
    }

    esp_err_t TestSessionManager::stopSession()
    {
        if (!isSessionActive())
        {
            ESP_LOGW(TAG, "No active session to stop");
            return ESP_ERR_INVALID_STATE;
        }

        // Find current session and update it
        int session_index = findSessionIndex(current_session_id_);
        if (session_index >= 0)
        {
            auto& session = sessions_[session_index];
            session.end_timestamp_us = esp_timer_get_time();
            session.is_active = false;
            session.duration_ms = (session.end_timestamp_us - session.start_timestamp_us) / 1000;
            session.brake_event_count = current_brake_event_id_;
            session.stop_event_count = current_stop_event_id_;

            ESP_LOGI(TAG, "Stopped session %lu: duration=%lums, brake_events=%lu, stop_events=%lu",
                     current_session_id_, session.duration_ms, session.brake_event_count, session.stop_event_count);
        }

        current_session_id_ = 0;
        return ESP_OK;
    }

    uint32_t TestSessionManager::markStopEvent()
    {
        if (!isSessionActive())
        {
            ESP_LOGW(TAG, "No active session for stop event");
            return 0;
        }

        current_stop_event_id_++;
        ESP_LOGI(TAG, "Marked stop event %lu in session %lu", current_stop_event_id_, current_session_id_);
        return current_stop_event_id_;
    }

    uint32_t TestSessionManager::markBrakeEvent()
    {
        if (!isSessionActive())
        {
            ESP_LOGW(TAG, "No active session for brake event");
            return 0;
        }

        current_brake_event_id_++;
        ESP_LOGI(TAG, "Marked brake event %lu in session %lu", current_brake_event_id_, current_session_id_);
        return current_brake_event_id_;
    }

    const digitoys::datamodeling::TestSessionData* TestSessionManager::getCurrentSession() const
    {
        if (!isSessionActive())
        {
            return nullptr;
        }

        int session_index = findSessionIndex(current_session_id_);
        if (session_index >= 0)
        {
            return &sessions_[session_index];
        }

        return nullptr;
    }

    const digitoys::datamodeling::TestSessionData* TestSessionManager::getSession(uint32_t session_id) const
    {
        int session_index = findSessionIndex(session_id);
        if (session_index >= 0)
        {
            return &sessions_[session_index];
        }

        return nullptr;
    }

    std::vector<digitoys::datamodeling::TestSessionData> TestSessionManager::getAllSessions() const
    {
        return sessions_;
    }

    void TestSessionManager::updateSessionStats(const BehaviorDataPoint& data_point)
    {
        if (!isSessionActive())
        {
            return;
        }

        int session_index = findSessionIndex(current_session_id_);
        if (session_index >= 0)
        {
            auto& session = sessions_[session_index];
            session.sample_count++;

            // Update maximum speed
            if (data_point.physics_data.calculated_speed > session.max_speed)
            {
                session.max_speed = data_point.physics_data.calculated_speed;
            }

            // Update total distance (accumulate distance deltas)
            if (data_point.physics_data.distance_delta > 0)
            {
                session.total_distance += data_point.physics_data.distance_delta;
            }
        }
    }

    bool TestSessionManager::isSessionActive() const
    {
        return current_session_id_ != 0;
    }

    uint32_t TestSessionManager::getCurrentSessionId() const
    {
        return current_session_id_;
    }

    uint64_t TestSessionManager::getSessionRelativeTime(uint64_t absolute_timestamp_us) const
    {
        if (!isSessionActive())
        {
            return 0;
        }

        const auto* session = getCurrentSession();
        if (session)
        {
            return (absolute_timestamp_us - session->start_timestamp_us) / 1000; // Convert to milliseconds
        }

        return 0;
    }

    uint32_t TestSessionManager::getNextSessionId() const
    {
        return next_session_id_;
    }

    void TestSessionManager::clearAllSessions()
    {
        sessions_.clear();
        next_session_id_ = 1;
        current_session_id_ = 0;
        current_brake_event_id_ = 0;
        current_stop_event_id_ = 0;

        ESP_LOGI(TAG, "Cleared all session data");
    }

    int TestSessionManager::findSessionIndex(uint32_t session_id) const
    {
        for (size_t i = 0; i < sessions_.size(); i++)
        {
            if (sessions_[i].session_id == session_id)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

} // namespace digitoys::datamodeling
