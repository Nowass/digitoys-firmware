#pragma once

#include "DataModelingTypes.hpp"
#include <esp_err.h>
#include <vector>
#include <memory>

namespace digitoys::datamodeling
{
    /**
     * @brief Test session management and coordination
     * 
     * Handles:
     * - Session lifecycle (start/stop/pause)
     * - Session metadata and statistics
     * - Event tracking (brake events, manual stops)
     * - Data correlation across sessions
     */
    class TestSessionManager
    {
    public:
        TestSessionManager();
        ~TestSessionManager() = default;

        /**
         * @brief Start a new test session
         * @param description Optional description for the session
         * @return Session ID (1,2,3...) or 0 on error
         */
        uint32_t startSession(const std::string& description = "");

        /**
         * @brief Stop the current active session
         * @return ESP_OK on success
         */
        esp_err_t stopSession();

        /**
         * @brief Mark a manual stop event in current session
         * @return Stop event ID or 0 if no active session
         */
        uint32_t markStopEvent();

        /**
         * @brief Mark a brake event in current session
         * @return Brake event ID or 0 if no active session
         */
        uint32_t markBrakeEvent();

        /**
         * @brief Get current active session info
         * @return Pointer to active session or nullptr
         */
        const digitoys::datamodeling::TestSessionData* getCurrentSession() const;

        /**
         * @brief Get session by ID
         * @param session_id Session ID to retrieve
         * @return Pointer to session or nullptr if not found
         */
        const digitoys::datamodeling::TestSessionData* getSession(uint32_t session_id) const;

        /**
         * @brief Get all completed sessions
         * @return Vector of all session metadata
         */
        std::vector<digitoys::datamodeling::TestSessionData> getAllSessions() const;

        /**
         * @brief Update session statistics
         * @param data_point Latest behavior data point
         */
        void updateSessionStats(const BehaviorDataPoint& data_point);

        /**
         * @brief Check if a session is currently active
         * @return true if session is running
         */
        bool isSessionActive() const;

        /**
         * @brief Get current session ID
         * @return Current session ID or 0 if none active
         */
        uint32_t getCurrentSessionId() const;

        /**
         * @brief Generate session-relative timestamp
         * @param absolute_timestamp_us Absolute timestamp from esp_timer
         * @return Milliseconds since current session start, or 0 if no session
         */
        uint64_t getSessionRelativeTime(uint64_t absolute_timestamp_us) const;

        /**
         * @brief Get next session ID (for preview/planning)
         * @return Next session ID that would be assigned
         */
        uint32_t getNextSessionId() const;

        /**
         * @brief Clear all session data (for testing/reset)
         */
        void clearAllSessions();

    private:
        static const char* TAG;

        std::vector<digitoys::datamodeling::TestSessionData> sessions_;
        uint32_t next_session_id_ = 1;
        uint32_t current_session_id_ = 0;

        // Event counters for current session
        uint32_t current_brake_event_id_ = 0;
        uint32_t current_stop_event_id_ = 0;

        /**
         * @brief Find session index by ID
         * @param session_id Session ID to find
         * @return Index in sessions_ vector or -1 if not found
         */
        int findSessionIndex(uint32_t session_id) const;
    };

} // namespace digitoys::datamodeling
