#include "SpeedController.hpp"
#include <esp_log.h>
#include <esp_timer.h>

static const char *TAG = "SPEED_CONTROLLER";

namespace control
{

    SpeedController::Action SpeedController::handleReverseMotion(ControlState &state)
    {
        if (state.isObstacleState() || state.isWarningState())
        {
            state.clearAllStates();
            ESP_LOGI(TAG, "Reverse motion - clearing brake states");
            return Action::RESUME_PASSTHROUGH;
        }
        return Action::NO_ACTION;
    }

    SpeedController::Action SpeedController::handleObstacleState(ControlState &state,
                                                                 const RCInputProcessor::RCStatus &rc_status,
                                                                 uint32_t current_time,
                                                                 adas::PwmDriver &driver,
                                                                 const RCInputProcessor &rc_processor)
    {
        if (!state.isObstacleState())
        {
            // Initialize obstacle state
            state.initializeObstacleState(current_time);
            ESP_LOGW(TAG, "Dynamic obstacle! Speed=%.4f, ActualDist=%.2fm",
                     rc_status.current_input, 0.0f); // Distance logged elsewhere
            return Action::APPLY_BRAKE;
        }
        else
        {
            // Brake is active - periodically check RC input for reverse/neutral
            if (rc_processor.shouldCheckRCDuringBrake(current_time, state.getLastRCCheckTime()))
            {
                auto rc_check = rc_processor.performRCCheck(driver);

                if (allowsEscape(rc_check))
                {
                    // Allow reverse/neutral motion - clear brake
                    state.setObstacleState(false);
                    ESP_LOGI(TAG, "RC input allows escape - clearing brake");
                    return Action::RESUME_PASSTHROUGH;
                }
                else
                {
                    // Still forward - reapply brake
                    state.setLastRCCheckTime(current_time);
                    return Action::APPLY_BRAKE;
                }
            }
        }
        return Action::NO_ACTION;
    }

    SpeedController::Action SpeedController::handleWarningState(ControlState &state,
                                                                const RCInputProcessor::RCStatus &rc_status,
                                                                float distance, uint32_t current_time,
                                                                adas::PwmDriver &driver,
                                                                const RCInputProcessor &rc_processor,
                                                                const ObstacleDetector &obstacle_detector)
    {
        state.setObstacleState(false);

        if (!state.isWarningState())
        {
            // Initialize warning state
            state.initializeWarningState(rc_status.current_input, distance, current_time);
            ESP_LOGI(TAG, "Dynamic warning! Speed=%.4f, ActualDist=%.2fm",
                     rc_status.current_input, distance);
            return Action::GRADUAL_SLOWDOWN;
        }
        else
        {
            // Warning is active - periodically check RC input for neutral/reverse
            if (rc_processor.shouldCheckRCDuringBrake(current_time, state.getLastRCCheckTime()))
            {
                auto rc_check = rc_processor.performRCCheck(driver);

                ESP_LOGI(TAG, "RC check during warning: rc_input=%.4f, slowdown_duty=%.4f, reverse=%s, neutral=%s",
                         rc_check.current_input, state.getSlowdownDuty(),
                         rc_check.wants_reverse ? "YES" : "NO",
                         rc_check.at_neutral ? "YES" : "NO");

                if (allowsEscape(rc_check, state.getSlowdownDuty()))
                {
                    // Allow escape or user reduced throttle to match slowdown
                    state.setWarningState(false);
                    if (rc_check.wants_reverse || rc_check.at_neutral)
                    {
                        ESP_LOGI(TAG, "RC input allows escape from warning - clearing warning state");
                    }
                    else
                    {
                        ESP_LOGI(TAG, "RC input matches slowdown level - resuming normal control");
                    }
                    return Action::RESUME_PASSTHROUGH;
                }
                else
                {
                    // Still forward and above slowdown level - continue warning behavior
                    applyGradualSlowdown(state, distance);
                    state.setLastRCCheckTime(current_time);
                    return Action::GRADUAL_SLOWDOWN;
                }
            }
            else
            {
                // Between RC checks - continue normal slowdown logic
                applyGradualSlowdown(state, distance);

                // Check if we've slowed down enough based on current slowdown duty
                float slowdown_warning_distance = obstacle_detector.calculateWarningDistance(state.getSlowdownDuty());
                if (distance > slowdown_warning_distance)
                {
                    ESP_LOGI(TAG, "Slowed down enough: actual=%.2fm > warning=%.2fm for duty=%.4f - maintaining warning",
                             distance, slowdown_warning_distance, state.getSlowdownDuty());
                }

                return Action::GRADUAL_SLOWDOWN;
            }
        }
        return Action::NO_ACTION;
    }

    SpeedController::Action SpeedController::handleClearPath(ControlState &state)
    {
        if (state.isObstacleState() || state.isWarningState())
        {
            state.clearAllStates();
            ESP_LOGI(TAG, "Path clear. Resuming passthrough");
            return Action::RESUME_PASSTHROUGH;
        }
        return Action::NO_ACTION;
    }

    SpeedController::Action SpeedController::handleNeutralThrottle(ControlState &state)
    {
        if (state.isObstacleState() || state.isWarningState())
        {
            state.clearAllStates();
            ESP_LOGI(TAG, "Throttle neutral - clearing all brake states");
            return Action::RESUME_PASSTHROUGH;
        }
        return Action::NO_ACTION;
    }

    void SpeedController::executeAction(Action action, adas::PwmDriver &driver, ControlState &state,
                                        float slowdown_duty)
    {
        switch (action)
        {
        case Action::RESUME_PASSTHROUGH:
            driver.resumePassthrough(0);
            break;

        case Action::APPLY_BRAKE:
            driver.pausePassthrough(0);
            driver.setDuty(0, ControlConstants::BRAKE);
            break;

        case Action::GRADUAL_SLOWDOWN:
            driver.pausePassthrough(0);
            if (slowdown_duty != 0.0f)
            {
                driver.setDuty(0, slowdown_duty);
            }
            else
            {
                driver.setDuty(0, state.getSlowdownDuty());
            }
            break;

        case Action::MAINTAIN_SPEED:
            driver.setDuty(0, state.getSlowdownDuty());
            break;

        case Action::CLEAR_WARNING:
            state.setWarningState(false);
            driver.resumePassthrough(0);
            break;

        case Action::NO_ACTION:
        default:
            // No action needed
            break;
        }
    }

    float SpeedController::applyGradualSlowdown(ControlState &state, float distance)
    {
        float slowdown_duty = state.getSlowdownDuty();

        // Reduce speed if obstacle isn't getting farther away
        if (distance <= state.getLastDistance() + 0.1f && slowdown_duty > ControlConstants::ZERO_SPEED)
        {
            slowdown_duty -= ControlConstants::DUTY_STEP;
            if (slowdown_duty < ControlConstants::ZERO_SPEED)
            {
                slowdown_duty = ControlConstants::ZERO_SPEED;
            }
            state.setSlowdownDuty(slowdown_duty);
            ESP_LOGI(TAG, "Warning slowdown: setting duty to %.4f", slowdown_duty);
        }
        else
        {
            // Keep current slowdown speed if distance is improving
            ESP_LOGI(TAG, "Warning maintaining: duty=%.4f", slowdown_duty);
        }

        state.setLastDistance(distance);
        return slowdown_duty;
    }

    bool SpeedController::allowsEscape(const RCInputProcessor::RCStatus &rc_status, float slowdown_duty) const
    {
        // Allow reverse/neutral motion
        if (rc_status.wants_reverse || rc_status.at_neutral)
        {
            return true;
        }

        // For warning state, also allow if RC input dropped to slowdown level (+tolerance)
        if (slowdown_duty > 0.0f && rc_status.current_input <= slowdown_duty + 0.01f)
        {
            return true;
        }

        return false;
    }

} // namespace control
