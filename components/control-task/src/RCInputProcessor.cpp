#include "RCInputProcessor.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "RC_PROCESSOR";

namespace control
{

    RCInputProcessor::RCStatus RCInputProcessor::processRCInput(adas::PwmDriver &driver) const
    {
        RCStatus status;

        // Get current RC input for direction detection
        status.current_input = driver.readCurrentDutyInput(0, 50);
        if (status.current_input < 0)
        {
            status.current_input = driver.lastDuty(0); // Fallback
        }

        // Analyze RC input
        status.driving_forward = isDrivingForward(status.current_input);
        status.wants_reverse = wantsReverse(status.current_input);
        status.at_neutral = isAtNeutral(status.current_input);
        status.throttle_pressed = isThrottlePressed(status.current_input);

        // Get cached readings for diagnostics
        status.cached_duty = driver.lastDuty(0);
        status.direct_duty = status.current_input;
        status.cached_throttle = driver.isThrottlePressed(0);

        return status;
    }

    bool RCInputProcessor::shouldCheckRCDuringBrake(uint32_t current_time, uint32_t last_check_time) const
    {
        return (current_time - last_check_time) >= ControlConstants::RC_CHECK_INTERVAL_MS;
    }

    RCInputProcessor::RCStatus RCInputProcessor::performRCCheck(adas::PwmDriver &driver) const
    {
        ESP_LOGI(TAG, "Checking RC input during brake/warning...");

        // Temporarily resume passthrough to get fresh RC reading
        driver.resumePassthrough(0);
        vTaskDelay(pdMS_TO_TICKS(ControlConstants::RC_READ_DELAY_MS)); // Brief delay to get fresh reading

        RCStatus status = processRCInput(driver);

        ESP_LOGI(TAG, "RC check: duty=%.4f, reverse=%s, neutral=%s",
                 status.current_input,
                 status.wants_reverse ? "YES" : "NO",
                 status.at_neutral ? "YES" : "NO");

        return status;
    }

    void RCInputProcessor::logDiagnostics(const RCStatus &status, int &log_counter) const
    {
        if (++log_counter >= ControlConstants::DUTY_TEST_LOG_INTERVAL)
        {
            log_counter = 0;

            ESP_LOGI(TAG, "DUTY_TEST: cached=%.4f, direct=%.4f, old_throttle=%s, new_throttle=%s, forward=%s, reverse=%s",
                     status.cached_duty, status.direct_duty,
                     status.cached_throttle ? "YES" : "NO",
                     status.throttle_pressed ? "YES" : "NO",
                     status.driving_forward ? "YES" : "NO",
                     status.wants_reverse ? "YES" : "NO");
        }
    }

    bool RCInputProcessor::isDrivingForward(float duty_cycle) const
    {
        return duty_cycle > ControlConstants::ZERO_SPEED + ControlConstants::DIRECTION_TOLERANCE;
    }

    bool RCInputProcessor::wantsReverse(float duty_cycle) const
    {
        return duty_cycle < ControlConstants::ZERO_SPEED - ControlConstants::DIRECTION_TOLERANCE;
    }

    bool RCInputProcessor::isAtNeutral(float duty_cycle) const
    {
        return duty_cycle >= ControlConstants::ZERO_SPEED - ControlConstants::DIRECTION_TOLERANCE &&
               duty_cycle <= ControlConstants::ZERO_SPEED + ControlConstants::DIRECTION_TOLERANCE;
    }

    bool RCInputProcessor::isThrottlePressed(float duty_cycle) const
    {
        return wantsReverse(duty_cycle) || isDrivingForward(duty_cycle);
    }

} // namespace control
