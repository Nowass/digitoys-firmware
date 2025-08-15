// app_main.cpp
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "adas_pwm_driver.hpp"
#include "LiDARConfig.hpp"
#include "LiDAR.hpp"
#include "Monitor.hpp"
#include "SystemMonitor.hpp"
#include <limits>

static const char *TAG = "APP_MAIN";
using namespace lidar;

// Control task context
struct ControlContext
{
    lidar::LiDAR *lidar;
    adas::PwmDriver *pwm_driver;
    monitor::Monitor *mon;
};

// ControlTask: reads LiDAR frames and switches passthrough vs. brake
static void ControlTask(void *pv)
{
    auto *ctx = static_cast<ControlContext *>(pv);
    lidar::LiDAR &lidar = *ctx->lidar;
    adas::PwmDriver &driver = *ctx->pwm_driver;

    // State variables for obstacle detection
    static bool obstacle_state = false;
    static bool warning_state = false;
    static float slowdown_duty = 0.0f;
    static float last_distance = 0.0f;
    static uint32_t last_rc_check_time = 0;
    constexpr uint32_t RC_CHECK_INTERVAL_MS = 100; // Check RC input every 100ms during brake

    constexpr float BRAKE = 0.058f;       // full brake duty
    constexpr float ZERO_SPEED = 0.0856f; // neutral duty (measured from RC)
    constexpr float DUTY_STEP = 0.005f;
    constexpr float DIRECTION_TOLERANCE = 0.005f; // tolerance for forward/reverse detection

    // Dynamic braking parameters
    constexpr float MIN_BRAKE_DISTANCE = 0.3f;            // Minimum brake distance (safety)
    constexpr float MAX_BRAKE_DISTANCE = 2.0f;            // Maximum brake distance
    constexpr float MIN_WARNING_DISTANCE = 0.6f;          // Minimum warning distance
    constexpr float MAX_WARNING_DISTANCE = 3.0f;          // Maximum warning distance
    constexpr float HIGH_SPEED_DUTY = ZERO_SPEED + 0.03f; // ~3% above neutral (for scaling)

    // Function to calculate dynamic brake distance based on current speed
    auto calculateBrakeDistance = [](float current_duty) -> float
    {
        if (current_duty <= ZERO_SPEED + DIRECTION_TOLERANCE)
        {
            return MIN_BRAKE_DISTANCE; // At neutral/reverse - use minimum
        }

        // Normalize duty to speed factor (0.0 to 1.0)
        float speed_factor = (current_duty - ZERO_SPEED) / (HIGH_SPEED_DUTY - ZERO_SPEED);
        speed_factor = std::max(0.0f, std::min(1.0f, speed_factor)); // Clamp to [0,1]

        // Linear interpolation between min and max brake distance
        return MIN_BRAKE_DISTANCE + speed_factor * (MAX_BRAKE_DISTANCE - MIN_BRAKE_DISTANCE);
    };

    // Function to calculate dynamic warning distance
    auto calculateWarningDistance = [](float current_duty) -> float
    {
        if (current_duty <= ZERO_SPEED + DIRECTION_TOLERANCE)
        {
            return MIN_WARNING_DISTANCE;
        }

        float speed_factor = (current_duty - ZERO_SPEED) / (HIGH_SPEED_DUTY - ZERO_SPEED);
        speed_factor = std::max(0.0f, std::min(1.0f, speed_factor));

        return MIN_WARNING_DISTANCE + speed_factor * (MAX_WARNING_DISTANCE - MIN_WARNING_DISTANCE);
    };

    while (true)
    {
        auto info = lidar.getObstacleInfo();
        ctx->mon->updateTelemetry(info.obstacle, info.distance, driver.lastDuty(0), info.warning);

        // Get current RC input for direction detection
        float current_rc_input = driver.readCurrentDutyInput(0, 50);
        if (current_rc_input < 0)
        {
            current_rc_input = driver.lastDuty(0); // Fallback
        }
        bool driving_forward = (current_rc_input > ZERO_SPEED + DIRECTION_TOLERANCE);
        bool wants_reverse = (current_rc_input < ZERO_SPEED - DIRECTION_TOLERANCE);

        // Calculate whether throttle is actually pressed using fresh data
        // (isThrottlePressed can be stale when passthrough is paused)
        bool fresh_throttle_pressed = (current_rc_input < ZERO_SPEED - DIRECTION_TOLERANCE) ||
                                      (current_rc_input > ZERO_SPEED + DIRECTION_TOLERANCE);

        // Test direct reading vs cached reading (every 2 seconds)
        static int log_counter = 0;
        bool throttle_pressed = driver.isThrottlePressed(0); // Check once per loop
        if (++log_counter >= 40)
        { // 40 * 50ms = 2 seconds
            log_counter = 0;
            float cached_duty = driver.lastDuty(0);
            float direct_duty = driver.readCurrentDutyInput(0, 50);

            ESP_LOGI(TAG, "DUTY_TEST: cached=%.4f, direct=%.4f, old_throttle=%s, new_throttle=%s, forward=%s, reverse=%s",
                     cached_duty, direct_duty, throttle_pressed ? "YES" : "NO",
                     fresh_throttle_pressed ? "YES" : "NO",
                     (direct_duty > ZERO_SPEED + DIRECTION_TOLERANCE) ? "YES" : "NO",
                     (direct_duty < ZERO_SPEED - DIRECTION_TOLERANCE) ? "YES" : "NO");
        }

        // Calculate dynamic brake and warning distances based on current speed
        float dynamic_brake_distance = calculateBrakeDistance(current_rc_input);
        float dynamic_warning_distance = calculateWarningDistance(current_rc_input);

        // Override LiDAR's hardcoded thresholds with our dynamic ones
        bool dynamic_obstacle = info.distance <= dynamic_brake_distance;
        bool dynamic_warning = !dynamic_obstacle && info.distance <= dynamic_warning_distance;

        // Debug log dynamic thresholds (every 2 seconds)
        static int threshold_log_counter = 0;
        if (driving_forward && ++threshold_log_counter >= 40)
        {
            threshold_log_counter = 0;
            ESP_LOGI(TAG, "Speed: %.4f, BrakeDist: %.2fm, WarnDist: %.2fm, ActualDist: %.2fm",
                     current_rc_input, dynamic_brake_distance, dynamic_warning_distance, info.distance);
        }

        if (fresh_throttle_pressed)
        {
            if (wants_reverse)
            {
                // Allow reverse motion - clear any brake states immediately
                if (obstacle_state || warning_state)
                {
                    obstacle_state = false;
                    warning_state = false;
                    ESP_LOGI(TAG, "Reverse motion - clearing brake states");
                    driver.resumePassthrough(0);
                }
            }
            else if (driving_forward && dynamic_obstacle)
            {
                if (!obstacle_state)
                {
                    obstacle_state = true;
                    warning_state = false;
                    ESP_LOGW(TAG, "Dynamic obstacle! Speed=%.4f, BrakeDist=%.2fm, ActualDist=%.2fm",
                             current_rc_input, dynamic_brake_distance, info.distance);
                    driver.pausePassthrough(0);
                    driver.setDuty(0, BRAKE);
                    last_rc_check_time = esp_timer_get_time() / 1000; // Initialize timer
                }
                else
                {
                    // Brake is active - periodically check RC input for reverse/neutral
                    uint32_t current_time = esp_timer_get_time() / 1000;
                    if ((current_time - last_rc_check_time) >= RC_CHECK_INTERVAL_MS)
                    {
                        // Temporarily resume passthrough to get fresh RC reading
                        ESP_LOGI(TAG, "Checking RC input during brake...");
                        driver.resumePassthrough(0);
                        vTaskDelay(pdMS_TO_TICKS(20)); // Brief delay to get fresh reading

                        float current_rc = driver.lastDuty(0);
                        bool wants_reverse = (current_rc < ZERO_SPEED - DIRECTION_TOLERANCE);
                        bool at_neutral = (current_rc >= ZERO_SPEED - DIRECTION_TOLERANCE &&
                                           current_rc <= ZERO_SPEED + DIRECTION_TOLERANCE);

                        ESP_LOGI(TAG, "RC check: duty=%.4f, reverse=%s, neutral=%s",
                                 current_rc, wants_reverse ? "YES" : "NO", at_neutral ? "YES" : "NO");

                        if (wants_reverse || at_neutral)
                        {
                            // Allow reverse/neutral motion - clear brake
                            obstacle_state = false;
                            ESP_LOGI(TAG, "RC input allows escape - clearing brake");
                            // Passthrough already resumed above
                        }
                        else
                        {
                            // Still forward - reapply brake
                            driver.pausePassthrough(0);
                            driver.setDuty(0, BRAKE);
                        }

                        last_rc_check_time = current_time;
                    }
                }
            }
            else if (driving_forward && dynamic_warning)
            {
                obstacle_state = false;
                if (!warning_state)
                {
                    warning_state = true;
                    slowdown_duty = driver.lastDuty(0);
                    last_distance = info.distance;
                    driver.pausePassthrough(0);
                    last_rc_check_time = esp_timer_get_time() / 1000; // Initialize timer for warning
                    ESP_LOGI(TAG, "Dynamic warning! Speed=%.4f, WarnDist=%.2fm, ActualDist=%.2fm",
                             current_rc_input, dynamic_warning_distance, info.distance);
                }
                else
                {
                    // Warning is active - periodically check RC input for neutral/reverse
                    uint32_t current_time = esp_timer_get_time() / 1000;
                    if ((current_time - last_rc_check_time) >= RC_CHECK_INTERVAL_MS)
                    {
                        // Temporarily resume passthrough to get fresh RC reading
                        ESP_LOGI(TAG, "Checking RC input during warning...");
                        driver.resumePassthrough(0);
                        vTaskDelay(pdMS_TO_TICKS(20)); // Brief delay to get fresh reading

                        float current_rc = driver.lastDuty(0);
                        bool wants_reverse = (current_rc < ZERO_SPEED - DIRECTION_TOLERANCE);
                        bool at_neutral = (current_rc >= ZERO_SPEED - DIRECTION_TOLERANCE &&
                                           current_rc <= ZERO_SPEED + DIRECTION_TOLERANCE);

                        ESP_LOGI(TAG, "RC check during warning: duty=%.4f, reverse=%s, neutral=%s",
                                 current_rc, wants_reverse ? "YES" : "NO", at_neutral ? "YES" : "NO");

                        if (wants_reverse || at_neutral)
                        {
                            // Allow reverse/neutral motion - clear warning
                            warning_state = false;
                            ESP_LOGI(TAG, "RC input allows escape from warning - clearing warning state");
                            // Passthrough already resumed above
                        }
                        else
                        {
                            // Still forward - continue warning behavior
                            driver.pausePassthrough(0);

                            // Apply gradual slowdown during warning
                            // Reduce speed if obstacle isn't getting farther away
                            if (info.distance <= last_distance + 0.1f && slowdown_duty > ZERO_SPEED)
                            {
                                slowdown_duty -= DUTY_STEP;
                                if (slowdown_duty < ZERO_SPEED)
                                    slowdown_duty = ZERO_SPEED;
                                ESP_LOGI(TAG, "Warning slowdown: setting duty to %.4f", slowdown_duty);
                                driver.setDuty(0, slowdown_duty);
                            }
                            else
                            {
                                // Keep current slowdown speed if distance is improving
                                driver.setDuty(0, slowdown_duty);
                            }
                        }

                        last_distance = info.distance;
                        last_rc_check_time = current_time;
                    }
                    else
                    {
                        // Between RC checks - continue normal slowdown logic
                        if (info.distance <= last_distance + 0.1f && slowdown_duty > ZERO_SPEED)
                        {
                            slowdown_duty -= DUTY_STEP;
                            if (slowdown_duty < ZERO_SPEED)
                                slowdown_duty = ZERO_SPEED;
                            ESP_LOGI(TAG, "Warning slowdown (between checks): setting duty to %.4f", slowdown_duty);
                            driver.setDuty(0, slowdown_duty);
                        }
                        else
                        {
                            // Keep current slowdown speed
                            driver.setDuty(0, slowdown_duty);
                        }
                        last_distance = info.distance;
                    }
                }
            }
            else if (driving_forward)
            {
                // Forward motion with no obstacles - clear any brake states
                if (obstacle_state || warning_state)
                {
                    obstacle_state = false;
                    warning_state = false;
                    ESP_LOGI(TAG, "Path clear. Resuming passthrough");
                    driver.resumePassthrough(0);
                }
            }
            // Neutral position - maintain current state
        }
        else
        {
            // Throttle not pressed (neutral) - clear any active brake/warning states
            if (obstacle_state || warning_state)
            {
                obstacle_state = false;
                warning_state = false;
                ESP_LOGI(TAG, "Throttle neutral - clearing all brake states");
                driver.resumePassthrough(0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main()
{
    // --- LiDAR hardware setup ---
    LiDARConfig cfg = {
        .uartPort = UART_NUM_1,
        .txPin = GPIO_NUM_10,
        .rxPin = GPIO_NUM_11,
        .dmaBufferLen = 2048,
        .angleMinDeg = 347.5f,
        .angleMaxDeg = 12.5f,
        .motorPin = GPIO_NUM_3,
        .motorChannel = LEDC_CHANNEL_1,
        .motorFreqHz = 50000,
        .motorDutyPct = 50};
    static lidar::LiDAR lidar{cfg};
    ESP_ERROR_CHECK(lidar.initialize());
    ESP_LOGI(TAG, "LiDAR initialized");

    // --- PWM passthrough driver (throttle only) ---
    adas::PwmChannelConfig thr_cfg = {GPIO_NUM_18, GPIO_NUM_19,
                                      LEDC_CHANNEL_0, LEDC_TIMER_0, 62};
    std::vector<adas::PwmChannelConfig> configs = {thr_cfg};
    static adas::PwmDriver pwm_driver{configs};
    ESP_ERROR_CHECK(pwm_driver.initialize());
    ESP_LOGI(TAG, "PWM passthrough running");

    // --- Telemetry monitor ---
    static monitor::Monitor mon;
    ESP_ERROR_CHECK(mon.start());
    ESP_LOGI(TAG, "Monitor started");

    // --- System monitor ---
    static monitor::SystemMonitor sys_mon;
    ESP_ERROR_CHECK(sys_mon.start());
    ESP_LOGI(TAG, "System monitor started");

    // --- Launch ControlTask ---
    static ControlContext ctx = {&lidar, &pwm_driver, &mon};
    BaseType_t rc = xTaskCreate(
        ControlTask, "ControlTask", 8192,
        &ctx, tskIDLE_PRIORITY + 2, nullptr);
    ESP_ERROR_CHECK(rc == pdPASS ? ESP_OK : ESP_FAIL);

    // Keep app_main idle
    vTaskDelete(nullptr);
}
