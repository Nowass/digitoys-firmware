/**
 * @file adas_pwm_driver.hpp
 * @brief PWM passthrough driver built on RMT and LEDC.
 *
 * Classes in this file capture RC style PWM signals using the RMT peripheral
 * and forward them to servos/ESCs via the LEDC generator.  The driver can also
 * be paused or overridden by higher level logic.
 */
#pragma once

#include <driver/rmt_types.h>
#include <driver/rmt_rx.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <array>
#include <memory>
#include <functional>
#include <vector>

namespace adas
{

    /**
     * @brief Configuration for a single PWM passthrough channel.
     *
     * Each channel captures a PWM input on @p rx_gpio and drives an output on
     * @p tx_gpio using the LEDC peripheral.
     */
    struct PwmChannelConfig
    {
        gpio_num_t rx_gpio;      ///< GPIO receiving the PWM signal
        gpio_num_t tx_gpio;      ///< GPIO driving the servo/ESC
        ledc_channel_t ledc_channel; ///< LEDC channel used for output
        ledc_timer_t ledc_timer;     ///< LEDC timer used for output
        uint32_t pwm_freq_hz = 62;   ///< Expected PWM frequency
    };

    class IPwmChannel
    {
    public:
        virtual ~IPwmChannel() = default;
        virtual esp_err_t setDuty(float duty) = 0;
    };

    /**
     * @brief Captures a PWM input using the RMT peripheral.
     */
    class RmtInput
    {
    public:
        /// Callback invoked when a new duty ratio was measured
        using DutyCallback = std::function<void(float)>;

        /// Construct RMT input channel
        /// @param cfg pin/timer configuration
        explicit RmtInput(const PwmChannelConfig &cfg);
        ~RmtInput();

        /// Start receiving PWM edges and calling @p cb with the duty ratio
        esp_err_t start(DutyCallback cb);
        /// Stop capturing PWM input
        esp_err_t stop();

    private:
        static bool IRAM_ATTR onRecvDone(rmt_channel_handle_t ch,
                                         const rmt_rx_done_event_data_t *evt,
                                         void *ctx);
        void taskLoop();

        const PwmChannelConfig cfg_;
        rmt_channel_handle_t rmt_ch_ = nullptr;
        rmt_receive_config_t recv_cfg_{};
        QueueHandle_t queue_ = nullptr;
        TaskHandle_t task_handle_ = nullptr;
        std::vector<rmt_symbol_word_t> buffer_;
        DutyCallback callback_;
        static constexpr size_t DEFAULT_BUFFER = 64;
    };

    /**
     * @brief Drives a PWM output using the LEDC peripheral.
     */
    class LedcOutput : public IPwmChannel
    {
    public:
        /// Construct LEDC output driver
        /// @param cfg configuration for output pin/channel
        explicit LedcOutput(const PwmChannelConfig &cfg);
        ~LedcOutput() override = default;

        /// Set duty ratio to output
        /// @param duty normalized duty in range [0,1]
        /// @return ESP_OK on success
        esp_err_t setDuty(float duty) override;

    private:
        const PwmChannelConfig cfg_;
        ledc_channel_t channel_;
        ledc_mode_t speed_mode_ = LEDC_LOW_SPEED_MODE;
        uint32_t max_duty_;
    };

    /**
     * @brief Convenience wrapper combining one input and one output channel.
     */
    class PwmPassthroughChannel
    {
    public:
        /// Construct passthrough channel
        /// @param cfg channel configuration
        explicit PwmPassthroughChannel(const PwmChannelConfig &cfg);
        ~PwmPassthroughChannel();

        /// Start capturing input and forwarding to output
        esp_err_t start();
        /// Stop capturing input
        esp_err_t stop();
        /// Override duty ratio manually
        esp_err_t setDuty(float duty);

        /// Return last duty captured by RMT input (absolute duty ratio).
        float lastDuty() const { return last_duty_; }

        /// Check if throttle is pressed outside neutral range.
        /// @param center duty ratio considered neutral (default 9%)
        /// @param range width of neutral band (default 1%)
        bool throttlePressed(float center = 0.09f, float range = 0.01f) const;

    private:
        PwmChannelConfig cfg_;
        std::unique_ptr<RmtInput> input_;
        std::unique_ptr<LedcOutput> output_;
        float last_duty_ = 0.0f;
    };

    /**
     * @brief Manager for multiple passthrough channels.
     */
    class PwmDriver
    {
    public:
        /// Construct driver with a list of channel configurations
        explicit PwmDriver(std::vector<PwmChannelConfig> configs);
        ~PwmDriver() = default;

        /// Initialize hardware for all channels
        esp_err_t initialize();
        /// Stop all channels and free resources
        esp_err_t shutdown();
        /// Set duty ratio of channel @p idx manually
        esp_err_t setDuty(size_t idx, float duty);

    public:
        /// Stop only the RMT input for channel @p idx
        esp_err_t pausePassthrough(size_t idx)
        {
            if (idx >= channels_.size())
                return ESP_ERR_INVALID_ARG;
            return channels_[idx]->stop(); // only stops the RMT task
        }

        /// Restart only the RMT input for channel @p idx
        esp_err_t resumePassthrough(size_t idx)
        {
            if (idx >= channels_.size())
                return ESP_ERR_INVALID_ARG;
            return channels_[idx]->start(); // leaves LEDC untouched
        }

        /// Return true if throttle channel @p idx is pressed outside neutral band
        bool isThrottlePressed(size_t idx,
                              float center = 0.09f,
                              float range = 0.01f) const
        {
            if (idx >= channels_.size())
                return false;
            return channels_[idx]->throttlePressed(center, range);
        }

    private:
        std::vector<PwmChannelConfig> configs_;
        std::vector<std::unique_ptr<PwmPassthroughChannel>> channels_;
    };

} // namespace adas
