// adas_pwm_driver.hpp
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

    struct PwmChannelConfig
    {
        gpio_num_t rx_gpio;
        gpio_num_t tx_gpio;
        ledc_channel_t ledc_channel;
        ledc_timer_t ledc_timer;
        uint32_t pwm_freq_hz = 62;
    };

    class IPwmChannel
    {
    public:
        virtual ~IPwmChannel() = default;
        virtual esp_err_t setDuty(float duty) = 0;
    };

    class RmtInput
    {
    public:
        using DutyCallback = std::function<void(float)>;
        explicit RmtInput(const PwmChannelConfig &cfg);
        ~RmtInput();

        esp_err_t start(DutyCallback cb);
        esp_err_t stop();

        /// Read current duty cycle directly (for getting fresh RC input)
        float readCurrentDuty(uint32_t timeout_ms = 100);

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
        bool running_ = false;

        // For direct duty reading
        float latest_duty_ = -1.0f;
    };

    class LedcOutput : public IPwmChannel
    {
    public:
        explicit LedcOutput(const PwmChannelConfig &cfg);
        ~LedcOutput() override = default;

        esp_err_t setDuty(float duty) override;

    private:
        const PwmChannelConfig cfg_;
        ledc_channel_t channel_;
        ledc_mode_t speed_mode_ = LEDC_LOW_SPEED_MODE;
        uint32_t max_duty_;
    };

    class PwmPassthroughChannel
    {
    public:
        explicit PwmPassthroughChannel(const PwmChannelConfig &cfg);
        ~PwmPassthroughChannel();

        esp_err_t start();
        esp_err_t stop();
        esp_err_t setDuty(float duty);

        /// Return last duty captured by RMT input (absolute duty ratio).
        float lastDuty() const { return last_duty_; }

        /// Check if throttle is pressed outside neutral range.
        /// @param center duty ratio considered neutral (measured from RC)
        /// @param range width of neutral band (default 0.8% for sensitivity)
        bool throttlePressed(float center = 0.0856f, float range = 0.008f) const;

        /// Read current duty directly from RMT (gets fresh RC input)
        float readCurrentDuty(uint32_t timeout_ms = 100);

    private:
        PwmChannelConfig cfg_;
        std::unique_ptr<RmtInput> input_;
        std::unique_ptr<LedcOutput> output_;
        float last_duty_ = 0.0f;
        bool running_ = false;
    };

    class PwmDriver
    {
    public:
        explicit PwmDriver(std::vector<PwmChannelConfig> configs);
        ~PwmDriver() = default;

        esp_err_t initialize();
        esp_err_t shutdown();
        esp_err_t setDuty(size_t idx, float duty);

    public:
        /// Stop only the RMT input for channel idx
        esp_err_t pausePassthrough(size_t idx)
        {
            if (idx >= channels_.size())
                return ESP_ERR_INVALID_ARG;
            return channels_[idx]->stop(); // only stops the RMT task
        }

        /// Restart only the RMT input for channel idx
        esp_err_t resumePassthrough(size_t idx)
        {
            if (idx >= channels_.size())
                return ESP_ERR_INVALID_ARG;
            return channels_[idx]->start(); // leaves LEDC untouched
        }

        /// Return last duty captured for channel idx
        float lastDuty(size_t idx) const
        {
            if (idx >= channels_.size())
                return 0.0f;
            return channels_[idx]->lastDuty();
        }

        /// Return true if throttle channel idx is pressed outside neutral band
        bool isThrottlePressed(size_t idx,
                               float center = 0.0856f,
                               float range = 0.008f) const
        {
            if (idx >= channels_.size())
                return false;
            return channels_[idx]->throttlePressed(center, range);
        }

        /// Read current RC input directly (bypasses passthrough state)
        float readCurrentDutyInput(size_t idx, uint32_t timeout_ms = 100)
        {
            if (idx >= channels_.size())
                return -1.0f;
            return channels_[idx]->readCurrentDuty(timeout_ms);
        }

    private:
        std::vector<PwmChannelConfig> configs_;
        std::vector<std::unique_ptr<PwmPassthroughChannel>> channels_;
    };

} // namespace adas