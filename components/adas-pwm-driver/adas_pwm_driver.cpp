// adas_pwm_driver.cpp
#include "adas_pwm_driver.hpp"

namespace adas
{

    // ------------ RmtInput ------------
    RmtInput::RmtInput(const PwmChannelConfig &cfg)
        : cfg_(cfg),
          buffer_(DEFAULT_BUFFER)
    {
        rmt_rx_channel_config_t rx_cfg = {
            .gpio_num = cfg_.rx_gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 1 * 1000 * 1000,
            .mem_block_symbols = DEFAULT_BUFFER,
            .intr_priority = 2,
            .flags = {.invert_in = false, .with_dma = false, .allow_pd = false},
        };
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rmt_ch_));
    }

    RmtInput::~RmtInput()
    {
        stop();
        if (rmt_ch_)
        {
            rmt_del_channel(rmt_ch_);
        }
    }

    esp_err_t RmtInput::start(DutyCallback cb)
    {
        if (running_)
        {
            return ESP_OK;
        }
        callback_ = std::move(cb);
        queue_ = xQueueCreate(3, sizeof(rmt_rx_done_event_data_t));
        rmt_rx_event_callbacks_t cbs = {.on_recv_done = onRecvDone};
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmt_ch_, &cbs, queue_));
        ESP_ERROR_CHECK(rmt_enable(rmt_ch_));

        recv_cfg_ = {.signal_range_min_ns = 500,
                     .signal_range_max_ns = 10 * 1000 * 1000,
                     .flags = {.en_partial_rx = 1}};
        ESP_ERROR_CHECK(rmt_receive(rmt_ch_, buffer_.data(), buffer_.size() * sizeof(rmt_symbol_word_t), &recv_cfg_));

        BaseType_t rc = xTaskCreate(
            [](void *arg)
            { static_cast<RmtInput *>(arg)->taskLoop(); },
            "rmt_in_task",
            4096,
            this,
            tskIDLE_PRIORITY + 1,
            &task_handle_);
        if (rc != pdPASS)
        {
            ESP_LOGE("RmtInput", "Failed to create RMT task");
            return ESP_FAIL;
        }

        running_ = true;
        return ESP_OK;

        // return xTaskCreate(
        //     [](void *arg)
        //     { static_cast<RmtInput *>(arg)->taskLoop(); },
        //     "rmt_in_task", 4096, this, tskIDLE_PRIORITY + 1, &task_handle_);
    }

    esp_err_t RmtInput::stop()
    {
        if (!running_)
        {
            return ESP_OK;
        }
        if (task_handle_)
        {
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }
        if (rmt_ch_)
        {
            ESP_ERROR_CHECK(rmt_disable(rmt_ch_));
        }
        if (queue_)
        {
            vQueueDelete(queue_);
            queue_ = nullptr;
        }
        running_ = false;
        return ESP_OK;
    }

    bool IRAM_ATTR RmtInput::onRecvDone(rmt_channel_handle_t ch,
                                        const rmt_rx_done_event_data_t *evt,
                                        void *ctx)
    {
        BaseType_t woken;
        xQueueSendFromISR((QueueHandle_t)ctx, evt, &woken);
        return woken == pdTRUE;
    }

    void RmtInput::taskLoop()
    {
        rmt_rx_done_event_data_t evt;
        while (xQueueReceive(queue_, &evt, portMAX_DELAY))
        {
            auto &s = evt.received_symbols[0];
            float duty = s.duration0 / 16129.0f;

            // Store latest duty for direct reading
            latest_duty_ = duty;

            callback_(duty);
            // re-arm
            rmt_receive(rmt_ch_, buffer_.data(), buffer_.size() * sizeof(rmt_symbol_word_t), &recv_cfg_);
        }
    }

    float RmtInput::readCurrentDuty(uint32_t timeout_ms)
    {
        // Simple implementation: return the latest captured duty
        // In a real implementation, you might want to check if the data is fresh
        return latest_duty_;
    }

    // ------------ LedcOutput ------------
    LedcOutput::LedcOutput(const PwmChannelConfig &cfg)
        : cfg_(cfg), channel_(cfg.ledc_channel)
    {
        ledc_timer_config_t tcfg = {
            .speed_mode = speed_mode_,
            .duty_resolution = LEDC_TIMER_15_BIT,
            .timer_num = cfg_.ledc_timer,
            .freq_hz = cfg_.pwm_freq_hz,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

        ledc_channel_config_t ccfg = {
            .gpio_num = cfg_.tx_gpio,
            .speed_mode = speed_mode_,
            .channel = channel_,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = cfg_.ledc_timer,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
        max_duty_ = (1 << LEDC_TIMER_15_BIT) - 1;
    }

    esp_err_t LedcOutput::setDuty(float duty)
    {
        duty = duty < 0 ? 0 : (duty > 1 ? 1 : duty);
        uint32_t d = static_cast<uint32_t>(duty * max_duty_ + 0.5f);
        ESP_ERROR_CHECK(ledc_set_duty(speed_mode_, channel_, d));
        ESP_ERROR_CHECK(ledc_update_duty(speed_mode_, channel_));
        return ESP_OK;
    }

    // ------------ PwmPassthroughChannel ------------
    PwmPassthroughChannel::PwmPassthroughChannel(const PwmChannelConfig &cfg)
        : cfg_(cfg),
          input_(new RmtInput(cfg)),
          output_(new LedcOutput(cfg))
    {
    }

    PwmPassthroughChannel::~PwmPassthroughChannel()
    {
        stop();
    }

    esp_err_t PwmPassthroughChannel::start()
    {
        if (running_)
        {
            return ESP_OK;
        }
        esp_err_t err = input_->start([this](float duty)
                                      {
                                      last_duty_ = duty;
                                      output_->setDuty(duty); });
        if (err == ESP_OK)
        {
            running_ = true;
        }
        return err;
    }

    esp_err_t PwmPassthroughChannel::stop()
    {
        if (!running_)
        {
            return ESP_OK;
        }
        esp_err_t err = input_->stop();
        if (err == ESP_OK)
        {
            running_ = false;
        }
        return err;
    }

    esp_err_t PwmPassthroughChannel::setDuty(float duty)
    {
        return output_->setDuty(duty);
    }

    bool PwmPassthroughChannel::throttlePressed(float center, float range) const
    {
        // Use direct reading to get fresh RC input (bypasses passthrough state)
        float current_duty = input_->readCurrentDuty(10); // Quick 10ms timeout
        if (current_duty < 0)
        {
            // Fallback to cached value if direct read fails
            current_duty = last_duty_;
        }

        float lower = center - range * 0.5f;
        float upper = center + range * 0.5f;
        return current_duty < lower || current_duty > upper;
    }

    float PwmPassthroughChannel::readCurrentDuty(uint32_t timeout_ms)
    {
        if (!input_)
        {
            return -1.0f;
        }
        return input_->readCurrentDuty(timeout_ms);
    }

    // ------------ PwmDriver ------------
    PwmDriver::PwmDriver(std::vector<PwmChannelConfig> configs)
        : configs_(std::move(configs))
    {
        for (auto &cfg : configs_)
        {
            channels_.emplace_back(new PwmPassthroughChannel(cfg));
        }
    }

    esp_err_t PwmDriver::initialize()
    {
        for (auto &ch : channels_)
        {
            ESP_ERROR_CHECK(ch->start());
        }
        return ESP_OK;
    }

    esp_err_t PwmDriver::shutdown()
    {
        for (auto &ch : channels_)
        {
            ESP_ERROR_CHECK(ch->stop());
        }
        return ESP_OK;
    }

    esp_err_t PwmDriver::setDuty(size_t idx, float duty)
    {
        if (idx >= channels_.size())
        {
            return ESP_ERR_INVALID_ARG;
        }
        return channels_[idx]->setDuty(duty);
    }

} // namespace adas