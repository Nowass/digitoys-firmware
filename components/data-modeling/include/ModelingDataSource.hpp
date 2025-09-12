#pragma once

#include "IDataSource.hpp"
#include "DataModeling.hpp"

namespace digitoys::datamodeling
{
    /**
     * @brief IDataSource adapter that exposes DataModeling real-time readings
     *        to the DataLogger (for streaming to dashboard/CSV).
     */
    class ModelingDataSource : public digitoys::datalogger::IDataSource
    {
    public:
        explicit ModelingDataSource(DataModeling *modeling,
                                    uint32_t sample_rate_ms = 100,
                                    bool enabled = true)
            : modeling_(modeling)
        {
            config_.source_name = "Modeling";
            config_.sample_rate_ms = sample_rate_ms;
            config_.enabled = enabled;
            config_.max_buffer_size = 20;
        }

        ~ModelingDataSource() override = default;

        // IDataSource
        digitoys::datalogger::DataSourceConfig getSourceConfig() const override { return config_; }

        esp_err_t collectData(std::vector<digitoys::datalogger::DataEntry> &entries) override;

        bool isReady() const override { return modeling_ != nullptr; }

        esp_err_t onRegistered() override { return ESP_OK; }
        esp_err_t onUnregistered() override { return ESP_OK; }

    private:
        digitoys::datalogger::DataSourceConfig config_;
        DataModeling *modeling_ = nullptr; // non-owning
    };

} // namespace digitoys::datamodeling
