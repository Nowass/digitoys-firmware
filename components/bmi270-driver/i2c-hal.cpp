#include "i2c-hal.hpp"
#include <esp_log.h>
#include <cstring>

namespace bmi270
{

    namespace
    {
        constexpr const char *TAG = "I2C_HAL";
        constexpr uint32_t I2C_ACK_CHECK_EN = 1;  // Enable ACK check for master
        constexpr uint32_t I2C_ACK_CHECK_DIS = 0; // Disable ACK check for master
    }

    I2C_HAL::~I2C_HAL()
    {
        deinit();
    }

    esp_err_t I2C_HAL::init(const I2CConfig &cfg)
    {
        if (initialized_)
        {
            ESP_LOGW(TAG, "I2C HAL already initialized, deinitializing first");
            deinit();
        }

        port_ = cfg.port;
        slaveAddr_ = cfg.slaveAddr;
        timeoutMs_ = cfg.timeoutMs;

        i2c_config_t i2cCfg = {};
        i2cCfg.mode = I2C_MODE_MASTER;
        i2cCfg.sda_io_num = cfg.sdaPin;
        i2cCfg.scl_io_num = cfg.sclPin;
        i2cCfg.sda_pullup_en = cfg.pullupEnable ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        i2cCfg.scl_pullup_en = cfg.pullupEnable ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        i2cCfg.master.clk_speed = cfg.clockSpeed;
        i2cCfg.clk_flags = 0; // Use default clock source

        ESP_LOGI(TAG, "Initializing I2C port %d with SDA=%d, SCL=%d, clock=%lu Hz, slave_addr=0x%02X",
                 port_, cfg.sdaPin, cfg.sclPin, cfg.clockSpeed, cfg.slaveAddr);

        esp_err_t err = i2c_param_config(port_, &i2cCfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
            return err;
        }

        err = i2c_driver_install(port_, I2C_MODE_MASTER, 0, 0, 0);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
            return err;
        }

        initialized_ = true;
        ESP_LOGI(TAG, "I2C HAL initialized successfully");
        return ESP_OK;
    }

    esp_err_t I2C_HAL::write(uint8_t regAddr, std::span<const std::byte> data) const
    {
        if (!initialized_)
        {
            ESP_LOGE(TAG, "I2C HAL not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        if (data.empty())
        {
            ESP_LOGW(TAG, "Empty data span provided for write");
            return ESP_ERR_INVALID_ARG;
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (cmd == nullptr)
        {
            ESP_LOGE(TAG, "Failed to create I2C command link");
            return ESP_ERR_NO_MEM;
        }

        // Start condition
        esp_err_t err = i2c_master_start(cmd);
        if (err != ESP_OK)
            goto cleanup;

        // Write slave address with write bit
        err = i2c_master_write_byte(cmd, (slaveAddr_ << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
        if (err != ESP_OK)
            goto cleanup;

        // Write register address
        err = i2c_master_write_byte(cmd, regAddr, I2C_ACK_CHECK_EN);
        if (err != ESP_OK)
            goto cleanup;

        // Write data
        err = i2c_master_write(cmd, reinterpret_cast<const uint8_t *>(data.data()), data.size(), I2C_ACK_CHECK_EN);
        if (err != ESP_OK)
            goto cleanup;

        // Stop condition
        err = i2c_master_stop(cmd);
        if (err != ESP_OK)
            goto cleanup;

        // Execute command
        err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(timeoutMs_));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
        }

    cleanup:
        i2c_cmd_link_delete(cmd);
        return err;
    }

    esp_err_t I2C_HAL::writeByte(uint8_t regAddr, uint8_t data) const
    {
        const std::byte byteData = static_cast<std::byte>(data);
        return write(regAddr, std::span<const std::byte>(&byteData, 1));
    }

    esp_err_t I2C_HAL::read(uint8_t regAddr, std::span<std::byte> buffer) const
    {
        if (!initialized_)
        {
            ESP_LOGE(TAG, "I2C HAL not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        if (buffer.empty())
        {
            ESP_LOGW(TAG, "Empty buffer provided for read");
            return ESP_ERR_INVALID_ARG;
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (cmd == nullptr)
        {
            ESP_LOGE(TAG, "Failed to create I2C command link");
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = ESP_OK;

        // Write register address first
        err = i2c_master_start(cmd);
        if (err != ESP_OK)
            goto cleanup;

        err = i2c_master_write_byte(cmd, (slaveAddr_ << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
        if (err != ESP_OK)
            goto cleanup;

        err = i2c_master_write_byte(cmd, regAddr, I2C_ACK_CHECK_EN);
        if (err != ESP_OK)
            goto cleanup;

        // Repeated start for read
        err = i2c_master_start(cmd);
        if (err != ESP_OK)
            goto cleanup;

        err = i2c_master_write_byte(cmd, (slaveAddr_ << 1) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
        if (err != ESP_OK)
            goto cleanup;

        // Read data
        if (buffer.size() > 1)
        {
            err = i2c_master_read(cmd, reinterpret_cast<uint8_t *>(buffer.data()), buffer.size() - 1, I2C_MASTER_ACK);
            if (err != ESP_OK)
                goto cleanup;
        }

        // Read last byte with NACK
        err = i2c_master_read_byte(cmd, reinterpret_cast<uint8_t *>(buffer.data()) + buffer.size() - 1, I2C_MASTER_NACK);
        if (err != ESP_OK)
            goto cleanup;

        err = i2c_master_stop(cmd);
        if (err != ESP_OK)
            goto cleanup;

        // Execute command
        err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(timeoutMs_));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
        }

    cleanup:
        i2c_cmd_link_delete(cmd);
        return err;
    }

    esp_err_t I2C_HAL::readByte(uint8_t regAddr, uint8_t &data) const
    {
        std::byte byteData;
        esp_err_t err = read(regAddr, std::span<std::byte>(&byteData, 1));
        if (err == ESP_OK)
        {
            data = static_cast<uint8_t>(byteData);
        }
        return err;
    }

    esp_err_t I2C_HAL::ping() const
    {
        if (!initialized_)
        {
            ESP_LOGE(TAG, "I2C HAL not initialized");
            return ESP_ERR_INVALID_STATE;
        }

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (cmd == nullptr)
        {
            ESP_LOGE(TAG, "Failed to create I2C command link");
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = i2c_master_start(cmd);
        if (err != ESP_OK)
            goto cleanup;

        err = i2c_master_write_byte(cmd, (slaveAddr_ << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
        if (err != ESP_OK)
            goto cleanup;

        err = i2c_master_stop(cmd);
        if (err != ESP_OK)
            goto cleanup;

        err = i2c_master_cmd_begin(port_, cmd, pdMS_TO_TICKS(timeoutMs_));
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "Device ping successful for address 0x%02X", slaveAddr_);
        }
        else
        {
            ESP_LOGW(TAG, "Device ping failed for address 0x%02X: %s", slaveAddr_, esp_err_to_name(err));
        }

    cleanup:
        i2c_cmd_link_delete(cmd);
        return err;
    }

    void I2C_HAL::deinit()
    {
        if (initialized_)
        {
            esp_err_t err = i2c_driver_delete(port_);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to delete I2C driver: %s", esp_err_to_name(err));
            }
            else
            {
                ESP_LOGI(TAG, "I2C HAL deinitialized successfully");
            }
            initialized_ = false;
        }
    }

} // namespace bmi270
