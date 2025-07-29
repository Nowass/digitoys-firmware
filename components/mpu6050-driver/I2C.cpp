#include "I2C.hpp"
#include <esp_log.h>
#include <string.h>

static const char *TAG = "I2C";

I2C::I2C(const Config &cfg) : cfg_(cfg) {}

esp_err_t I2C::init()
{
    if (initialized_)
    {
        return ESP_OK;
    }

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = cfg_.sda_pin;
    conf.scl_io_num = cfg_.scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = cfg_.frequency_hz;

    esp_err_t ret = i2c_param_config(cfg_.port, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(cfg_.port, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "I2C initialized on port %d, SDA: %d, SCL: %d, freq: %lu Hz",
             cfg_.port, cfg_.sda_pin, cfg_.scl_pin, cfg_.frequency_hz);
    return ESP_OK;
}

esp_err_t I2C::write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (len > 0)
    {
        i2c_master_write(cmd, data, len, true);
    }
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(cfg_.port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t I2C::read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    if (!initialized_)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);

    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(cfg_.port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t I2C::writeByte(uint8_t addr, uint8_t reg, uint8_t value)
{
    return write(addr, reg, &value, 1);
}

esp_err_t I2C::readByte(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return read(addr, reg, value, 1);
}
