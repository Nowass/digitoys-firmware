#include "I2C.hpp"
#include <string.h>

I2C::I2C(const Config &cfg) : cfg_(cfg) {}

esp_err_t I2C::init()
{
    if (initialized_)
        return ESP_OK;

    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = cfg_.sda_pin;
    conf.scl_io_num = cfg_.scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = cfg_.frequency_hz;
    esp_err_t err = i2c_param_config(cfg_.port, &conf);
    if (err != ESP_OK)
        return err;
    err = i2c_driver_install(cfg_.port, conf.mode, 0, 0, 0);
    if (err == ESP_OK)
        initialized_ = true;
    return err;
}

esp_err_t I2C::write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (len)
        i2c_master_write(cmd, const_cast<uint8_t *>(data), len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(cfg_.port, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t I2C::read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(cfg_.port, addr, &reg, 1, data, len,
                                        pdMS_TO_TICKS(20));
}
