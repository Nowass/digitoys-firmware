#include "bmi270_sensor.hpp"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace bmi270_sensor {

static const char *TAG = "BMI270";

namespace {
constexpr uint8_t REG_CHIP_ID    = 0x00;
constexpr uint8_t REG_CMD        = 0x7E;
constexpr uint8_t REG_ACC_CONF   = 0x40;
constexpr uint8_t REG_ACC_RANGE  = 0x41;
constexpr uint8_t REG_DATA_START = 0x12;
constexpr uint8_t CHIP_ID        = 0x24;
constexpr uint8_t CMD_SOFT_RESET = 0xB6;
} // namespace

esp_err_t Bmi270Sensor::write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(PORT, ADDR, buf, sizeof(buf),
                                      pdMS_TO_TICKS(100));
}

esp_err_t Bmi270Sensor::read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(PORT, ADDR, &reg, 1, data, len,
                                        pdMS_TO_TICKS(100));
}

esp_err_t Bmi270Sensor::init()
{
    i2c_config_t cfg{};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = SDA;
    cfg.scl_io_num = SCL;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = 400000;
    esp_err_t err = i2c_param_config(PORT, &cfg);
    if (err != ESP_OK)
        return err;
    err = i2c_driver_install(PORT, cfg.mode, 0, 0, 0);
    if (err != ESP_OK)
        return err;

    // allow sensor to power up
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t id = 0;
    err = read_regs(REG_CHIP_ID, &id, 1);
    if (err != ESP_OK)
        return err;
    if (id != CHIP_ID)
    {
        ESP_LOGE(TAG, "Unexpected chip id 0x%02X", id);
        return ESP_FAIL;
    }

    err = write_reg(REG_CMD, CMD_SOFT_RESET);
    if (err != ESP_OK)
        return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    err = write_reg(REG_ACC_CONF, 0x08); // 100 Hz
    if (err != ESP_OK)
        return err;
    err = write_reg(REG_ACC_RANGE, 0x01); // 4G
    if (err != ESP_OK)
        return err;
    scale_ = 9.80665f * 4.0f / 32768.0f;

    return ESP_OK;
}

esp_err_t Bmi270Sensor::read_accel(float &x, float &y, float &z)
{
    uint8_t buf[6];
    esp_err_t err = read_regs(REG_DATA_START, buf, sizeof(buf));
    if (err != ESP_OK)
        return err;
    int16_t raw_x = int16_t(buf[0] | (buf[1] << 8));
    int16_t raw_y = int16_t(buf[2] | (buf[3] << 8));
    int16_t raw_z = int16_t(buf[4] | (buf[5] << 8));
    x = raw_x * scale_;
    y = raw_y * scale_;
    z = raw_z * scale_;
    return ESP_OK;
}

} // namespace bmi270_sensor
