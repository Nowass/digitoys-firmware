#include "BMI270.hpp"
#include <driver/gpio.h>
#include <esp_log.h>

using namespace bmi270;

static const char *TAG = "BMI270";

BMI270::BMI270(const Config &cfg) : cfg_(cfg) {}

esp_err_t BMI270::i2cInit()
{
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = cfg_.sda_pin;
    conf.scl_io_num = cfg_.scl_pin;
    conf.master.clk_speed = cfg_.clk_speed_hz;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(i2c_param_config(cfg_.port, &conf));
    return i2c_driver_install(cfg_.port, conf.mode, 0, 0, 0);
}

esp_err_t BMI270::init()
{
    esp_err_t ret = i2cInit();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Example configuration using BMI270 SensorAPI style registers
    // Reset device
    writeReg(0x7E, 0xB6); // CMD: soft-reset
    vTaskDelay(pdMS_TO_TICKS(2));

    // Load config file (required for BMI270)
    // Typically this is done via bmi270_write_config_file from the API.
    // Here we assume config file array bmi270_config_file[] is linked in.
    extern const uint8_t bmi270_config_file[];   // from SensorAPI
    extern const size_t bmi270_config_file_len;  // from SensorAPI
    writeReg(0x5E, 0x00); // init start
    for (size_t i = 0; i < bmi270_config_file_len; ++i)
    {
        writeReg(0x5E, bmi270_config_file[i]);
    }
    writeReg(0x5E, 0x00); // init end

    // Configure accelerometer ODR and range
    writeReg(0x40, cfg_.accel_range); // ACC_CONF
    writeReg(0x42, (cfg_.odr_hz / 25));

    // Map data ready interrupt if pin provided
    if (cfg_.int_pin != GPIO_NUM_NC)
    {
        gpio_config_t io_conf{};
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL << cfg_.int_pin;
        gpio_config(&io_conf);
        gpio_set_intr_type(cfg_.int_pin, GPIO_INTR_NEGEDGE);
        gpio_isr_handler_add(cfg_.int_pin, gpioIsr, this);
    }

    initialized_ = true;
    ESP_LOGI(TAG, "BMI270 initialized");
    return ESP_OK;
}

void BMI270::setDataReadyCallback(std::function<void()> cb)
{
    data_ready_cb_ = std::move(cb);
}

void IRAM_ATTR BMI270::gpioIsr(void *arg)
{
    auto *self = static_cast<BMI270 *>(arg);
    if (self->data_ready_cb_)
        self->data_ready_cb_();
}

esp_err_t BMI270::writeReg(uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg_.address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(cfg_.port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t BMI270::readReg(uint8_t reg, uint8_t &value)
{
    return readBytes(reg, &value, 1);
}

esp_err_t BMI270::readBytes(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg_.address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (cfg_.address << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(cfg_.port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t BMI270::readAccel(float &x, float &y, float &z)
{
    uint8_t buf[6];
    esp_err_t ret = readBytes(0x12, buf, 6); // ACC_DATA
    if (ret != ESP_OK)
        return ret;
    int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
    float scale = 9.81f / 16384.0f; // for ±2g
    x = ax * scale;
    y = ay * scale;
    z = az * scale;
    return ESP_OK;
}

esp_err_t BMI270::readGyro(float &x, float &y, float &z)
{
    uint8_t buf[6];
    esp_err_t ret = readBytes(0x0C, buf, 6); // GYR_DATA
    if (ret != ESP_OK)
        return ret;
    int16_t gx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t gy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t gz = (int16_t)((buf[5] << 8) | buf[4]);
    float scale = 2000.0f / 32768.0f; // ±2000dps
    x = gx * scale;
    y = gy * scale;
    z = gz * scale;
    return ESP_OK;
}

esp_err_t BMI270::readTemperature(float &t)
{
    uint8_t buf[2];
    esp_err_t ret = readBytes(0x22, buf, 2); // TEMP_DATA
    if (ret != ESP_OK)
        return ret;
    int16_t raw = (int16_t)((buf[1] << 8) | buf[0]);
    t = raw / 512.0f + 23.0f; // from datasheet
    return ESP_OK;
}

