#pragma once

#include <driver/i2c.h>
#include <esp_err.h>

namespace bmi270_sensor {

class Bmi270Sensor {
public:
    esp_err_t init();
    esp_err_t read_accel(float &x, float &y, float &z);

private:
    static constexpr i2c_port_t PORT = I2C_NUM_0;
    static constexpr gpio_num_t SDA = GPIO_NUM_4;
    static constexpr gpio_num_t SCL = GPIO_NUM_5;
    static constexpr uint8_t ADDR = 0x68;

    float scale_ = 9.80665f * 2.0f / 32768.0f;

    esp_err_t write_reg(uint8_t reg, uint8_t value);
    esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len);
};

} // namespace bmi270_sensor
