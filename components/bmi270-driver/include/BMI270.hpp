#pragma once
#include <driver/i2c.h>
#include <esp_err.h>
#include <functional>

namespace bmi270
{
    /// Configuration parameters for BMI270
    struct Config
    {
        i2c_port_t port = I2C_NUM_0;      ///< I2C controller
        gpio_num_t sda_pin = GPIO_NUM_NC; ///< SDA pin
        gpio_num_t scl_pin = GPIO_NUM_NC; ///< SCL pin
        uint32_t clk_speed_hz = 400000;   ///< I2C clock
        uint8_t address = 0x68;           ///< Device address (0x68 or 0x69)
        gpio_num_t int_pin = GPIO_NUM_NC; ///< Optional interrupt pin
        uint8_t accel_range = 0x00;       ///< Accel range setting
        uint16_t odr_hz = 100;            ///< Output data rate
    };

    class BMI270
    {
    public:
        explicit BMI270(const Config &cfg);
        esp_err_t init();
        esp_err_t readAccel(float &x, float &y, float &z);
        esp_err_t readGyro(float &x, float &y, float &z);
        esp_err_t readTemperature(float &t);

        void setDataReadyCallback(std::function<void()> cb);

    private:
        Config cfg_;
        bool initialized_ = false;
        std::function<void()> data_ready_cb_;

        esp_err_t i2cInit();
        esp_err_t writeReg(uint8_t reg, uint8_t value);
        esp_err_t readReg(uint8_t reg, uint8_t &value);
        esp_err_t readBytes(uint8_t reg, uint8_t *data, size_t len);
        static void IRAM_ATTR gpioIsr(void *arg);
    };

} // namespace bmi270
