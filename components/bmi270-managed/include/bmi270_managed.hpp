#pragma once

#include "bmi270.h"
#include "bmi2.h"
#include "i2c_bus.h"
#include <esp_err.h>
#include <driver/gpio.h>
#include <cstdint>

namespace bmi270_managed
{

    /**
     * @brief I2C configuration structure for BMI270
     */
    struct I2CConfig
    {
        int port = 0;                                ///< I2C port number (0 or 1)
        gpio_num_t sda_pin = GPIO_NUM_4;             ///< SDA GPIO pin
        gpio_num_t scl_pin = GPIO_NUM_5;             ///< SCL GPIO pin
        uint32_t clk_speed = 400000;                 ///< I2C clock speed (400kHz)
        uint8_t device_address = BMI270_I2C_ADDRESS; ///< BMI270 I2C address (0x68)
    };

    /**
     * @brief Accelerometer data structure
     */
    struct AccelData
    {
        float x; ///< X-axis acceleration in m/s²
        float y; ///< Y-axis acceleration in m/s²
        float z; ///< Z-axis acceleration in m/s²
    };

    /**
     * @brief Gyroscope data structure
     */
    struct GyroData
    {
        float x; ///< X-axis angular velocity in rad/s
        float y; ///< Y-axis angular velocity in rad/s
        float z; ///< Z-axis angular velocity in rad/s
    };

    /**
     * @brief Combined sensor data structure
     */
    struct SensorData
    {
        AccelData accel;   ///< Accelerometer data
        GyroData gyro;     ///< Gyroscope data
        float temperature; ///< Temperature in degrees Celsius
        bool accel_valid;  ///< Accelerometer data validity flag
        bool gyro_valid;   ///< Gyroscope data validity flag
        bool temp_valid;   ///< Temperature data validity flag
    };

    /**
     * @brief C++ wrapper class for BMI270 managed component
     *
     * This class provides a modern C++ interface to the official BMI270 sensor API
     * from Bosch Sensortec, distributed as an ESP-IDF managed component.
     */
    class BMI270
    {
    public:
        /**
         * @brief Constructor
         */
        BMI270();

        /**
         * @brief Destructor
         */
        ~BMI270();

        /**
         * @brief Initialize the BMI270 sensor
         *
         * @param config I2C configuration
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t init(const I2CConfig &config);

        /**
         * @brief Deinitialize the BMI270 sensor and I2C bus
         *
         * @return ESP_OK on success
         */
        esp_err_t deinit();

        /**
         * @brief Check if the sensor is initialized
         *
         * @return true if initialized, false otherwise
         */
        bool is_initialized() const { return initialized_; }

        /**
         * @brief Read chip ID
         *
         * @param chip_id Pointer to store chip ID
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t read_chip_id(uint8_t *chip_id);

        /**
         * @brief Configure accelerometer settings
         *
         * @param range Accelerometer range (BMI2_ACC_RANGE_2G, BMI2_ACC_RANGE_4G, BMI2_ACC_RANGE_8G, BMI2_ACC_RANGE_16G)
         * @param odr Output data rate (BMI2_ACC_ODR_25HZ, BMI2_ACC_ODR_50HZ, BMI2_ACC_ODR_100HZ, BMI2_ACC_ODR_200HZ, etc.)
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t configure_accel(uint8_t range = BMI2_ACC_RANGE_4G, uint8_t odr = BMI2_ACC_ODR_100HZ);

        /**
         * @brief Configure gyroscope settings
         *
         * @param range Gyroscope range (BMI2_GYR_RANGE_125, BMI2_GYR_RANGE_250, BMI2_GYR_RANGE_500, BMI2_GYR_RANGE_1000, BMI2_GYR_RANGE_2000)
         * @param odr Output data rate (BMI2_GYR_ODR_25HZ, BMI2_GYR_ODR_50HZ, BMI2_GYR_ODR_100HZ, BMI2_GYR_ODR_200HZ, etc.)
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t configure_gyro(uint8_t range = BMI2_GYR_RANGE_1000, uint8_t odr = BMI2_GYR_ODR_100HZ);

        /**
         * @brief Enable sensors
         *
         * @param enable_accel Enable accelerometer
         * @param enable_gyro Enable gyroscope
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t enable_sensors(bool enable_accel = true, bool enable_gyro = true);

        /**
         * @brief Read accelerometer data
         *
         * @param data Pointer to store accelerometer data
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t read_accel_data(AccelData *data);

        /**
         * @brief Read gyroscope data
         *
         * @param data Pointer to store gyroscope data
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t read_gyro_data(GyroData *data);

        /**
         * @brief Read all sensor data
         *
         * @param data Pointer to store combined sensor data
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t read_sensor_data(SensorData *data);

        /**
         * @brief Perform soft reset
         *
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t soft_reset();

        /**
         * @brief Get last error code from BMI270 API
         *
         * @return Last BMI2 error code
         */
        int8_t get_last_error() const { return last_bmi2_error_; }

    private:
        struct bmi2_dev bmi_dev_;            ///< BMI2 device structure
        I2CConfig config_;                   ///< I2C configuration
        bool initialized_;                   ///< Initialization status
        int8_t last_bmi2_error_;             ///< Last BMI2 API error code
        i2c_bus_handle_t i2c_bus_;           ///< I2C bus handle
        i2c_bus_device_handle_t i2c_device_; ///< I2C device handle

        /**
         * @brief Initialize I2C bus
         *
         * @return ESP_OK on success, error code on failure
         */
        esp_err_t init_i2c();

        /**
         * @brief Convert LSB to m/s² for accelerometer
         *
         * @param lsb Raw LSB value
         * @param range Accelerometer range setting
         * @return Acceleration in m/s²
         */
        float lsb_to_mps2(int16_t lsb, uint8_t range);

        /**
         * @brief Convert LSB to rad/s for gyroscope
         *
         * @param lsb Raw LSB value
         * @param range Gyroscope range setting
         * @return Angular velocity in rad/s
         */
        float lsb_to_rads(int16_t lsb, uint8_t range);
    };

} // namespace bmi270_managed
