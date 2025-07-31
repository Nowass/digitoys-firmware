#include "bmi270_managed.hpp"
#include "i2c_bus.h"
#include <esp_log.h>
#include <math.h>

static const char *TAG = "BMI270_MANAGED";

namespace bmi270_managed
{

    // I2C interface functions for BMI270 managed component
    BMI2_INTF_RETURN_TYPE bmi2_i2c_read_impl(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
    {
        i2c_bus_device_handle_t device = (i2c_bus_device_handle_t)intf_ptr;
        esp_err_t ret = i2c_bus_read_bytes(device, reg_addr, len, reg_data);
        return (ret == ESP_OK) ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
    }

    BMI2_INTF_RETURN_TYPE bmi2_i2c_write_impl(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
    {
        i2c_bus_device_handle_t device = (i2c_bus_device_handle_t)intf_ptr;
        esp_err_t ret = i2c_bus_write_bytes(device, reg_addr, len, reg_data);
        return (ret == ESP_OK) ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
    }

    void bmi2_delay_us_impl(uint32_t period, void *intf_ptr)
    {
        // Convert microseconds to milliseconds for vTaskDelay
        uint32_t delay_ms = (period + 999) / 1000; // Round up
        if (delay_ms == 0)
        {
            delay_ms = 1; // Minimum delay
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    BMI270::BMI270()
        : initialized_(false), last_bmi2_error_(BMI2_OK), i2c_bus_(nullptr), i2c_device_(nullptr)
    {
        // Initialize BMI2 device structure
        memset(&bmi_dev_, 0, sizeof(bmi_dev_));
    }

    BMI270::~BMI270()
    {
        if (initialized_)
        {
            deinit();
        }
    }

    esp_err_t BMI270::init(const I2CConfig &config)
    {
        if (initialized_)
        {
            ESP_LOGW(TAG, "BMI270 already initialized");
            return ESP_OK;
        }

        config_ = config;

        // Initialize I2C bus
        esp_err_t ret = init_i2c();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
            return ret;
        }

        // Configure BMI2 device structure
        bmi_dev_.intf = BMI2_I2C_INTF;
        bmi_dev_.read = bmi2_i2c_read_impl;
        bmi_dev_.write = bmi2_i2c_write_impl;
        bmi_dev_.delay_us = bmi2_delay_us_impl;
        bmi_dev_.intf_ptr = i2c_device_; // Set device handle as interface pointer
        bmi_dev_.read_write_len = 46;    // Maximum read/write length
        bmi_dev_.config_file_ptr = NULL; // Use default config file

        // Initialize BMI270 sensor
        last_bmi2_error_ = bmi270_init(&bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "BMI270 initialization failed with error: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Verify chip ID
        uint8_t chip_id;
        ret = read_chip_id(&chip_id);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read chip ID");
            return ret;
        }

        if (chip_id != BMI270_CHIP_ID)
        {
            ESP_LOGE(TAG, "Invalid chip ID: 0x%02X, expected: 0x%02X", chip_id, BMI270_CHIP_ID);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "BMI270 initialized successfully, chip ID: 0x%02X", chip_id);
        initialized_ = true;

        return ESP_OK;
    }

    esp_err_t BMI270::deinit()
    {
        if (!initialized_)
        {
            return ESP_OK;
        }

        // Perform soft reset before deinitializing
        soft_reset();

        // Cleanup I2C resources
        if (i2c_device_ != nullptr)
        {
            i2c_bus_device_delete(&i2c_device_);
            i2c_device_ = nullptr;
        }

        if (i2c_bus_ != nullptr)
        {
            i2c_bus_delete(&i2c_bus_);
            i2c_bus_ = nullptr;
        }

        initialized_ = false;
        ESP_LOGI(TAG, "BMI270 deinitialized");

        return ESP_OK;
    }

    esp_err_t BMI270::read_chip_id(uint8_t *chip_id)
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (chip_id == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        last_bmi2_error_ = bmi2_get_regs(BMI2_CHIP_ID_ADDR, chip_id, 1, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to read chip ID: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        return ESP_OK;
    }

    esp_err_t BMI270::configure_accel(uint8_t range, uint8_t odr)
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        struct bmi2_sens_config config;
        config.type = BMI2_ACCEL;

        // Get current configuration
        last_bmi2_error_ = bmi2_get_sensor_config(&config, 1, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to get accel config: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Set new configuration
        config.cfg.acc.range = range;
        config.cfg.acc.odr = odr;
        config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
        config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        last_bmi2_error_ = bmi2_set_sensor_config(&config, 1, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to set accel config: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Accelerometer configured: range=%d, odr=%d", range, odr);
        return ESP_OK;
    }

    esp_err_t BMI270::configure_gyro(uint8_t range, uint8_t odr)
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        struct bmi2_sens_config config;
        config.type = BMI2_GYRO;

        // Get current configuration
        last_bmi2_error_ = bmi2_get_sensor_config(&config, 1, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to get gyro config: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Set new configuration
        config.cfg.gyr.range = range;
        config.cfg.gyr.odr = odr;
        config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
        config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
        config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        last_bmi2_error_ = bmi2_set_sensor_config(&config, 1, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to set gyro config: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Gyroscope configured: range=%d, odr=%d", range, odr);
        return ESP_OK;
    }

    esp_err_t BMI270::enable_sensors(bool enable_accel, bool enable_gyro)
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        uint8_t sensor_list[2];
        uint8_t n_sensors = 0;

        if (enable_accel)
        {
            sensor_list[n_sensors++] = BMI2_ACCEL;
        }
        if (enable_gyro)
        {
            sensor_list[n_sensors++] = BMI2_GYRO;
        }

        if (n_sensors > 0)
        {
            last_bmi2_error_ = bmi2_sensor_enable(sensor_list, n_sensors, &bmi_dev_);
            if (last_bmi2_error_ != BMI2_OK)
            {
                ESP_LOGE(TAG, "Failed to enable sensors: %d", last_bmi2_error_);
                return ESP_FAIL;
            }
        }

        ESP_LOGI(TAG, "Sensors enabled: accel=%s, gyro=%s",
                 enable_accel ? "true" : "false",
                 enable_gyro ? "true" : "false");
        return ESP_OK;
    }

    esp_err_t BMI270::read_accel_data(AccelData *data)
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (data == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        struct bmi2_sens_data sens_data;
        last_bmi2_error_ = bmi2_get_sensor_data(&sens_data, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to read sensor data: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Check if accelerometer data is ready
        if (!(sens_data.status & BMI2_DRDY_ACC))
        {
            ESP_LOGW(TAG, "Accelerometer data not ready");
            return ESP_ERR_NOT_FOUND;
        }

        // Get current accelerometer range for conversion
        struct bmi2_sens_config config;
        config.type = BMI2_ACCEL;
        last_bmi2_error_ = bmi2_get_sensor_config(&config, 1, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to get accel config for conversion: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Convert LSB to m/s²
        data->x = lsb_to_mps2(sens_data.acc.x, config.cfg.acc.range);
        data->y = lsb_to_mps2(sens_data.acc.y, config.cfg.acc.range);
        data->z = lsb_to_mps2(sens_data.acc.z, config.cfg.acc.range);

        return ESP_OK;
    }

    esp_err_t BMI270::read_gyro_data(GyroData *data)
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (data == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        struct bmi2_sens_data sens_data;
        last_bmi2_error_ = bmi2_get_sensor_data(&sens_data, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to read sensor data: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Check if gyroscope data is ready
        if (!(sens_data.status & BMI2_DRDY_GYR))
        {
            ESP_LOGW(TAG, "Gyroscope data not ready");
            return ESP_ERR_NOT_FOUND;
        }

        // Get current gyroscope range for conversion
        struct bmi2_sens_config config;
        config.type = BMI2_GYRO;
        last_bmi2_error_ = bmi2_get_sensor_config(&config, 1, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to get gyro config for conversion: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Convert LSB to rad/s
        data->x = lsb_to_rads(sens_data.gyr.x, config.cfg.gyr.range);
        data->y = lsb_to_rads(sens_data.gyr.y, config.cfg.gyr.range);
        data->z = lsb_to_rads(sens_data.gyr.z, config.cfg.gyr.range);

        return ESP_OK;
    }

    esp_err_t BMI270::read_sensor_data(SensorData *data)
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        if (data == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        // Initialize validity flags
        data->accel_valid = false;
        data->gyro_valid = false;
        data->temp_valid = false;

        struct bmi2_sens_data sens_data;
        last_bmi2_error_ = bmi2_get_sensor_data(&sens_data, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to read sensor data: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Get sensor configurations for conversion
        struct bmi2_sens_config configs[2];
        configs[0].type = BMI2_ACCEL;
        configs[1].type = BMI2_GYRO;
        last_bmi2_error_ = bmi2_get_sensor_config(configs, 2, &bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Failed to get sensor configs: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Process accelerometer data
        if (sens_data.status & BMI2_DRDY_ACC)
        {
            data->accel.x = lsb_to_mps2(sens_data.acc.x, configs[0].cfg.acc.range);
            data->accel.y = lsb_to_mps2(sens_data.acc.y, configs[0].cfg.acc.range);
            data->accel.z = lsb_to_mps2(sens_data.acc.z, configs[0].cfg.acc.range);
            data->accel_valid = true;
        }

        // Process gyroscope data
        if (sens_data.status & BMI2_DRDY_GYR)
        {
            data->gyro.x = lsb_to_rads(sens_data.gyr.x, configs[1].cfg.gyr.range);
            data->gyro.y = lsb_to_rads(sens_data.gyr.y, configs[1].cfg.gyr.range);
            data->gyro.z = lsb_to_rads(sens_data.gyr.z, configs[1].cfg.gyr.range);
            data->gyro_valid = true;
        }

        // Process temperature data (if available)
        // Note: BMI270 doesn't have a dedicated temperature ready flag,
        // so we'll read temperature data directly if needed
        uint8_t temp_data[2];
        last_bmi2_error_ = bmi2_get_regs(BMI2_TEMPERATURE_0_ADDR, temp_data, 2, &bmi_dev_);
        if (last_bmi2_error_ == BMI2_OK)
        {
            // Convert temperature LSB to Celsius
            // BMI270 temperature formula: temp_degC = 23 + (temp_lsb / 512)
            int16_t temp_lsb = (int16_t)((temp_data[1] << 8) | temp_data[0]);
            data->temperature = 23.0f + (temp_lsb / 512.0f);
            data->temp_valid = true;
        }

        return ESP_OK;
    }

    esp_err_t BMI270::soft_reset()
    {
        if (!initialized_)
        {
            return ESP_ERR_INVALID_STATE;
        }

        last_bmi2_error_ = bmi2_soft_reset(&bmi_dev_);
        if (last_bmi2_error_ != BMI2_OK)
        {
            ESP_LOGE(TAG, "Soft reset failed: %d", last_bmi2_error_);
            return ESP_FAIL;
        }

        // Wait for reset to complete
        vTaskDelay(pdMS_TO_TICKS(1));

        ESP_LOGI(TAG, "Soft reset completed");
        return ESP_OK;
    }

    esp_err_t BMI270::init_i2c()
    {
        // Use the i2c_bus component configuration approach
        // This avoids conflicts with driver/i2c.h types

        // Create I2C bus using default configuration for the specified port
        i2c_bus_ = i2c_bus_create((i2c_port_t)config_.port, NULL);
        if (i2c_bus_ == NULL)
        {
            ESP_LOGE(TAG, "Failed to create I2C bus");
            return ESP_FAIL;
        }

        // Create I2C device handle
        i2c_device_ = i2c_bus_device_create(i2c_bus_, config_.device_address, config_.clk_speed);
        if (i2c_device_ == NULL)
        {
            ESP_LOGE(TAG, "Failed to create I2C device");
            i2c_bus_delete(&i2c_bus_);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "I2C initialized: port=%d, speed=%lu, addr=0x%02X",
                 config_.port, config_.clk_speed, config_.device_address);

        return ESP_OK;
    }

    float BMI270::lsb_to_mps2(int16_t lsb, uint8_t range)
    {
        float g_range;
        switch (range)
        {
        case BMI2_ACC_RANGE_2G:
            g_range = 2.0f;
            break;
        case BMI2_ACC_RANGE_4G:
            g_range = 4.0f;
            break;
        case BMI2_ACC_RANGE_8G:
            g_range = 8.0f;
            break;
        case BMI2_ACC_RANGE_16G:
            g_range = 16.0f;
            break;
        default:
            g_range = 4.0f; // Default to 4G
        }

        // Convert LSB to g, then to m/s²
        float g_val = (float)lsb * g_range / 32768.0f; // 16-bit signed
        return g_val * 9.80665f;                       // Convert g to m/s²
    }

    float BMI270::lsb_to_rads(int16_t lsb, uint8_t range)
    {
        float dps_range;
        switch (range)
        {
        case BMI2_GYR_RANGE_125:
            dps_range = 125.0f;
            break;
        case BMI2_GYR_RANGE_250:
            dps_range = 250.0f;
            break;
        case BMI2_GYR_RANGE_500:
            dps_range = 500.0f;
            break;
        case BMI2_GYR_RANGE_1000:
            dps_range = 1000.0f;
            break;
        case BMI2_GYR_RANGE_2000:
            dps_range = 2000.0f;
            break;
        default:
            dps_range = 1000.0f; // Default to 1000 DPS
        }

        // Convert LSB to degrees per second, then to radians per second
        float dps_val = (float)lsb * dps_range / 32768.0f; // 16-bit signed
        return dps_val * (M_PI / 180.0f);                  // Convert degrees/s to radians/s
    }

} // namespace bmi270_managed
