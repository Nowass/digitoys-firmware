# BMI270 Managed Component

This component provides a modern C++ wrapper for the official BMI270 sensor API from Bosch Sensortec, distributed as an ESP-IDF managed component (`espressif2022/bmi270`).

## Features

- **Official BMI270 API**: Uses the official Bosch Sensortec BMI270 sensor API
- **Modern C++ Interface**: Clean, modern C++ wrapper with RAII principles
- **ESP-IDF Integration**: Seamless integration with ESP-IDF I2C bus component
- **Complete Sensor Support**: Full accelerometer and gyroscope functionality
- **Unit Conversion**: Automatic conversion to standard units (m/s² for acceleration, rad/s for angular velocity)
- **Error Handling**: Comprehensive error checking and logging
- **Thread-Safe**: Safe for use in multi-threaded applications

## Dependencies

This component requires the following managed components:
- `espressif2022/bmi270^1.1.0` - Official BMI270 sensor API
- `espressif/i2c_bus` - ESP-IDF I2C bus component

These dependencies are automatically resolved when you add this component to your project.

## Installation

Add this component to your ESP-IDF project by including it in your `components` directory, or add the managed dependencies to your main component:

```yaml
# In your idf_component.yml
dependencies:
  espressif2022/bmi270: "^1.1.0"
  espressif/i2c_bus: "*"
```

## Hardware Setup

Connect the BMI270 sensor to your ESP32:

```
BMI270    ESP32
------    -----
VCC   ->  3.3V
GND   ->  GND
SDA   ->  GPIO21 (configurable)
SCL   ->  GPIO22 (configurable)
SDO   ->  GND (for I2C address 0x68) or 3.3V (for address 0x69)
```

**Note**: Pull-up resistors (4.7kΩ) on SDA and SCL lines are recommended if not already present on your BMI270 module.

## Basic Usage

### Initialization

```cpp
#include "bmi270_managed.hpp"

using namespace bmi270_managed;

void app_main() {
    // Configure I2C
    I2CConfig config;
    config.port = I2C_NUM_0;
    config.sda_pin = GPIO_NUM_21;
    config.scl_pin = GPIO_NUM_22;
    config.clk_speed = 400000;  // 400kHz
    config.device_address = 0x68;  // BMI270 default address

    // Create and initialize BMI270
    BMI270 sensor;
    esp_err_t ret = sensor.init(config);
    if (ret == ESP_OK) {
        ESP_LOGI("APP", "BMI270 initialized successfully!");
    } else {
        ESP_LOGE("APP", "BMI270 initialization failed");
        return;
    }

    // Configure sensors
    sensor.configure_accel(BMI2_ACC_RANGE_4G, BMI2_ACC_ODR_100HZ);
    sensor.configure_gyro(BMI2_GYR_RANGE_1000DPS, BMI2_GYR_ODR_100HZ);
    
    // Enable sensors
    sensor.enable_sensors(true, true);  // Enable both accel and gyro
}
```

### Reading Sensor Data

```cpp
void read_sensor_data_example(BMI270& sensor) {
    SensorData data;
    esp_err_t ret = sensor.read_sensor_data(&data);
    
    if (ret == ESP_OK) {
        if (data.accel_valid) {
            ESP_LOGI("ACCEL", "X: %.2f m/s², Y: %.2f m/s², Z: %.2f m/s²", 
                     data.accel.x, data.accel.y, data.accel.z);
        }
        
        if (data.gyro_valid) {
            ESP_LOGI("GYRO", "X: %.2f rad/s, Y: %.2f rad/s, Z: %.2f rad/s", 
                     data.gyro.x, data.gyro.y, data.gyro.z);
        }
        
        if (data.temp_valid) {
            ESP_LOGI("TEMP", "Temperature: %.1f °C", data.temperature);
        }
    }
}
```

### Individual Sensor Reading

```cpp
void read_individual_sensors(BMI270& sensor) {
    // Read accelerometer data only
    AccelData accel;
    if (sensor.read_accel_data(&accel) == ESP_OK) {
        ESP_LOGI("ACCEL", "X: %.2f, Y: %.2f, Z: %.2f m/s²", accel.x, accel.y, accel.z);
    }

    // Read gyroscope data only
    GyroData gyro;
    if (sensor.read_gyro_data(&gyro) == ESP_OK) {
        ESP_LOGI("GYRO", "X: %.2f, Y: %.2f, Z: %.2f rad/s", gyro.x, gyro.y, gyro.z);
    }
}
```

## Configuration Options

### Accelerometer Ranges

- `BMI2_ACC_RANGE_2G` - ±2g range
- `BMI2_ACC_RANGE_4G` - ±4g range (default)
- `BMI2_ACC_RANGE_8G` - ±8g range
- `BMI2_ACC_RANGE_16G` - ±16g range

### Accelerometer Output Data Rates

- `BMI2_ACC_ODR_0_78HZ` - 0.78 Hz
- `BMI2_ACC_ODR_1_56HZ` - 1.56 Hz
- `BMI2_ACC_ODR_3_12HZ` - 3.12 Hz
- `BMI2_ACC_ODR_6_25HZ` - 6.25 Hz
- `BMI2_ACC_ODR_12_5HZ` - 12.5 Hz
- `BMI2_ACC_ODR_25HZ` - 25 Hz
- `BMI2_ACC_ODR_50HZ` - 50 Hz
- `BMI2_ACC_ODR_100HZ` - 100 Hz (default)
- `BMI2_ACC_ODR_200HZ` - 200 Hz
- `BMI2_ACC_ODR_400HZ` - 400 Hz
- `BMI2_ACC_ODR_800HZ` - 800 Hz
- `BMI2_ACC_ODR_1600HZ` - 1600 Hz

### Gyroscope Ranges

- `BMI2_GYR_RANGE_125` - ±125°/s
- `BMI2_GYR_RANGE_250` - ±250°/s
- `BMI2_GYR_RANGE_500` - ±500°/s
- `BMI2_GYR_RANGE_1000` - ±1000°/s (default)
- `BMI2_GYR_RANGE_2000` - ±2000°/s

### Gyroscope Output Data Rates

- `BMI2_GYR_ODR_25HZ` - 25 Hz
- `BMI2_GYR_ODR_50HZ` - 50 Hz
- `BMI2_GYR_ODR_100HZ` - 100 Hz (default)
- `BMI2_GYR_ODR_200HZ` - 200 Hz
- `BMI2_GYR_ODR_400HZ` - 400 Hz
- `BMI2_GYR_ODR_800HZ` - 800 Hz
- `BMI2_GYR_ODR_1600HZ` - 1600 Hz
- `BMI2_GYR_ODR_3200HZ` - 3200 Hz

## Advanced Usage

### Error Handling

```cpp
void advanced_error_handling(BMI270& sensor) {
    esp_err_t ret = sensor.read_sensor_data(&data);
    if (ret != ESP_OK) {
        ESP_LOGE("APP", "Failed to read sensor data: %s", esp_err_to_name(ret));
        
        // Get BMI270 specific error code
        int8_t bmi_error = sensor.get_last_error();
        ESP_LOGE("APP", "BMI270 error code: %d", bmi_error);
    }
}
```

### Chip ID Verification

```cpp
void verify_chip_id(BMI270& sensor) {
    uint8_t chip_id;
    if (sensor.read_chip_id(&chip_id) == ESP_OK) {
        ESP_LOGI("APP", "BMI270 Chip ID: 0x%02X", chip_id);
        if (chip_id == BMI270_CHIP_ID) {
            ESP_LOGI("APP", "BMI270 detected successfully");
        }
    }
}
```

### Soft Reset

```cpp
void reset_sensor(BMI270& sensor) {
    esp_err_t ret = sensor.soft_reset();
    if (ret == ESP_OK) {
        ESP_LOGI("APP", "BMI270 soft reset completed");
        // Reconfigure sensor after reset
        sensor.configure_accel();
        sensor.configure_gyro();
        sensor.enable_sensors(true, true);
    }
}
```

## API Reference

### Class: BMI270

#### Public Methods

- `esp_err_t init(const I2CConfig& config)` - Initialize the sensor
- `esp_err_t deinit()` - Deinitialize the sensor
- `bool is_initialized() const` - Check initialization status
- `esp_err_t read_chip_id(uint8_t* chip_id)` - Read chip ID
- `esp_err_t configure_accel(uint8_t range, uint8_t odr)` - Configure accelerometer
- `esp_err_t configure_gyro(uint8_t range, uint8_t odr)` - Configure gyroscope
- `esp_err_t enable_sensors(bool enable_accel, bool enable_gyro)` - Enable/disable sensors
- `esp_err_t read_accel_data(AccelData* data)` - Read accelerometer data
- `esp_err_t read_gyro_data(GyroData* data)` - Read gyroscope data
- `esp_err_t read_sensor_data(SensorData* data)` - Read all sensor data
- `esp_err_t soft_reset()` - Perform soft reset
- `int8_t get_last_error() const` - Get last BMI270 API error code

### Structures

#### I2CConfig
- `i2c_port_t port` - I2C port number
- `gpio_num_t sda_pin` - SDA GPIO pin
- `gpio_num_t scl_pin` - SCL GPIO pin
- `uint32_t clk_speed` - I2C clock speed in Hz
- `uint8_t device_address` - BMI270 I2C address

#### AccelData
- `float x, y, z` - Acceleration in m/s²

#### GyroData
- `float x, y, z` - Angular velocity in rad/s

#### SensorData
- `AccelData accel` - Accelerometer data
- `GyroData gyro` - Gyroscope data
- `float temperature` - Temperature in °C
- `bool accel_valid, gyro_valid, temp_valid` - Data validity flags

## Troubleshooting

### Common Issues

1. **Initialization fails**: Check I2C wiring and pull-up resistors
2. **Wrong chip ID**: Verify BMI270 is connected and powered correctly
3. **No data ready**: Ensure sensors are enabled and configured properly
4. **I2C errors**: Check I2C address (0x68 vs 0x69) and bus configuration

### Debug Logging

Enable debug logging by setting the log level:

```cpp
esp_log_set_level("BMI270_MANAGED", ESP_LOG_DEBUG);
```

## License

This component uses the official BMI270 sensor API from Bosch Sensortec, which is licensed under BSD-3-Clause. The wrapper code follows the same license terms.
