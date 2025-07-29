# MPU6050 Driver Component

This component provides a modern C++ driver for the InvenSense MPU6050 6-axis IMU (Inertial Measurement Unit) sensor for ESP32 projects.

## Features

- **6-axis motion sensing**: 3-axis accelerometer + 3-axis gyroscope
- **Temperature sensor**: Built-in temperature measurement
- **Configurable ranges**: 
  - Accelerometer: ±2g, ±4g, ±8g, ±16g
  - Gyroscope: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **Digital Low-Pass Filter (DLPF)**: Configurable from 260Hz to 5Hz
- **IIR filtering**: Software-based noise reduction
- **Self-test capability**: Built-in sensor validation
- **Interrupt support**: Data ready interrupt
- **Efficient bulk read**: Read all sensor data in a single I2C transaction

## Hardware Connection

| MPU6050 Pin | ESP32 Pin | Description |
|-------------|-----------|-------------|
| VCC         | 3.3V      | Power supply |
| GND         | GND       | Ground |
| SDA         | GPIO4     | I2C data line |
| SCL         | GPIO5     | I2C clock line |
| INT         | GPIO (optional) | Interrupt pin |

## Usage

### Basic Usage

```cpp
#include "mpu6050_example.hpp"

// In your task or main function
MPU6050Example::Example mpu6050_example;

// Initialize
esp_err_t ret = mpu6050_example.initialize();
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize MPU6050");
    return;
}

// Perform self-test
ret = mpu6050_example.self_test();
if (ret != ESP_OK) {
    ESP_LOGW(TAG, "MPU6050 self-test failed");
}

// Read and log data continuously
while (true) {
    ret = mpu6050_example.read_and_log_data();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050 data");
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Read every 100ms
}
```

### Advanced Usage

```cpp
#include "MPU6050.hpp"
#include "I2C.hpp"

// Configure I2C bus
I2C::Config i2c_config = {
    .port = I2C_NUM_0,
    .sda_pin = GPIO_NUM_4,
    .scl_pin = GPIO_NUM_5,
    .frequency_hz = 400000
};
I2C i2c_bus(i2c_config);

// Configure MPU6050
MPU6050::Config mpu_config = {
    .accelRange = MPU6050::Config::ACCEL_RANGE_8G,
    .gyroRange = MPU6050::Config::GYRO_RANGE_500,
    .dlpf = MPU6050::Config::DLPF_44HZ,
    .sampleRateDiv = 19,  // 50Hz sample rate
    .filterAlpha = 0.2f,
    .i2c_address = 0x68
};

MPU6050 sensor(i2c_bus, mpu_config);

// Initialize
i2c_bus.init();
sensor.init();

// Read individual values
float accelX = sensor.getAccelX();
float gyroZ = sensor.getGyroZ();
float temperature = sensor.getTemperature();

// Read all data at once (more efficient)
MPU6050::SensorData data;
if (sensor.readAll(data)) {
    printf("Accel: %.2f, %.2f, %.2f m/s²\\n", 
           data.accelX, data.accelY, data.accelZ);
    printf("Gyro: %.2f, %.2f, %.2f deg/s\\n", 
           data.gyroX, data.gyroY, data.gyroZ);
    printf("Temperature: %.2f °C\\n", data.temperature);
}
```

## Configuration Options

### Accelerometer Ranges
- `ACCEL_RANGE_2G`: ±2g (most sensitive)
- `ACCEL_RANGE_4G`: ±4g
- `ACCEL_RANGE_8G`: ±8g (default)
- `ACCEL_RANGE_16G`: ±16g (least sensitive)

### Gyroscope Ranges
- `GYRO_RANGE_250`: ±250°/s (most sensitive)
- `GYRO_RANGE_500`: ±500°/s (default)
- `GYRO_RANGE_1000`: ±1000°/s
- `GYRO_RANGE_2000`: ±2000°/s (least sensitive)

### Digital Low-Pass Filter
- `DLPF_260HZ`: 260Hz bandwidth
- `DLPF_184HZ`: 184Hz bandwidth
- `DLPF_94HZ`: 94Hz bandwidth
- `DLPF_44HZ`: 44Hz bandwidth (default)
- `DLPF_21HZ`: 21Hz bandwidth
- `DLPF_10HZ`: 10Hz bandwidth
- `DLPF_5HZ`: 5Hz bandwidth

### Sample Rate
The sample rate is calculated as: `Sample Rate = 1kHz / (1 + sampleRateDiv)`
- `sampleRateDiv = 0`: 1000Hz
- `sampleRateDiv = 9`: 100Hz
- `sampleRateDiv = 19`: 50Hz (default)

## API Reference

### MPU6050 Class

#### Public Methods
- `bool init()`: Initialize the sensor
- `bool selfTest()`: Perform self-test
- `bool dataReady()`: Check if new data is available
- `float getAccelX/Y/Z()`: Get individual acceleration values (m/s²)
- `float getGyroX/Y/Z()`: Get individual gyroscope values (deg/s)
- `float getTemperature()`: Get temperature (°C)
- `bool readAll(SensorData &data)`: Read all sensor data at once

#### SensorData Structure
```cpp
struct SensorData {
    float accelX, accelY, accelZ;    // Acceleration in m/s²
    float gyroX, gyroY, gyroZ;       // Angular velocity in deg/s
    float temperature;               // Temperature in °C
};
```

## Error Handling

The driver uses ESP-IDF's error handling conventions:
- `ESP_OK`: Success
- `ESP_FAIL`: Generic failure
- `ESP_ERR_INVALID_STATE`: Driver not initialized
- `ESP_ERR_INVALID_ARG`: Invalid argument

## Thread Safety

The driver is **not** thread-safe. If you need to access the sensor from multiple tasks, implement your own synchronization using mutexes or semaphores.

## Performance Considerations

- Use `readAll()` instead of individual getters for better performance
- The IIR filter adds minimal computational overhead
- I2C transactions are blocking operations
- Consider using interrupts for data-ready signaling in high-frequency applications

## Troubleshooting

### Sensor Not Detected
1. Check wiring connections
2. Verify I2C address (0x68 or 0x69)
3. Ensure proper power supply (3.3V)
4. Check I2C pull-up resistors

### Noisy Data
1. Lower the DLPF frequency
2. Adjust the `filterAlpha` parameter
3. Check for electromagnetic interference
4. Ensure stable power supply

### Self-Test Failure
1. Check if sensor is properly mounted
2. Ensure sensor is not moving during self-test
3. Verify power supply stability
4. Check for hardware damage

## Dependencies

- ESP-IDF driver component
- FreeRTOS (for delays and task management)
- Standard C++ library

## License

This component is part of the digitoys-firmware project.
